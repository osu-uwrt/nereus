// Detached `ros2 bag record` control: the shell scripts that start, poll and stop a recording (locally or over
// ssh), the parser for their NEREUS_BAG reports, and a poll-based process runner with a timeout.
#include "nereus/ros_viewer/panels/bag_recorder.hpp"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sstream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;

namespace nereus::ros_viewer::panels {
namespace {

// Shared by every script. `active` describes the running recording, `last` the most recent one that ended,
// `stopping` marks a recorder told to finish. Values are reported one per line, newlines flattened.
constexpr const char *preamble = R"SH(
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/nereus/bag"
active="$state_dir/active"
report() { printf 'NEREUS_BAG %s %s\n' "$1" "$(printf '%s' "$2" | tr '\r\n' '  ')"; }
expand_dir() { case "$1" in "~") printf '%s' "$HOME" ;; "~/"*) printf '%s/%s' "$HOME" "${1#"~/"}" ;; *) printf '%s' "$1" ;; esac; }
alive() { [ -n "$1" ] && kill -0 "$1" 2>/dev/null && tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q 'bag record'; }
size_of() { du -sb "$1" 2>/dev/null | cut -f1; }
free_of() {
  d="$1"
  while [ ! -d "$d" ] && [ "$d" != / ] && [ -n "$d" ]; do d=$(dirname "$d"); done
  df -PB1 "$d" 2>/dev/null | awk 'NR==2 {print $4}'
}
read_active() {
  pid= bag= start= log=
  [ -f "$active" ] || return 1
  while IFS='=' read -r key value; do
    case "$key" in pid) pid=$value ;; bag) bag=$value ;; start) start=$value ;; log) log=$value ;; esac
  done < "$active"
}
# Moves the active recording to `last`: $1 its end (stopped, died), $2 a note.
finish() {
  {
    echo "state=$1"; echo "bag=$bag"; echo "bytes=$(size_of "$bag")"; echo "end=$(date +%s)"
    if [ -n "$2" ]; then printf 'note=%s\n' "$(printf '%s' "$2" | tr '\r\n' '  ')"; fi
    if [ -f "$bag/metadata.yaml" ]; then echo "complete=1"; fi
  } > "$state_dir/last.tmp" && mv -f "$state_dir/last.tmp" "$state_dir/last"
  rm -f "$active" "$state_dir/stopping"
}
log_tail() { [ -f "$1" ] && tail -n 4 "$1" | cut -c1-300; }
report_current() {
  report pid "$pid"; report bag "$bag"; report start "$start"; report now "$(date +%s)"; report bytes "$(size_of "$bag")"
}
report_last() {
  [ -f "$state_dir/last" ] || return 0
  while IFS='=' read -r key value; do report "last_$key" "$value"; done < "$state_dir/last"
}
# Ends every script: the active recording's state (once it is known to have ended), the last one and the disk.
conclude() {
  if read_active; then
    if alive "$pid"; then
      if [ -f "$state_dir/stopping" ]; then report state stopping; else report state recording; fi
      report_current
    elif [ -f "$state_dir/stopping" ]; then
      finish stopped ""; report state stopped
    else
      finish died "$(log_tail "$log")"; report state died
    fi
  else
    report state idle
  fi
  report_last
  report free "$(free_of "$(expand_dir "$directory")")"
}
fail() { report state error; report error "$1"; report_last; exit 0; }
)SH";

// Every script starts with the (quoted) bag directory followed by the shared shell functions.
std::string header(const BagHost &host) {
    return "directory=" + shellQuote(host.directory) + "\n" + preamble;
}

} // namespace

double BagReport::number(const std::string &key, double fallback) const {
    const auto text = get(key);
    if (text.empty())
        return fallback;
    char *end = nullptr;
    const double value = std::strtod(text.c_str(), &end);
    return end && *end == '\0' && std::isfinite(value) ? value : fallback;
}

// Keeps only the "NEREUS_BAG <key> <value>" lines; a key with no value maps to "".
BagReport parseBagReport(const std::string &output) {
    BagReport report;
    std::istringstream lines(output);
    std::string line;
    const std::string tag = "NEREUS_BAG ";
    while (std::getline(lines, line)) {
        if (line.compare(0, tag.size(), tag) != 0)
            continue;
        const auto rest = line.substr(tag.size());
        const auto space = rest.find(' ');
        if (space == std::string::npos)
            report.values[rest] = "";
        else
            report.values[rest.substr(0, space)] = rest.substr(space + 1);
    }
    return report;
}

std::string shellQuote(const std::string &value) {
    std::string quoted = "'";
    for (const char c : value)
        quoted += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return quoted + "'";
}

std::string bagStartScript(const BagHost &host, const BagRequest &request) {
    // The record command line, every argument quoted.
    std::string record = "ros2 bag record -o \"$bag\"";
    for (const auto &arg : host.recordArgs)
        record += " " + shellQuote(arg);
    if (!request.exclude.empty())
        record += " -x " + shellQuote(request.exclude);
    if (request.all)
        record += " -a";
    else
        for (const auto &topic : request.topics)
            record += " " + shellQuote(topic);

    // Clear a stale active record, resolve the bag path and refuse to overwrite an existing bag.
    std::string script = header(host);
    script += "name=" + shellQuote(request.name) + "\n";
    script += R"SH(
mkdir -p "$state_dir" || fail "cannot create $state_dir"
if read_active; then
  if alive "$pid"; then fail "already recording $bag"; fi
  if [ -f "$state_dir/stopping" ]; then finish stopped ""; else finish died "$(log_tail "$log")"; fi
fi
dir=$(expand_dir "$directory")
mkdir -p "$dir" || fail "cannot create $dir"
dir=$(cd "$dir" && pwd -P)
bag="$dir/$name"
[ -e "$bag" ] && fail "$bag already exists"
{
:
)SH";
    // The host's setup lines go inside the { ... } group opened above; its stderr goes to setup.log.
    script += host.setup;
    script += R"SH(
} >/dev/null 2>"$state_dir/setup.log" </dev/null
command -v ros2 >/dev/null 2>&1 || fail "ros2 not found after the setup. $(log_tail "$state_dir/setup.log")"
command -v python3 >/dev/null 2>&1 || fail "python3 not found"
log="$state_dir/record.log"
# A background job of a non-interactive shell starts with SIGINT ignored, and Python only turns SIGINT into a clean
# shutdown when it starts at the default: reset it on the way, so Stop closes the bag as Ctrl-C does.
setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
)SH";
    script += "  " + record + " </dev/null >\"$log\" 2>&1 &\n";

    // Record the recorder in the active file, then give it ~1.5 s to show it did not exit at once.
    script += R"SH(pid=$!
start=$(date +%s)
printf 'pid=%s\nbag=%s\nstart=%s\nlog=%s\n' "$pid" "$bag" "$start" "$log" > "$active.tmp" && mv -f "$active.tmp" "$active"
for i in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15; do alive "$pid" || break; sleep 0.1; done
if ! alive "$pid"; then
  note=$(log_tail "$log")
  finish died "$note"
  fail "ros2 bag record exited at once. $note"
fi
conclude
)SH";
    return script;
}

std::string bagStatusScript(const BagHost &host) {
    return header(host) + "conclude\n";
}

std::string bagStopScript(const BagHost &host, double waitSeconds, bool kill) {
    // The scripts poll every 0.1 s, so `steps` is the wait in tenths of a second (at least one).
    std::string script = header(host);
    script += "steps=" + std::to_string(std::max(1, int(std::lround(waitSeconds * 10)))) + "\n";
    if (kill)
        script += R"SH(
if read_active && alive "$pid"; then
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
  i=0; while alive "$pid" && [ $i -lt 20 ]; do sleep 0.1; i=$((i + 1)); done
  finish stopped "killed before it closed the bag: ros2 bag reindex can rebuild its metadata"
  report state stopped; report_last; report free "$(free_of "$(expand_dir "$directory")")"; exit 0
fi
conclude
)SH";
    else // a second SIGINT could cut the recorder's own shutdown short: only the first stop sends one
        script += R"SH(
if read_active && alive "$pid"; then
  if [ ! -f "$state_dir/stopping" ]; then touch "$state_dir/stopping"; kill -INT "$pid"; fi
  i=0; while alive "$pid" && [ $i -lt $steps ]; do sleep 0.1; i=$((i + 1)); done
fi
conclude
)SH";
    return script;
}

std::vector<std::string> bagCommand(const BagHost &host, const std::string &script) {
    if (host.host.empty())
        return {"bash", "-c", script};
    std::vector<std::string> argv = host.ssh;

    // Never prompt, give up fast on a dead link, and reuse one master connection for the frequent status polls.
    for (const char *option : {"BatchMode=yes", "ConnectTimeout=5", "ServerAliveInterval=2", "ServerAliveCountMax=2",
                               "StrictHostKeyChecking=accept-new", "ControlMaster=auto", "ControlPath=~/.ssh/nereus-%C",
                               "ControlPersist=60"}) {
        argv.push_back("-o");
        argv.push_back(option);
    }
    argv.push_back("-T");
    argv.push_back(host.host);
    argv.push_back("bash -c " + shellQuote(script));
    return argv;
}

std::string bagName(const std::string &prefix, bool timestamp) {
    if (!timestamp)
        return prefix;
    char text[32];
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::strftime(text, sizeof(text), "%Y%m%d_%H%M%S", &local);
    return prefix.empty() ? std::string(text) : prefix + "_" + text;
}

bool validBagName(const std::string &name) {
    if (name.empty() || name == "." || name == ".." || name.size() > 200)
        return false;
    for (const char c : name)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' || c == '-'))
            return false;
    return true;
}

std::string byteSize(double bytes) {
    const char *units[] = {"B", "KB", "MB", "GB", "TB"};
    int unit = 0;
    while (bytes >= 1000 && unit < 4) {
        bytes /= 1000;
        ++unit;
    }
    char text[32];
    std::snprintf(text, sizeof(text), unit == 0 || bytes >= 100 ? "%.0f %s" : "%.1f %s", bytes, units[unit]);
    return text;
}

ProcessResult runProcess(const std::vector<std::string> &argv, double timeoutSeconds, const std::atomic<bool> *cancel) {
    ProcessResult result;
    if (argv.empty())
        return result;

    // One pipe takes both stdout and stderr; stdin is /dev/null.
    int out[2];
    if (pipe2(out, O_CLOEXEC) != 0) {
        result.output = std::string("pipe: ") + std::strerror(errno);
        return result;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&actions, out[1], 1);
    posix_spawn_file_actions_adddup2(&actions, out[1], 2);
#if defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 34))
    posix_spawn_file_actions_addclosefrom_np(&actions, 3); // ROS and GL descriptors stay with the viewer
#endif
    // Its own process group (a timeout kills ssh with whatever it started), default signals and an empty mask
    // whatever the calling thread had.
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    sigset_t none, defaults;
    sigemptyset(&none);
    sigfillset(&defaults);
    posix_spawnattr_setsigmask(&attributes, &none);
    posix_spawnattr_setsigdefault(&attributes, &defaults);
    posix_spawnattr_setpgroup(&attributes, 0);
    posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);
    std::vector<char *> args;
    for (const auto &arg : argv)
        args.push_back(const_cast<char *>(arg.c_str()));
    args.push_back(nullptr);
    pid_t pid = -1;
    const int spawned = posix_spawnp(&pid, args[0], &actions, &attributes, args.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attributes);
    close(out[1]); // the child holds the only write end now
    if (spawned != 0) {
        close(out[0]);
        result.output = argv[0] + ": " + std::strerror(spawned);
        return result;
    }

    // Wait loop: poll the pipe (50 ms) and reap the child, killing its group on timeout or cancel.
    fcntl(out[0], F_SETFL, O_NONBLOCK);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSeconds);
    bool open = true; // the pipe has not reached EOF

    // Reads whatever is available (output capped at 1 MiB); stops at EOF, EAGAIN or an error.
    auto drain = [&] {
        char buffer[4096];
        for (;;) {
            const ssize_t n = read(out[0], buffer, sizeof(buffer));
            if (n > 0) {
                if (result.output.size() < (1u << 20))
                    result.output.append(buffer, size_t(n));
                continue;
            }
            if (n == 0)
                open = false;
            if (n == 0 || errno != EINTR)
                return;
        }
    };

    int status = 0;
    for (;;) {
        if (open) {
            pollfd ready{out[0], POLLIN, 0};
            poll(&ready, 1, 50);
            drain();
        } else
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const pid_t waited = waitpid(pid, &status, WNOHANG);
        if (waited == pid || (waited < 0 && errno == ECHILD)) {
            if (open)
                drain(); // what it wrote just before exiting; a leftover child may hold the pipe open, so no EOF wait
            if (waited == pid)
                result.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            break;
        }
        if (std::chrono::steady_clock::now() > deadline || (cancel && *cancel)) {
            kill(-pid, SIGKILL);
            waitpid(pid, &status, 0);
            result.timedOut = true;
            break;
        }
    }
    close(out[0]);
    return result;
}

} // namespace nereus::ros_viewer::panels
