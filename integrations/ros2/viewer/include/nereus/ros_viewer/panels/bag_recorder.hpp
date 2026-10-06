// `ros2 bag record` on this computer or on another over ssh, run detached (its own session, output to a log) so a
// bag keeps recording when the viewer quits or the link to the robot drops. Each machine holds at most one such
// recording, described in ${XDG_STATE_HOME:-~/.local/state}/nereus/bag/active, so any viewer (a restarted one,
// another laptop) finds and can stop it. No ROS or ImGui here: shell scripts, their reports and a process runner.
#pragma once
#include <atomic>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace nereus::ros_viewer::panels {
// Where bags are recorded.
struct BagHost {
    std::string host;      // ssh destination (user@host); empty: this computer
    std::string directory; // where bags go on that machine; a leading ~ is that machine's home
    std::string setup;     // shell lines run before ros2 (source the workspace); empty: the inherited environment
    std::vector<std::string> ssh{"ssh"}; // the ssh program (and its own leading arguments)
    std::vector<std::string> recordArgs;  // extra `ros2 bag record` arguments (storage, splitting, ...)
};
// What to record.
struct BagRequest {
    std::string name;                // the bag's folder name inside the directory
    std::string directory;           // the directory on that machine (the provider's target default when empty)
    bool all = true;                 // -a, else `topics`
    std::vector<std::string> topics; // explicit topics (with all = false)
    std::string exclude;             // -x regex (empty: none)
};
// A script's report: lines "NEREUS_BAG <key> <value>" among whatever else the setup printed.
struct BagReport {
    std::map<std::string, std::string> values;
    bool has(const std::string &key) const {
        return values.count(key) != 0;
    }
    std::string get(const std::string &key) const {
        const auto found = values.find(key);
        return found == values.end() ? std::string() : found->second;
    }
    // A number reported as text, or `fallback`.
    double number(const std::string &key, double fallback = 0) const;
};
BagReport parseBagReport(const std::string &output);

// POSIX shell single quoting: the value as one word.
std::string shellQuote(const std::string &value);
// Scripts for `bash -c`. Each ends in a report with `state`: idle, recording, stopping, stopped, died or error
// (with `error`). Recording / stopping reports carry pid, bag (absolute path), start and now (the machine's epoch
// seconds, so elapsed time does not depend on the clocks agreeing), bytes and free (bytes left on its disk).
// start: refuses while a recording runs or the bag exists, sources the setup, starts the recorder and reports it
// once it has survived a moment (else the end of its log).
std::string bagStartScript(const BagHost &, const BagRequest &);
// status: the recording, if any; a recorder that exited without being stopped is reported once as died.
std::string bagStatusScript(const BagHost &);
// stop: SIGINT (as Ctrl-C, so the bag is closed properly) and up to `waitSeconds` for it to finish; still running
// after that, it reports stopping, and status reports stopped once it exits. kill = true: SIGKILL instead.
std::string bagStopScript(const BagHost &, double waitSeconds, bool kill = false);
// The command that runs `script` on the host: bash -c locally, else ssh (non-interactive, short timeouts, one
// shared connection) running bash -c there.
std::vector<std::string> bagCommand(const BagHost &, const std::string &script);
// "<prefix>_<YYYYmmdd_HHMMSS>" (local time), or the prefix alone without a timestamp.
std::string bagName(const std::string &prefix, bool timestamp);
// Whether `name` can be a bag folder name (no slashes, not . or ..; letters, digits and ._- only).
bool validBagName(const std::string &name);
// "1.4 GB" style sizes.
std::string byteSize(double bytes);

struct ProcessResult {
    int status = -1; // exit code; -1 on a signal, a spawn failure or a timeout
    bool timedOut = false;
    std::string output; // stdout and stderr, interleaved
};
// Runs argv (stdin /dev/null, no inherited descriptors, its own process group) and returns once it exits, or
// kills the group after `timeoutSeconds` or once `cancel` is set (timedOut either way). Returns on exit even when a
// child it left behind (an ssh connection master) still holds the output open.
ProcessResult runProcess(const std::vector<std::string> &argv, double timeoutSeconds,
                         const std::atomic<bool> *cancel = nullptr);
} // namespace nereus::ros_viewer::panels
