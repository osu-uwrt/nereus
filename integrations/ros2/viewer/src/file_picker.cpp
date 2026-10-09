// The desktop's "open file" dialog. See file_picker.hpp.
#include "file_picker.hpp"
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <fcntl.h>
#include <mutex>
#include <spawn.h>
#include <sstream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

extern char **environ;

namespace fs = std::filesystem;

namespace nereus::ros_viewer::host {
namespace {

// `name` on PATH, as posix_spawnp would find it; empty when it isn't there.
std::string onPath(const std::string &name) {
    const char *path = std::getenv("PATH");
    std::istringstream folders(path ? path : "");
    for (std::string folder; std::getline(folders, folder, ':');)
        if (!folder.empty() && access((fs::path(folder) / name).c_str(), X_OK) == 0)
            return name;
    return {};
}

std::string joined(const std::vector<std::string> &items) {
    std::string result;
    for (const auto &item : items)
        result += (result.empty() ? "" : " ") + item;
    return result;
}

} // namespace

// What the waiting thread and the UI thread share.
struct FilePicker::State {
    std::mutex mutex;
    pid_t pid = 0; // the dialog's process while it is open
    bool busy = false;
    std::optional<fs::path> picked;
};

FilePicker::FilePicker() : state_(std::make_shared<State>()) {
    program_ = onPath("zenity");
    if (program_.empty())
        program_ = onPath("kdialog");
}

FilePicker::~FilePicker() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->pid > 0)
        kill(state_->pid, SIGTERM); // the waiting thread reaps it
}

std::vector<std::string> FilePicker::command(const std::string &program, const std::string &title,
                                             const std::string &start, const std::string &kind,
                                             const std::vector<std::string> &patterns, unsigned long parent) {
    if (program == "kdialog") {
        std::vector<std::string> args{"kdialog", "--title",
                                      title,     "--getopenfilename",
                                      start,     patterns.empty() ? "*" : joined(patterns) + "|" + kind};
        if (parent)
            args.insert(args.begin() + 1, {"--attach", std::to_string(parent)});
        return args;
    }
    std::vector<std::string> args{"zenity", "--file-selection", "--title=" + title, "--filename=" + start};
    if (!patterns.empty())
        args.push_back("--file-filter=" + kind + " | " + joined(patterns));
    args.push_back("--file-filter=All files | *");
    if (parent) {
        args.push_back("--attach=" + std::to_string(parent));
        args.push_back("--modal");
    }
    return args;
}

bool FilePicker::open(const std::string &title, const fs::path &start, const std::string &kind,
                      const std::vector<std::string> &patterns) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (program_.empty() || state_->busy)
        return false;

    // Start in the file's folder with the file selected, in a folder (zenity wants its trailing slash), or at home.
    std::error_code error;
    std::string from = start.string();
    if (from.empty()) {
        const char *home = std::getenv("HOME");
        from = home ? home : "/";
    }
    if (fs::is_directory(from, error) && from.back() != '/')
        from += '/';
    else if (!fs::exists(from, error) && fs::is_directory(fs::path(from).parent_path(), error))
        from = fs::path(from).parent_path().string() + '/'; // a file that is gone: its folder

    // The dialog writes the path to stdout; its stderr (GTK chatter) goes nowhere.
    int out[2];
    if (pipe2(out, O_CLOEXEC) != 0)
        return false;
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
    posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);
    const auto args = command(program_, title, from, kind, patterns, parent_);
    std::vector<char *> argv;
    for (const auto &arg : args)
        argv.push_back(const_cast<char *>(arg.c_str()));
    argv.push_back(nullptr);
    pid_t pid = 0;
    const int spawned = posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(out[1]);
    if (spawned != 0) {
        close(out[0]);
        return false;
    }
    state_->pid = pid;
    state_->busy = true;
    state_->picked.reset();

    // Read the answer until the dialog closes, then reap it; a pick is a zero exit with a path.
    std::thread([state = state_, fd = out[0], pid] {
        std::string text;
        char buffer[512];
        for (ssize_t n; (n = read(fd, buffer, sizeof(buffer))) != 0;)
            if (n > 0)
                text.append(buffer, size_t(n));
            else if (errno != EINTR)
                break;
        close(fd);
        int status = 0;
        while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
            text.pop_back();
        std::lock_guard<std::mutex> lock(state->mutex);
        if (WIFEXITED(status) && WEXITSTATUS(status) == 0 && !text.empty())
            state->picked = fs::path(text);
        state->pid = 0;
        state->busy = false;
    }).detach();
    return true;
}

bool FilePicker::busy() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->busy;
}

const char *FilePicker::hint() const {
    if (program_.empty())
        return "Needs a desktop file picker: sudo apt install zenity (or kdialog)";
    return busy() ? "The file picker is open" : "Find the file in the desktop's file picker";
}

std::optional<fs::path> FilePicker::take() {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto picked = std::move(state_->picked);
    state_->picked.reset();
    return picked;
}
} // namespace nereus::ros_viewer::host
