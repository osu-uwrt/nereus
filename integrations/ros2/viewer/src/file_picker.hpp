// The desktop's "open file" dialog (zenity, else kdialog), run as its own process so the viewer keeps drawing
// while it is open: the UI thread opens it, then polls for the file picked.
#pragma once
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
class FilePicker {
  public:
    // Finds the picker program on PATH (zenity, else kdialog).
    FilePicker();
    ~FilePicker(); // closes a dialog that is still open
    FilePicker(const FilePicker &) = delete;
    FilePicker &operator=(const FilePicker &) = delete;

    // The picker program found, or empty when the desktop has none.
    const std::string &program() const {
        return program_;
    }

    // The X11 window the dialog stays above (0: none).
    void setParent(unsigned long x11Window) {
        parent_ = x11Window;
    }

    // Opens the dialog at `start` (a file to preselect or a folder; empty: home) listing `patterns` (e.g. *.yaml)
    // under the filter name `kind`, or everything. False when there is no picker or a dialog is already open.
    bool open(const std::string &title, const std::filesystem::path &start, const std::string &kind,
              const std::vector<std::string> &patterns);
    bool busy() const;
    // The file picked, once, after the dialog closed on one (nothing on cancel or failure).
    std::optional<std::filesystem::path> take();
    // A Browse button's tooltip: what it does, or why it can't.
    const char *hint() const;

    // The command line open() runs for `program` (zenity or kdialog).
    static std::vector<std::string> command(const std::string &program, const std::string &title,
                                            const std::string &start, const std::string &kind,
                                            const std::vector<std::string> &patterns, unsigned long parent);

  private:
    struct State; // shared with the thread that waits for the dialog
    std::shared_ptr<State> state_;
    std::string program_;
    unsigned long parent_ = 0;
};
} // namespace nereus::ros_viewer::host
