// The desktop's "open file" dialog: its command lines, and the open / poll / take cycle against a stand-in zenity.
#include "file_picker.hpp"
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <optional>
#include <thread>

using namespace nereus::ros_viewer::host;
namespace fs = std::filesystem;

namespace {
// A temporary folder holding a stand-in picker; PATH is only that folder while it lives.
struct FakePicker {
    fs::path dir;
    std::string path;

    FakePicker(const std::string &name, const std::string &script) {
        dir = fs::temp_directory_path() / ("nereus_picker_" + std::to_string(getpid()));
        fs::remove_all(dir);
        fs::create_directories(dir);
        std::ofstream(dir / name) << "#!/bin/sh\n" << script;
        fs::permissions(dir / name, fs::perms::owner_all);
        const char *old = std::getenv("PATH");
        path = old ? old : "";
        setenv("PATH", dir.c_str(), 1);
    }
    ~FakePicker() {
        setenv("PATH", path.c_str(), 1);
        fs::remove_all(dir);
    }
};

// Polls until the dialog closed (or `seconds` passed).
void waitClosed(const FilePicker &picker, double seconds = 5) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (picker.busy() && std::chrono::steady_clock::now() < end)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

std::string read(const fs::path &file) {
    std::ifstream in(file);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
} // namespace

TEST(FilePicker, CommandLines) {
    EXPECT_EQ(FilePicker::command("zenity", "Open", "/maps/", "YAML", {"*.yaml", "*.yml"}, 0),
              (std::vector<std::string>{"zenity", "--file-selection", "--title=Open", "--filename=/maps/",
                                        "--file-filter=YAML | *.yaml *.yml", "--file-filter=All files | *"}));
    EXPECT_EQ(FilePicker::command("zenity", "Open", "/maps/a.yaml", "", {}, 42),
              (std::vector<std::string>{"zenity", "--file-selection", "--title=Open", "--filename=/maps/a.yaml",
                                        "--file-filter=All files | *", "--attach=42", "--modal"}));
    EXPECT_EQ(FilePicker::command("kdialog", "Open", "/maps/", "YAML", {"*.yaml", "*.yml"}, 42),
              (std::vector<std::string>{"kdialog", "--attach", "42", "--title", "Open", "--getopenfilename", "/maps/",
                                        "*.yaml *.yml|YAML"}));
}

// A pick comes back once; the dialog starts in the given folder (with zenity's trailing slash).
TEST(FilePicker, ReturnsThePickOnce) {
    FakePicker fake("zenity", "echo \"$@\" > \"${0%/*}/args\"\necho /maps/config.yaml\n");
    FilePicker picker;
    EXPECT_EQ(picker.program(), "zenity");
    ASSERT_TRUE(picker.open("Open prior map", fake.dir, "YAML", {"*.yaml"}));
    EXPECT_FALSE(picker.open("again", fake.dir, "YAML", {"*.yaml"})) << "one dialog at a time";
    waitClosed(picker);
    ASSERT_FALSE(picker.busy());
    EXPECT_EQ(picker.take(), std::optional<fs::path>("/maps/config.yaml"));
    EXPECT_EQ(picker.take(), std::nullopt);
    EXPECT_NE(read(fake.dir / "args").find("--filename=" + fake.dir.string() + "/ "), std::string::npos);
}

// Cancel (a non-zero exit) picks nothing, and the picker can open again.
TEST(FilePicker, CancelPicksNothing) {
    FakePicker fake("zenity", "exit 1\n");
    FilePicker picker;
    ASSERT_TRUE(picker.open("Open", {}, "YAML", {"*.yaml"}));
    waitClosed(picker);
    EXPECT_EQ(picker.take(), std::nullopt);
    EXPECT_TRUE(picker.open("Open", {}, "YAML", {"*.yaml"}));
    waitClosed(picker);
}

// kdialog when there is no zenity; nothing at all disables the picker.
TEST(FilePicker, FallsBackToKdialogOrNothing) {
    {
        FakePicker fake("kdialog", "echo /maps/k.yaml\n");
        FilePicker picker;
        EXPECT_EQ(picker.program(), "kdialog");
        ASSERT_TRUE(picker.open("Open", {}, "YAML", {"*.yaml"}));
        waitClosed(picker);
        EXPECT_EQ(picker.take(), std::optional<fs::path>("/maps/k.yaml"));
    }
    FakePicker none("unrelated", "exit 0\n");
    FilePicker picker;
    EXPECT_TRUE(picker.program().empty());
    EXPECT_FALSE(picker.open("Open", {}, "YAML", {"*.yaml"}));
}

// Destroying the picker closes a dialog that is still open.
TEST(FilePicker, ClosesAnOpenDialogWhenDestroyed) {
    FakePicker fake("zenity", "echo $$ > \"${0%/*}/pid\"\nexec /bin/sleep 30\n");
    pid_t pid = 0;
    {
        FilePicker picker;
        ASSERT_TRUE(picker.open("Open", {}, "YAML", {"*.yaml"}));
        for (int i = 0; i < 500 && !fs::exists(fake.dir / "pid"); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // the pid file is complete
        pid = std::stoi(read(fake.dir / "pid"));
        ASSERT_EQ(kill(pid, 0), 0);
    }
    // SIGTERM ends it and the waiting thread reaps it.
    bool gone = false;
    for (int i = 0; i < 500 && !gone; ++i) {
        gone = kill(pid, 0) != 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(gone);
}
