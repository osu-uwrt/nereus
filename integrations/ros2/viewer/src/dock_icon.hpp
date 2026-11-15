// macOS: the Dock icon, which GLFW leaves alone (window icons don't exist there).
#pragma once
#include <filesystem>

namespace nereus::ros_viewer::host {
// Sets the application's Dock icon from a PNG; no-op if the file doesn't load.
void setDockIcon(const std::filesystem::path &png);
} // namespace nereus::ros_viewer::host
