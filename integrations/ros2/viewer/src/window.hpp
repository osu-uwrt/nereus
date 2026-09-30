// GLFW/GLEW/ImGui context of the host: one window, DejaVu fonts, the dark pool-viewer theme.
#pragma once
#include <filesystem>
#include <imgui.h>
#include <string>
#include <vector>

struct GLFWwindow;
namespace robotics::ros_viewer::host {
class Window {
  public:
    Window(int width, int height, const std::string &title, bool hidden, bool vsync = true);
    ~Window();
    Window(const Window &) = delete;
    Window &operator=(const Window &) = delete;
    GLFWwindow *handle() const {
        return window_;
    }
    bool closing() const;
    void setTitle(const std::string &);
    void beginFrame();                                                           // poll events + ImGui NewFrame
    void present(bool screenshotFrame, const std::filesystem::path &screenshot); // render ImGui, optional PNG
    void swap();                                                                 // present the frame (blocks on vsync)
    ImFont *normal = nullptr, *small = nullptr, *title = nullptr, *number = nullptr;
    int imguiErrors() const;

  private:
    GLFWwindow *window_ = nullptr;
};
// Writes RGB8 rows (top row first) as a PNG; throws on failure.
void writePng(const std::filesystem::path &, int width, int height, const std::vector<unsigned char> &rgb);
} // namespace robotics::ros_viewer::host
