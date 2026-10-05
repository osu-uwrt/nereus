// GLFW/GLEW/ImGui context of the host: one window, DejaVu fonts, the dark pool-viewer theme.
#pragma once
#include <filesystem>
#include <imgui.h>
#include <string>
#include <vector>

struct GLFWwindow;
namespace nereus::ros_viewer::host {
class Window {
  public:
    // customTitleBar: no system decorations; the application draws the title bar (menus, window buttons) and
    // asks the window manager to move / resize through beginMove / beginResize. Falls back to the system title
    // bar where that is unavailable (not X11).
    Window(int width, int height, const std::string &title, bool hidden, bool vsync = true,
           bool customTitleBar = false);
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
    bool customTitleBar() const {
        return custom_;
    }
    // Window-manager move / resize from the current pointer position, as when dragging a system title bar or
    // border. Edge: 0 top-left, 1 top, 2 top-right, 3 right, 4 bottom-right, 5 bottom, 6 bottom-left, 7 left.
    void beginMove();
    void beginResize(int edge);
    void minimize();
    void toggleMaximized();
    bool maximized() const;
    void requestClose();
    void cancelClose(); // a close the application refuses (asks first)
    // The window's icon (dock, task switcher) from PNG files of several sizes; missing files are skipped.
    void setIcon(const std::vector<std::filesystem::path> &pngs);
    ImFont *normal = nullptr, *small = nullptr, *title = nullptr, *number = nullptr;
    ImFont *strong = nullptr;      // the body size in bold: section titles, table headers
    ImFont *smallStrong = nullptr; // the small size in bold: chart labels
    ImFont *menu = nullptr; // the title bar's menus, at the desktop's scale like other apps' title bars
    // Loads the fonts at the interface scale (`ui`) and the title bar's at `titleBar`; between frames only. A
    // family (fontconfig name, e.g. a Qt theme's) at its point size replaces the viewer's DejaVu Sans.
    void loadFonts(float ui, float titleBar, const std::string &family = {}, float points = 0);
    // The desktop's UI scale for this window (GNOME at 200 %: 2), as other applications size themselves.
    float contentScale() const;
    int imguiErrors() const;

  private:
    void moveResize(long direction);
    bool backendReady_ = false;
    GLFWwindow *window_ = nullptr;
    bool custom_ = false;
};
// Writes RGB8 rows (top row first) as a PNG; throws on failure.
void writePng(const std::filesystem::path &, int width, int height, const std::vector<unsigned char> &rgb);
// Reads a PNG as RGBA8 rows (top row first); false if it cannot be read.
bool readPng(const std::filesystem::path &, int &width, int &height, std::vector<unsigned char> &rgba);
} // namespace nereus::ros_viewer::host
