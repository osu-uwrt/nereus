// GLFW/GLEW/ImGui context of the host: one window, DejaVu fonts, the dark pool-viewer theme.
#pragma once
#include "nereus/ros_viewer/theme.hpp"
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
    // Frames from another thread while this one is busy (the loading screen while the scene builds): this thread
    // keeps the frame it just drew (keepFrame, before swap) and detaches; the other makes the GL context current
    // there and draws ImGui frames over the kept one without GLFW's window calls (main-thread only, so no input
    // meanwhile), then releases it for this thread to reattach. ImGui is that thread's alone in between.
    void keepFrame();
    void detach();
    void reattach();
    void currentOnThisThread(bool current);
    void beginDetachedFrame(float dt);
    void presentDetached();
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
    ImFont *titleSmall = nullptr; // the title bar's search box (smaller than its menus, as VS Code's)
    // Loads the fonts at the interface scale (`ui`) and the title bar's at `titleBar`; between frames only. The
    // theme's own font files (in `fontDirectory`) or family (fontconfig name, e.g. a Qt theme's) at its point size
    // replace the viewer's DejaVu Sans.
    void loadFonts(float ui, float titleBar, const Theme &theme = {}, const std::filesystem::path &fontDirectory = {});
    // The desktop's UI scale for this window (GNOME at 200 %: 2), as other applications size themselves.
    float contentScale() const;
    int imguiErrors() const;

  private:
    void moveResize(long direction);
    void render(int width, int height); // ImGui's draw data over a cleared framebuffer
    bool backendReady_ = false;
    bool detached_ = false;    // another thread is drawing (contentScale answers from the last query)
    mutable float scale_ = 1;  // the last content scale queried
    unsigned kept_ = 0;        // the frame kept for detached frames (a texture, until reattach)
    GLFWwindow *window_ = nullptr;
    bool custom_ = false;
};
// Writes RGB8 rows (top row first) as a PNG; throws on failure.
void writePng(const std::filesystem::path &, int width, int height, const std::vector<unsigned char> &rgb);
// Reads a PNG as RGBA8 rows (top row first); false if it cannot be read.
bool readPng(const std::filesystem::path &, int &width, int &height, std::vector<unsigned char> &rgba);
} // namespace nereus::ros_viewer::host
