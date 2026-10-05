#include "window.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <cmath>
#include <cstdio>
#include <fontconfig/fontconfig.h>
#include <imgui_internal.h>
#include <iostream>
#include <png.h>
#include <stdexcept>
#include <strings.h>
#define GLFW_EXPOSE_NATIVE_X11
#include <GLFW/glfw3native.h> // after everything else: X11 headers define macros such as None and Status

namespace nereus::ros_viewer::host {
Window::Window(int width, int height, const std::string &titleText, bool hidden, bool vsync, bool customTitleBar)
    : custom_(customTitleBar) {
    glfwSetErrorCallback([](int, const char *text) { std::cerr << "GLFW: " << text << '\n'; });
    if (!glfwInit())
        throw std::runtime_error("GLFW initialization failed. OpenGL 3.3 and an X/Wayland display are required.");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, hidden ? GLFW_FALSE : GLFW_TRUE);
    glfwWindowHint(GLFW_DECORATED, custom_ ? GLFW_FALSE : GLFW_TRUE);
    window_ = glfwCreateWindow(width, height, titleText.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Cannot create an OpenGL 3.3 window");
    }
    if (custom_ && !glfwGetX11Display()) { // no window-manager protocol to move / resize with
        custom_ = false;
        glfwSetWindowAttrib(window_, GLFW_DECORATED, GLFW_TRUE);
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(vsync && !hidden ? 1 : 0);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
        throw std::runtime_error("OpenGL loading failed");
    while (glGetError() != GL_NO_ERROR) {
    }
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << " / " << glGetString(GL_VERSION) << '\n';
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    auto &io = ImGui::GetIO();
    io.IniFilename = nullptr; // the host saves layouts itself (session file, named layouts)
    // Every operator window docks: drag a tab to rearrange, split or float. Windows move only by their title
    // bar or tab, so drags inside them (orbit, map pan, gizmo) never move the window.
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    loadFonts(1, contentScale());
    ImGui::StyleColorsDark(); // the application applies its theme
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    backendReady_ = true;
}

float Window::contentScale() const {
    float x = 1, y = 1;
    glfwGetWindowContentScale(window_, &x, &y);
    return std::isfinite(x) && x > 0 ? x : 1;
}

namespace {
// The file fontconfig picks for a family and style (empty if none).
std::string fontFile(const std::string &family, const char *style) {
    std::string file;
    if (family.empty() || !FcInit())
        return file;
    FcPattern *pattern = FcPatternCreate();
    FcPatternAddString(pattern, FC_FAMILY, reinterpret_cast<const FcChar8 *>(family.c_str()));
    FcPatternAddString(pattern, FC_STYLE, reinterpret_cast<const FcChar8 *>(style));
    FcConfigSubstitute(nullptr, pattern, FcMatchPattern);
    FcDefaultSubstitute(pattern);
    FcResult result = FcResultNoMatch;
    if (FcPattern *match = FcFontMatch(nullptr, pattern, &result)) {
        FcChar8 *path = nullptr, *matched = nullptr;
        // Only the family asked for: fontconfig always returns something, often an unrelated fallback.
        if (FcPatternGetString(match, FC_FILE, 0, &path) == FcResultMatch &&
            FcPatternGetString(match, FC_FAMILY, 0, &matched) == FcResultMatch &&
            strcasecmp(reinterpret_cast<const char *>(matched), family.c_str()) == 0)
            file = reinterpret_cast<const char *>(path);
        FcPatternDestroy(match);
    }
    FcPatternDestroy(pattern);
    return file;
}
} // namespace

void Window::loadFonts(float ui, float titleBar, const std::string &family, float points) {
    auto &io = ImGui::GetIO();
    io.Fonts->Clear();
    std::filesystem::path font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                          bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf";
    float body = 15; // pixels at 100 %; the other sizes keep their ratios to it
    if (const auto regular = fontFile(family, "Regular"); !regular.empty()) {
        font = regular;
        const auto heavy = fontFile(family, "Bold");
        bold = heavy.empty() ? regular : heavy;
        if (points > 0)
            body = points * 96.f / 72.f;
    }
    const float k = body / 15;
    if (std::filesystem::exists(font)) {
        normal = io.Fonts->AddFontFromFileTTF(font.c_str(), 15 * k * ui);
        const auto *heavy = std::filesystem::exists(bold) ? bold.c_str() : font.c_str();
        strong = io.Fonts->AddFontFromFileTTF(heavy, 15 * k * ui);
        smallStrong = io.Fonts->AddFontFromFileTTF(heavy, 12 * k * ui);
        small = io.Fonts->AddFontFromFileTTF(font.c_str(), 12 * k * ui);
        title = io.Fonts->AddFontFromFileTTF(std::filesystem::exists(bold) ? bold.c_str() : font.c_str(), 21 * k * ui);
        number = io.Fonts->AddFontFromFileTTF(font.c_str(), 25 * k * ui);
        menu = io.Fonts->AddFontFromFileTTF(font.c_str(), 13 * k * titleBar);
    } else
        normal = small = title = number = menu = strong = smallStrong = io.Fonts->AddFontDefault();
    io.FontDefault = normal;
    setTypeRamp({strong, number, small, smallStrong});
    io.Fonts->Build();
    if (backendReady_) { // a running viewer: replace the GPU copy of the atlas
        ImGui_ImplOpenGL3_DestroyFontsTexture();
        ImGui_ImplOpenGL3_CreateFontsTexture();
    }
}

Window::~Window() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (window_)
        glfwDestroyWindow(window_);
    glfwTerminate();
}
bool Window::closing() const {
    return glfwWindowShouldClose(window_) != 0;
}
void Window::setTitle(const std::string &text) {
    glfwSetWindowTitle(window_, text.c_str());
}
int Window::imguiErrors() const {
    return ImGui::GetCurrentContext()->ErrorCountCurrentFrame;
}
void Window::beginFrame() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}
void Window::present(bool screenshotFrame, const std::filesystem::path &screenshot) {
    ImGui::Render();
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, w, h);
    glClearColor(.028f, .043f, .057f, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (screenshotFrame && !screenshot.empty()) {
        std::vector<unsigned char> pixels(std::size_t(w) * std::size_t(h) * 3), flipped(pixels.size());
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        for (int y = 0; y < h; ++y)
            std::copy_n(pixels.data() + std::size_t(h - 1 - y) * std::size_t(w) * 3, std::size_t(w) * 3,
                        flipped.data() + std::size_t(y) * std::size_t(w) * 3);
        writePng(screenshot, w, h, flipped);
        std::cout << "Saved " << screenshot << '\n';
    }
}
void Window::swap() {
    glfwSwapBuffers(window_);
}
void Window::minimize() {
    glfwIconifyWindow(window_);
}
bool Window::maximized() const {
    return glfwGetWindowAttrib(window_, GLFW_MAXIMIZED) != 0;
}
void Window::toggleMaximized() {
    if (maximized())
        glfwRestoreWindow(window_);
    else
        glfwMaximizeWindow(window_);
}
void Window::requestClose() {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
}
void Window::cancelClose() {
    glfwSetWindowShouldClose(window_, GLFW_FALSE);
}
void Window::setIcon(const std::vector<std::filesystem::path> &pngs) {
    std::vector<std::vector<unsigned char>> pixels;
    std::vector<GLFWimage> images;
    pixels.reserve(pngs.size());
    for (const auto &path : pngs) {
        int width = 0, height = 0;
        std::vector<unsigned char> rgba;
        if (!readPng(path, width, height, rgba))
            continue;
        pixels.push_back(std::move(rgba));
        images.push_back({width, height, pixels.back().data()});
    }
    if (!images.empty())
        glfwSetWindowIcon(window_, int(images.size()), images.data());
}

void Window::beginMove() {
    moveResize(8); // _NET_WM_MOVERESIZE_MOVE
}
void Window::beginResize(int edge) {
    if (edge >= 0 && edge <= 7)
        moveResize(edge); // _NET_WM_MOVERESIZE_SIZE_TOPLEFT ... _SIZE_LEFT
}
// EWMH _NET_WM_MOVERESIZE: the window manager takes the pointer and moves or resizes the window like its own
// decorations would (snapping, tiling, maximize on drag). It also takes the button release, so ImGui is told the
// button went up.
void Window::moveResize(long direction) {
    Display *display = glfwGetX11Display();
    const ::Window window = glfwGetX11Window(window_);
    if (!display || !window)
        return;
    ::Window root = 0, child = 0;
    int rootX = 0, rootY = 0, x = 0, y = 0;
    unsigned int mask = 0;
    XQueryPointer(display, window, &root, &child, &rootX, &rootY, &x, &y, &mask);
    XUngrabPointer(display, CurrentTime); // release the implicit grab of the button press
    XEvent event{};
    event.xclient.type = ClientMessage;
    event.xclient.window = window;
    event.xclient.message_type = XInternAtom(display, "_NET_WM_MOVERESIZE", False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = rootX;
    event.xclient.data.l[1] = rootY;
    event.xclient.data.l[2] = direction;
    event.xclient.data.l[3] = Button1;
    event.xclient.data.l[4] = 1; // source: a normal application
    XSendEvent(display, DefaultRootWindow(display), False, SubstructureRedirectMask | SubstructureNotifyMask, &event);
    XFlush(display);
    ImGui::GetIO().AddMouseButtonEvent(ImGuiMouseButton_Left, false);
}

bool readPng(const std::filesystem::path &path, int &width, int &height, std::vector<unsigned char> &rgba) {
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&image, path.c_str()))
        return false;
    image.format = PNG_FORMAT_RGBA;
    rgba.resize(PNG_IMAGE_SIZE(image));
    if (!png_image_finish_read(&image, nullptr, rgba.data(), 0, nullptr)) {
        png_image_free(&image);
        return false;
    }
    width = int(image.width);
    height = int(image.height);
    return true;
}

void writePng(const std::filesystem::path &path, int width, int height, const std::vector<unsigned char> &rgb) {
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file)
        throw std::runtime_error("cannot write screenshot: " + path.string());
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png ? png_create_info_struct(png) : nullptr;
    if (!png || !info || setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        std::fclose(file);
        throw std::runtime_error("cannot encode screenshot: " + path.string());
    }
    png_init_io(png, file);
    png_set_IHDR(png, info, png_uint_32(width), png_uint_32(height), 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    for (int y = 0; y < height; ++y)
        png_write_row(png, const_cast<png_bytep>(rgb.data() + std::size_t(y) * std::size_t(width) * 3));
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    std::fclose(file);
}
} // namespace nereus::ros_viewer::host
