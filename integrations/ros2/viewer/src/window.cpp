#include "window.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <cstdio>
#include <imgui_internal.h>
#include <iostream>
#include <png.h>
#include <stdexcept>

namespace nereus::ros_viewer::host {
namespace {
const ImVec4 cyan(.32f, .86f, .82f, 1), muted(.47f, .57f, .64f, 1), white(.87f, .92f, .95f, 1);
} // namespace

Window::Window(int width, int height, const std::string &titleText, bool hidden, bool vsync) {
    glfwSetErrorCallback([](int, const char *text) { std::cerr << "GLFW: " << text << '\n'; });
    if (!glfwInit())
        throw std::runtime_error("GLFW initialization failed. OpenGL 3.3 and an X/Wayland display are required.");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, hidden ? GLFW_FALSE : GLFW_TRUE);
    window_ = glfwCreateWindow(width, height, titleText.c_str(), nullptr, nullptr);
    if (!window_) {
        glfwTerminate();
        throw std::runtime_error("Cannot create an OpenGL 3.3 window");
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
    io.IniFilename = nullptr;
    const std::filesystem::path font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                bold = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf";
    if (std::filesystem::exists(font)) {
        normal = io.Fonts->AddFontFromFileTTF(font.c_str(), 15);
        small = io.Fonts->AddFontFromFileTTF(font.c_str(), 12);
        title = io.Fonts->AddFontFromFileTTF(std::filesystem::exists(bold) ? bold.c_str() : font.c_str(), 21);
        number = io.Fonts->AddFontFromFileTTF(font.c_str(), 25);
    } else
        normal = small = title = number = io.Fonts->AddFontDefault();
    ImGui::StyleColorsDark();
    auto &s = ImGui::GetStyle();
    s.Colors[ImGuiCol_ScrollbarBg].w = 0;
    s.Colors[ImGuiCol_ScrollbarGrab].w = 0;
    s.WindowPadding = {18, 16};
    s.FramePadding = {10, 7};
    s.ItemSpacing = {10, 9};
    s.WindowRounding = 9;
    s.ChildRounding = 8;
    s.FrameRounding = 5;
    s.WindowBorderSize = 0;
    s.ChildBorderSize = 1;
    s.PopupRounding = 6;
    s.GrabRounding = 5;
    s.Colors[ImGuiCol_WindowBg] = {.035f, .052f, .066f, 1};
    s.Colors[ImGuiCol_ChildBg] = {.052f, .074f, .091f, 1};
    s.Colors[ImGuiCol_Border] = {.12f, .18f, .21f, 1};
    s.Colors[ImGuiCol_Text] = white;
    s.Colors[ImGuiCol_TextDisabled] = muted;
    s.Colors[ImGuiCol_FrameBg] = {.083f, .12f, .145f, 1};
    s.Colors[ImGuiCol_Button] = {.085f, .14f, .17f, 1};
    s.Colors[ImGuiCol_ButtonHovered] = {.13f, .27f, .29f, 1};
    s.Colors[ImGuiCol_ButtonActive] = {.11f, .36f, .35f, 1};
    s.Colors[ImGuiCol_CheckMark] = cyan;
    s.Colors[ImGuiCol_SliderGrab] = cyan;
    s.Colors[ImGuiCol_SliderGrabActive] = {.5f, .96f, .89f, 1};
    s.Colors[ImGuiCol_Header] = {.09f, .25f, .27f, 1};
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
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
