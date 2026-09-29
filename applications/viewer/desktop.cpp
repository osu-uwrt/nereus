#include "desktop.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace robotics::viewer {
Desktop::Desktop(bool hidden) {
    try {
        glfwSetErrorCallback([](int, const char *text) { std::cerr << "GLFW: " << text << '\n'; });
        if (!glfwInit())
            throw std::runtime_error("GLFW initialization failed; a desktop display is required");
        glfw_ready_ = true;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, hidden ? GLFW_FALSE : GLFW_TRUE);
        window_ = glfwCreateWindow(1280, 800, "Robotics Viewer", nullptr, nullptr);
        if (!window_)
            throw std::runtime_error("cannot create an OpenGL 3.3 window");
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK)
            throw std::runtime_error("OpenGL loading failed");
        // GLEW probes legacy extension state on some core contexts.
        while (glGetError() != GL_NO_ERROR) {
        }
        std::cout << "Renderer: " << glGetString(GL_RENDERER) << '\n';
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        imgui_ready_ = true;
        auto &io = ImGui::GetIO();
        io.IniFilename = nullptr;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        const std::filesystem::path font("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
        if (std::filesystem::exists(font))
            io.Fonts->AddFontFromFileTTF(font.c_str(), 15);
        ImGui::StyleColorsDark();
        auto &style = ImGui::GetStyle();
        style.WindowPadding = {16, 14};
        style.FramePadding = {8, 6};
        style.ItemSpacing = {9, 9};
        style.FrameRounding = 4;
        style.ChildRounding = 6;
        style.Colors[ImGuiCol_WindowBg] = {0.035F, 0.05F, 0.07F, 1};
        style.Colors[ImGuiCol_ChildBg] = {0.05F, 0.07F, 0.09F, 1};
        style.Colors[ImGuiCol_Button] = {0.08F, 0.23F, 0.25F, 1};
        style.Colors[ImGuiCol_ButtonHovered] = {0.12F, 0.35F, 0.37F, 1};
        platform_ready_ = ImGui_ImplGlfw_InitForOpenGL(window_, true);
        if (!platform_ready_)
            throw std::runtime_error("ImGui window backend failed");
        renderer_ready_ = ImGui_ImplOpenGL3_Init("#version 330 core");
        if (!renderer_ready_)
            throw std::runtime_error("ImGui renderer backend failed");
    } catch (...) {
        shutdown();
        throw;
    }
}
Desktop::~Desktop() {
    shutdown();
}
void Desktop::shutdown() {
    if (renderer_ready_)
        ImGui_ImplOpenGL3_Shutdown();
    if (platform_ready_)
        ImGui_ImplGlfw_Shutdown();
    if (imgui_ready_)
        ImGui::DestroyContext();
    if (window_)
        glfwDestroyWindow(window_);
    if (glfw_ready_)
        glfwTerminate();
}
bool Desktop::closing() const {
    return glfwWindowShouldClose(window_) != 0;
}
void Desktop::begin() {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}
void Desktop::finish(const std::filesystem::path &screenshot) {
    ImGui::Render();
    int width = 0, height = 0;
    glfwGetFramebufferSize(window_, &width, &height);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, width, height);
    glClearColor(0.03F, 0.04F, 0.06F, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    if (!screenshot.empty()) {
        if (width <= 0 || height <= 0)
            throw std::runtime_error("cannot capture a zero-size framebuffer");
        std::vector<unsigned char> pixels(static_cast<std::size_t>(width) *
                                          static_cast<std::size_t>(height) * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
        std::ofstream output(screenshot, std::ios::binary);
        output << "P6\n" << width << ' ' << height << "\n255\n";
        for (int row = height - 1; row >= 0; --row)
            output.write(reinterpret_cast<const char *>(pixels.data() +
                                                        static_cast<std::size_t>(row) *
                                                            static_cast<std::size_t>(width) * 3),
                         static_cast<std::streamsize>(width) * 3);
        if (!output)
            throw std::runtime_error("cannot write screenshot: " + screenshot.string());
    }
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("OpenGL error while presenting frame");
    glfwSwapBuffers(window_);
}
Viewport::Viewport() {
    glGenFramebuffers(1, &framebuffer_);
    glGenFramebuffers(1, &read_framebuffer_);
    glGenTextures(1, &texture_);
    glGenRenderbuffers(1, &depth_);
}
Viewport::~Viewport() {
    glDeleteRenderbuffers(1, &depth_);
    glDeleteTextures(1, &texture_);
    glDeleteFramebuffers(1, &framebuffer_);
    glDeleteFramebuffers(1, &read_framebuffer_);
}
unsigned int Viewport::render(const std::vector<visualization::Line> &lines,
                              const Eigen::Matrix4f &matrix, int width, int height,
                              ViewportBackground background) {
    width = std::clamp(width, 1, 4096);
    height = std::clamp(height, 1, 4096);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    if (width != width_ || height != height_) {
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                     nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_, 0);
        glBindRenderbuffer(GL_RENDERBUFFER, depth_);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error("viewport framebuffer is incomplete");
        width_ = width;
        height_ = height;
    }
    glViewport(0, 0, width, height);
    glDisable(GL_SCISSOR_TEST);
    glDepthMask(GL_TRUE);
    glClearColor(0.025F, 0.04F, 0.055F, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (background.color) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, read_framebuffer_);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                               background.color, 0);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                               background.depth, 0);
        if (!background.depth ||
            glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            throw std::runtime_error("scene background framebuffer is incomplete");
        }
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glBlitFramebuffer(0, 0, width, height, 0, 0, width, height,
                          GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
        glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    }
    lines_.draw(lines, matrix);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return texture_;
}
} // namespace robotics::viewer
