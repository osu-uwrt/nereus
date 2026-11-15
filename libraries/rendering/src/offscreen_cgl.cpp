// Headless CGL host that runs a Renderer without a window: the macOS side of offscreen.cpp (macOS has no EGL).
// A CGL context needs no window or main thread, so camera workers can use it like the EGL one.
#define GL_SILENCE_DEPRECATION // CGL and OpenGL still work, Apple just stopped adding to them
#include "nereus/rendering/offscreen.hpp"
#include <GL/glew.h>
#include <OpenGL/OpenGL.h>
#include <mutex>
#include <stdexcept>

namespace nereus::rendering {
namespace {

// GLEW has process-global function pointers. Serialize initialization and all capture
// hosts as well as context migration; the renderer itself remains caller-context-owned.
std::mutex host_mutex;

std::runtime_error cglError(const char *operation, CGLError error) {
    return std::runtime_error(std::string(operation) + " failed (CGL " + CGLErrorString(error) + ")");
}

// RAII: makes the host context current for one call, then restores the caller's (e.g. a GLFW window's).
struct Binding {
    CGLContextObj previous = CGLGetCurrentContext();

    explicit Binding(CGLContextObj context) {
        if (const auto error = CGLSetCurrentContext(context))
            throw cglError("bind offscreen context", error);
    }
    ~Binding() {
        CGLSetCurrentContext(previous);
    }
};

} // namespace

struct OffscreenRenderer::Impl {
    CGLContextObj context = nullptr;
    std::unique_ptr<Renderer> renderer;
    std::string device; // GL_RENDERER string of the context.

    // Constructed/destroyed under host_mutex. Tear down every partially-created resource.
    ~Impl() {
        if (!context)
            return;
        if (renderer) {
            try {
                Binding binding(context);
                renderer.reset();
            } catch (...) {
                // Release CPU state without GL calls; destroying the context releases its GPU allocations.
                renderer->abandonContext();
                renderer.reset();
            }
        }
        CGLDestroyContext(context);
    }
};

OffscreenRenderer::OffscreenRenderer(const std::filesystem::path &shader_directory) {
    std::lock_guard<std::mutex> lock(host_mutex);
    auto next = std::make_unique<Impl>();

    // Core profile (3.2 asks for the newest core, 4.1), RGB8 + 24-bit depth. No drawable: all drawing goes to
    // the renderer's own framebuffers. Offline GPUs allowed, so a Mac with the display off still renders.
    const CGLPixelFormatAttribute attributes[] = {kCGLPFAOpenGLProfile,
                                                  CGLPixelFormatAttribute(kCGLOGLPVersion_3_2_Core),
                                                  kCGLPFAColorSize,
                                                  CGLPixelFormatAttribute(24),
                                                  kCGLPFADepthSize,
                                                  CGLPixelFormatAttribute(24),
                                                  kCGLPFAAllowOfflineRenderers,
                                                  CGLPixelFormatAttribute(0)};
    CGLPixelFormatObj format = nullptr;
    GLint count = 0;
    if (const auto error = CGLChoosePixelFormat(attributes, &format, &count))
        throw cglError("choose offscreen configuration", error);
    if (!format)
        throw std::runtime_error("choose offscreen configuration failed (no OpenGL 3.3 core pixel format)");
    const auto error = CGLCreateContext(format, nullptr, &next->context);
    CGLDestroyPixelFormat(format);
    if (error)
        throw cglError("create OpenGL core context", error);

    // Load GL entry points and create the renderer while the new context is current.
    {
        Binding binding(next->context);
        glewExperimental = GL_TRUE;
        if (const auto status = glewInit(); status != GLEW_OK)
            throw std::runtime_error("initialize OpenGL functions failed: " + std::to_string(status));
        if (!GLEW_VERSION_3_3)
            throw std::runtime_error("OpenGL 3.3 is unavailable in the offscreen context");
        // Drain errors glewInit may leave (e.g. core-profile GL_INVALID_ENUM) so checkGl starts clean.
        while (glGetError() != GL_NO_ERROR) {
        }
        const auto *name = glGetString(GL_RENDERER);
        next->device = name ? reinterpret_cast<const char *>(name) : "unknown";
        next->renderer = std::make_unique<Renderer>(shader_directory);
    }
    impl_ = std::move(next);
}

OffscreenRenderer::~OffscreenRenderer() {
    std::lock_guard<std::mutex> lock(host_mutex);
    impl_.reset();
}

ImageCapture OffscreenRenderer::capture(const Scene &scene, const View &view, const Appearance &appearance, float time,
                                        int width, int height, bool color, bool depth,
                                        const std::optional<SensorFormat> &sensor) {
    std::lock_guard<std::mutex> lock(host_mutex);
    Binding binding(impl_->context);
    impl_->renderer->draw(scene, view, appearance, time, width, height);
    return impl_->renderer->captureImage(color, depth, sensor);
}

LabelCapture OffscreenRenderer::captureLabels(const Scene &scene, const std::vector<InstanceLabel> &labels,
                                              const View &view, int width, int height) {
    std::lock_guard<std::mutex> lock(host_mutex);
    Binding binding(impl_->context);
    return impl_->renderer->drawLabels(scene, labels, view, width, height);
}

const std::string &OffscreenRenderer::device() const {
    return impl_->device;
}

} // namespace nereus::rendering
