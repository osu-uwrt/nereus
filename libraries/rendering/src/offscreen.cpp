#include "nereus/rendering/offscreen.hpp"
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/glew.h>
#include <mutex>
#include <stdexcept>

namespace nereus::rendering {
namespace {
// GLEW has process-global function pointers. Serialize initialization and all capture
// hosts as well as context migration; the renderer itself remains caller-context-owned.
std::mutex host_mutex;
std::runtime_error eglError(const char *operation) {
    return std::runtime_error(std::string(operation) + " failed (EGL " + std::to_string(eglGetError()) + ")");
}
EGLDisplay sharedDisplay() {
    // EGL initialization is not reference-counted. Keep the process-wide display
    // initialized so destroying a host cannot invalidate another EGL user's context.
    // Access is serialized by host_mutex; a failed initialization is never cached.
    static EGLDisplay display = EGL_NO_DISPLAY;
    auto next = display;
    if (next == EGL_NO_DISPLAY) {
        const auto get =
            reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
        if (!get)
            throw std::runtime_error("surfaceless EGL platform support is unavailable");
        next = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    }
    // Another EGL user may terminate the display after every host is gone.
    // Repeated initialization is harmless and restores that valid lifecycle.
    EGLint major = 0, minor = 0;
    if (next == EGL_NO_DISPLAY || !eglInitialize(next, &major, &minor))
        throw eglError("initialize surfaceless display");
    display = next;
    return display;
}
struct Binding {
    EGLDisplay display, previous_display = eglGetCurrentDisplay();
    EGLContext previous_context = eglGetCurrentContext();
    EGLSurface previous_draw = eglGetCurrentSurface(EGL_DRAW);
    EGLSurface previous_read = eglGetCurrentSurface(EGL_READ);
    EGLenum previous_api = eglQueryAPI();
    Binding(EGLDisplay d, EGLSurface surface, EGLContext context) : display(d) {
        if (!eglBindAPI(EGL_OPENGL_API))
            throw eglError("bind OpenGL API");
        if (!eglMakeCurrent(d, surface, surface, context)) {
            const auto error = eglError("bind offscreen context");
            eglBindAPI(previous_api);
            throw error;
        }
    }
    ~Binding() {
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        eglBindAPI(previous_api);
        if (previous_context != EGL_NO_CONTEXT)
            eglMakeCurrent(previous_display, previous_draw, previous_read, previous_context);
    }
};
} // namespace
struct OffscreenRenderer::Impl {
    EGLDisplay display = EGL_NO_DISPLAY;
    EGLContext context = EGL_NO_CONTEXT;
    EGLSurface surface = EGL_NO_SURFACE;
    std::unique_ptr<Renderer> renderer;
    std::string device;
    // Constructed/destroyed under host_mutex. Tear down every partially-created resource.
    ~Impl() {
        if (display == EGL_NO_DISPLAY)
            return;
        if (renderer) {
            try {
                Binding binding(display, surface, context);
                renderer.reset();
            } catch (...) {
                // The context may have been lost. Release CPU state without GL calls;
                // destroying the context below releases its GPU allocations.
                renderer->abandonContext();
                renderer.reset();
            }
        }
        if (surface != EGL_NO_SURFACE)
            eglDestroySurface(display, surface);
        if (context != EGL_NO_CONTEXT)
            eglDestroyContext(display, context);
    }
};
OffscreenRenderer::OffscreenRenderer(const std::filesystem::path &shader_directory) {
    std::lock_guard<std::mutex> lock(host_mutex);
    auto next = std::make_unique<Impl>();
    next->display = sharedDisplay();
    const auto display = next->display;
    const EGLint attributes[] = {EGL_SURFACE_TYPE,
                                 EGL_PBUFFER_BIT,
                                 EGL_RENDERABLE_TYPE,
                                 EGL_OPENGL_BIT,
                                 EGL_RED_SIZE,
                                 8,
                                 EGL_GREEN_SIZE,
                                 8,
                                 EGL_BLUE_SIZE,
                                 8,
                                 EGL_DEPTH_SIZE,
                                 24,
                                 EGL_NONE};
    EGLConfig config;
    EGLint count = 0;
    if (!eglChooseConfig(display, attributes, &config, 1, &count) || count != 1)
        throw eglError("choose offscreen configuration");
    const EGLint surface_attributes[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    next->surface = eglCreatePbufferSurface(display, config, surface_attributes);
    if (next->surface == EGL_NO_SURFACE)
        throw eglError("create offscreen surface");
    const auto api = eglQueryAPI();
    if (!eglBindAPI(EGL_OPENGL_API))
        throw eglError("select OpenGL API");
    const EGLint context_attributes[] = {EGL_CONTEXT_MAJOR_VERSION_KHR,
                                         3,
                                         EGL_CONTEXT_MINOR_VERSION_KHR,
                                         3,
                                         EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR,
                                         EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR,
                                         EGL_NONE};
    next->context = eglCreateContext(display, config, EGL_NO_CONTEXT, context_attributes);
    const auto context_error = eglGetError();
    eglBindAPI(api);
    if (next->context == EGL_NO_CONTEXT)
        throw std::runtime_error("create OpenGL 3.3 context failed (EGL " + std::to_string(context_error) + ")");
    {
        Binding binding(display, next->surface, next->context);
        glewExperimental = GL_TRUE;
        const auto status = glewInit();
        // Linux GLEW loads the GL entry points before asking for a GLX display.
        // EGL deliberately has no GLX display. Accept only that specific status.
        if (status != GLEW_OK && status != GLEW_ERROR_NO_GLX_DISPLAY)
            throw std::runtime_error("initialize OpenGL functions failed: " + std::to_string(status));
        if (!GLEW_VERSION_3_3)
            throw std::runtime_error("OpenGL 3.3 is unavailable in the offscreen context");
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
                                        int width, int height, bool color, bool depth) {
    std::lock_guard<std::mutex> lock(host_mutex);
    Binding binding(impl_->display, impl_->surface, impl_->context);
    impl_->renderer->draw(scene, view, appearance, time, width, height);
    return impl_->renderer->captureImage(color, depth);
}
LabelCapture OffscreenRenderer::captureLabels(const Scene &scene, const std::vector<InstanceLabel> &labels,
                                              const View &view, int width, int height) {
    std::lock_guard<std::mutex> lock(host_mutex);
    Binding binding(impl_->display, impl_->surface, impl_->context);
    return impl_->renderer->drawLabels(scene, labels, view, width, height);
}
const std::string &OffscreenRenderer::device() const {
    return impl_->device;
}
} // namespace nereus::rendering
