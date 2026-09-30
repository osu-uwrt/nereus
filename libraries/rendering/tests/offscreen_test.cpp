#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <future>
#include <gtest/gtest.h>
#include <robotics/rendering/offscreen.hpp>

namespace r = robotics::rendering;
namespace {
r::Scene scene() {
    r::Scene result;
    r::Instance box;
    box.mesh = r::makeBoxMesh();
    box.transform(2, 3) = -3;
    result.instances.push_back(box);
    return result;
}
r::View view() {
    r::View result;
    result.projection.setZero();
    result.projection(0, 0) = result.projection(1, 1) = 2;
    result.projection(2, 2) = -100.05f / 99.95f;
    result.projection(2, 3) = -10.f / 99.95f;
    result.projection(3, 2) = -1;
    return result;
}
r::Appearance appearance() {
    r::Appearance result;
    result.shadows = result.reflections = false;
    result.caustics = 0;
    return result;
}
r::ImageCapture capture(r::OffscreenRenderer &host, int width = 32) {
    return host.capture(scene(), view(), appearance(), 0, width, 32);
}
} // namespace
TEST(Offscreen, CapturesWithoutAWindowAndOwnsPixelsAcrossResize) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(host.device().empty());
    const auto image = capture(host);
    EXPECT_EQ(image.rgb.size(), 32u * 32 * 3);
    EXPECT_EQ(image.depth.size(), 32u * 32);
    EXPECT_LT(image.depth[16 * 32 + 16], 1);
    EXPECT_EQ(image.depth[0], 1);
    const auto saved = image.rgb;
    const auto resized = capture(host, 17);
    EXPECT_EQ(resized.rgb.size(), 17u * 32 * 3);
    EXPECT_EQ(image.rgb, saved);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
}
TEST(Offscreen, CanMoveCaptureToAWorkerAndSerializeConcurrentCalls) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto expected = capture(host);
    auto first = std::async(std::launch::async, [&] { return capture(host); });
    auto second = std::async(std::launch::async, [&] { return capture(host); });
    EXPECT_EQ(first.get().rgb, expected.rgb);
    EXPECT_EQ(second.get().depth, expected.depth);
    EXPECT_EQ(capture(host).rgb, expected.rgb);
}
TEST(Offscreen, HostsShareDisplayLifetimeAndRecoverFromInvalidInput) {
    auto first = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    r::OffscreenRenderer second(NEREUS_RENDERING_SHADERS);
    const auto expected = capture(second);
    first.reset();
    EXPECT_EQ(capture(second).depth, expected.depth);
    EXPECT_THROW(capture(second, 0), std::invalid_argument);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    EXPECT_EQ(capture(second).rgb, expected.rgb);
}
TEST(Offscreen, FailedConstructionReleasesItsContext) {
    EXPECT_THROW(r::OffscreenRenderer("/nonexistent/nereus-shaders"), std::runtime_error);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}
TEST(Offscreen, RestoresCallersEglContextAndApiAfterSuccessAndFailure) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    ASSERT_NE(get, nullptr);
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config;
    EGLint count;
    ASSERT_TRUE(eglChooseConfig(display, attributes, &config, 1, &count));
    ASSERT_EQ(count, 1);
    const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    const auto surface = eglCreatePbufferSurface(display, config, size);
    ASSERT_NE(surface, EGL_NO_SURFACE);
    ASSERT_TRUE(eglBindAPI(EGL_OPENGL_API));
    const auto context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
    ASSERT_NE(context, EGL_NO_CONTEXT);
    ASSERT_TRUE(eglMakeCurrent(display, surface, surface, context));
    EXPECT_NO_THROW(capture(host));
    EXPECT_EQ(eglGetCurrentContext(), context);
    EXPECT_EQ(eglGetCurrentSurface(EGL_DRAW), surface);
    EXPECT_EQ(eglGetCurrentSurface(EGL_READ), surface);
    EXPECT_EQ(eglQueryAPI(), EGL_OPENGL_API);
    EXPECT_THROW(capture(host, -1), std::invalid_argument);
    EXPECT_EQ(eglGetCurrentContext(), context);
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglDestroySurface(display, surface);
    eglBindAPI(EGL_OPENGL_ES_API);
    EXPECT_NO_THROW(capture(host));
    EXPECT_EQ(eglQueryAPI(), EGL_OPENGL_ES_API);
}

TEST(Offscreen, HostCanBeDestroyedOnAWorkerThread) {
    auto host = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    capture(*host);
    std::async(std::launch::async, [host = std::move(host)]() mutable {
        host.reset();
        EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    }).get();
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}

TEST(Offscreen, LastHostDoesNotInvalidateAnExternalContextOnTheSameDisplay) {
    auto host = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config;
    EGLint count;
    ASSERT_TRUE(eglChooseConfig(display, attributes, &config, 1, &count));
    ASSERT_EQ(count, 1);
    const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    const auto surface = eglCreatePbufferSurface(display, config, size);
    ASSERT_NE(surface, EGL_NO_SURFACE);
    ASSERT_TRUE(eglBindAPI(EGL_OPENGL_API));
    const auto context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
    ASSERT_NE(context, EGL_NO_CONTEXT);
    ASSERT_TRUE(eglMakeCurrent(display, surface, surface, context));
    host.reset();
    EXPECT_EQ(eglGetCurrentContext(), context);
    EGLint width = 0;
    EXPECT_TRUE(eglQuerySurface(display, surface, EGL_WIDTH, &width));
    EXPECT_EQ(width, 1);
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglDestroySurface(display, surface);
    eglBindAPI(EGL_OPENGL_ES_API);
}

TEST(Offscreen, ReinitializesDisplayTerminatedBetweenHostLifetimes) {
    {
        r::OffscreenRenderer first(NEREUS_RENDERING_SHADERS);
        capture(first);
    }
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    ASSERT_TRUE(eglTerminate(display));
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}
