#pragma once
#include "robotics/rendering/renderer.hpp"
#include <string>

namespace robotics::rendering {
// Optional EGL host for display-free OpenGL capture. No window, simulation, transport,
// clock or worker thread. Calls are serialized; the context is bound only within a call,
// so a camera worker can use an instance created on another thread. Scene/mesh inputs
// must remain immutable during capture. Returned buffers own their pixels.
// Hosts initialize a process-wide EGL surfaceless display and leave it initialized;
// callers must not terminate that display while hosts exist.
// A caller's prior EGL context is restored. GLX contexts must live on a different thread.
// Other GLEW users must serialize their initialization against these hosts.
class OffscreenRenderer {
  public:
    explicit OffscreenRenderer(const std::filesystem::path &shader_directory);
    ~OffscreenRenderer();
    OffscreenRenderer(const OffscreenRenderer &) = delete;
    OffscreenRenderer &operator=(const OffscreenRenderer &) = delete;
    ImageCapture capture(const Scene &, const View &, const Appearance &, float time, int width, int height,
                         bool color = true, bool depth = true);
    const std::string &device() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace robotics::rendering
