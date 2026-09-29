#pragma once
#include "robotics/rendering/scene.hpp"

namespace robotics::rendering {
struct RenderedFrame {
    std::uint32_t color_texture =
        0; // Borrowed GL texture; contents change on draw, ID invalidated by resize/destruction.
    int width = 0, height = 0;
    std::uint32_t depth_texture = 0; // Borrowed read-only composite depth; same lifetime as color.
};
struct Capture {
    int width = 0, height = 0;
    // All rows start at the bottom (OpenGL convention). Depth is nonlinear [0,1].
    std::vector<std::uint8_t> rgba;
    std::vector<float> opaque_rgba, opaque_depth, composite_rgba, composite_depth;
};
// Context-owned OpenGL 3.3 renderer. Caller initializes GLEW, keeps the same context
// current on the owning thread, and destroys Renderer before the context.
// Does not create a window, read a clock, or own simulation/transport state.
// Draw resets relevant GL state, then leaves state changed and framebuffer zero bound.
// Meshes must remain immutable while shared. Unused GPU assets are released each draw.
class Renderer {
  public:
    explicit Renderer(const std::filesystem::path &shader_directory);
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;
    RenderedFrame draw(const Scene &, const View &, const Appearance &, float time, int width,
                       int height);
    // Requires the most recent draw to have completed successfully.
    Capture capture() const; // Explicit synchronous readback; no per-frame CPU copy otherwise.

  private:
    struct Resources;
    std::unique_ptr<Resources> resources_;
};
} // namespace robotics::rendering
