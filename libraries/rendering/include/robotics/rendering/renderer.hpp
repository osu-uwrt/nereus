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
// Sensor-sized readback: final tone-mapped RGB8 and opaque-pass depth (nonlinear [0,1],
// before the water surface is composited). Rows start at the bottom; omitted outputs are
// empty vectors.
struct ImageCapture {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
};
// Context-owned OpenGL 3.3 renderer. Caller initializes GLEW, keeps the same context
// current on the owning thread, and destroys Renderer before the context.
// Does not create a window, read a clock, or own simulation/transport state.
// Draw resets relevant GL state, then leaves state changed and framebuffer zero bound.
// Meshes must remain immutable while shared. Unused GPU assets are released each full (non-preview) draw.
// Diffuse textures are 8-bit PNG files (at most 16384 pixels per side and 256 MiB),
// uploaded with rows flipped, sRGB internal format, trilinear
// mipmaps, repeat wrapping, raw stored values (no gamma chunk conversion), and a
// textured submesh drawn with an opaque white base color.
class Renderer {
  public:
    explicit Renderer(const std::filesystem::path &shader_directory);
    ~Renderer();
    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;
    RenderedFrame draw(const Scene &, const View &, const Appearance &, float time, int width, int height);
    // Requires the most recent draw to have completed successfully.
    Capture capture() const; // Explicit synchronous readback; no per-frame CPU copy otherwise.
    // Requires the most recent draw to have completed successfully. Resets pixel-pack
    // state like capture() and leaves framebuffer zero bound for reading.
    ImageCapture captureImage(bool color = true, bool depth = true) const;
    // Terminal cleanup after context loss: release CPU state without any GL calls.
    // The host must destroy the context to reclaim its GPU allocations. Idempotent;
    // drawing/capture is no longer allowed after this call.
    void abandonContext() noexcept;

  private:
    struct Resources;
    std::unique_ptr<Resources> resources_;
};
} // namespace robotics::rendering
