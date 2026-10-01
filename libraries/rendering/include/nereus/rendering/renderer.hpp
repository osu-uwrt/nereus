#pragma once
#include "nereus/rendering/scene.hpp"

namespace nereus::rendering {
struct RenderedFrame {
    // Borrowed GL texture of the requested output size; contents change on draw, ID invalidated by
    // resize/destruction.
    std::uint32_t color_texture = 0;
    int width = 0, height = 0;
    std::uint32_t depth_texture = 0; // Borrowed read-only composite depth; same lifetime as color.
    // depth_texture's size: Appearance::supersample x width/height (the scene passes' size).
    int depth_width = 0, depth_height = 0;
};
struct Capture {
    int width = 0, height = 0;             // rgba (the output)
    int scene_width = 0, scene_height = 0; // opaque_* and composite_*: supersample x width/height
    // All rows start at the bottom (OpenGL convention). Depth is nonlinear [0,1].
    std::vector<std::uint8_t> rgba;
    std::vector<float> opaque_rgba, opaque_depth, composite_rgba, composite_depth;
};
// Sensor-sized readback: final tone-mapped RGB8 and opaque-pass depth (nonlinear [0,1],
// before the water surface is composited). Rows start at the bottom; omitted outputs are
// empty vectors. Both are width x height. With Appearance::supersample n > 1 the depth is one
// scene sample per n x n block, never an average across an edge: the sample at block offset
// (n/2, n/2) (integer division), whose centre is the pixel centre for odd n and 1/(2n) pixel right of and
// above it for even n.
struct ImageCapture {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb;
    std::vector<float> depth;
};
// Label pass: per scene instance an id (24 bits; 0 = occludes only, never labelled), per submesh a fixed part
// value or an 8-bit part map sampled nearest at the submesh UVs (same row convention as diffuse textures:
// uv (0, 0) is the bottom-left of the PNG, wrap repeats). Pixel = id << 8 | part (part 0 = the instance with
// no part there). Water surface and Marking instances are not drawn; Clear submeshes (material Clear, or
// untextured Asset with base alpha < .999) are drawn only when their instance id is nonzero (a labelled clear
// cover). Diffuse texels with alpha < .4 and UV cutouts discard as in colour, except that a cutout fragment on
// a part-mapped submesh of a nonzero id is kept where the map is nonzero (a ring's value fills its hole); id 0
// writes 0 everywhere. Depth-tested; depth is nonlinear [0,1] like ImageCapture, from the same vertex
// arithmetic as the colour pass (equal where both draw the same fragments on the drivers tested). Rows start
// at the bottom.
struct SubmeshLabel {
    std::uint8_t part = 0;
    std::optional<std::filesystem::path> part_map; // 8-bit PNG (gray, or first channel); overrides `part`
};
struct InstanceLabel {
    std::uint32_t id = 0;                // < 2^24
    std::vector<SubmeshLabel> submeshes; // empty = every submesh part 0; else one per submesh
};
struct LabelCapture {
    int width = 0, height = 0;
    std::vector<std::uint32_t> ids;
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
    // state like capture() and leaves framebuffer zero bound for reading. A supersampled depth readback
    // first draws the per-pixel sample into an internal target (changing viewport, program and depth state).
    ImageCapture captureImage(bool color = true, bool depth = true) const;
    // labels: one per scene.instances entry (same order). Independent of draw(): may be called before or
    // after it, renders into its own target and leaves the last draw's frame and captures intact. Meshes and
    // diffuse textures share draw()'s cache (uploaded once for both; released by the next full draw when no
    // longer drawn there); part maps stay cached while the latest call references them (visible or not), and
    // every referenced map is loaded, so a bad path throws. Leaves framebuffer zero bound. Validates like draw().
    LabelCapture drawLabels(const Scene &, const std::vector<InstanceLabel> &labels, const View &, int width,
                            int height);
    // Terminal cleanup after context loss: release CPU state without any GL calls.
    // The host must destroy the context to reclaim its GPU allocations. Idempotent;
    // drawing/capture is no longer allowed after this call.
    void abandonContext() noexcept;

  private:
    struct Resources;
    std::unique_ptr<Resources> resources_;
};
} // namespace nereus::rendering
