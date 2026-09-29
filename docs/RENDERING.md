# Independent scene rendering

`RoboticsPlatform::scene_rendering` renders explicit mesh instances, view/projection
matrices, appearance and simulation time in a caller-owned OpenGL context. It owns
no window, viewer, simulation, ROS node or clock. `Scene` starts empty, with water
absent; the original pool appearance is an optional `makePoolScene()` builder.
This is the original rendering pipeline extracted from the old simulator, not a
replacement water approximation.

```sh
# Additional system dependencies: libassimp-dev libglew-dev libgl1-mesa-dev.
# The capture executable also needs libglfw3-dev and a working DISPLAY.
cmake --preset rendering
cmake --build --preset rendering
build/rendering/robotics-render-capture libraries/rendering/shaders \
  content/visuals/talos build/rendering/captures
python3 tools/check_rendering.py
```

The `rendering` preset disables simulation/CLI and builds the optional CPU mesh
loader, renderer, reference assets and a hidden-window capture executable. It
includes no viewer, ImGui or robot-stack library. `RP_BUILD_SCENE_RENDERER` selects
the library independently of `RP_BUILD_RENDER_CAPTURE`. Normal headless/standalone
viewer builds retain their existing dependency boundaries.

## Public ownership and inputs

- `Instance` shares immutable CPU mesh data and supplies a model matrix, tint,
  material, emission radiance, visibility and shadow selection. Names of robots,
  mechanisms, courses and transport topics do not appear in renderer dispatch.
- `View` supplies separate float view/projection matrices and world-space eye.
  Matrices use OpenGL clip conventions. Camera calibration/optical transforms
  belong to a camera provider, not this renderer.
- `Scene.water` optionally supplies a horizontal surface, dimensions, world Z and
  an XY/yaw local frame. A misplaced/tilted surface is rejected. `Appearance.surface`
  hides surface composition while retaining water optics; removing `Scene.water`
  removes underwater shading entirely, as needed for non-simulation visualization.
- `Appearance` retains original water attenuation/scattering, caustics, lighting,
  reflections, shadows, exposure and glare defaults. Time is an explicit float;
  repeated inputs replay without dependence on wall time.
- Construct/use/destroy the renderer with the same initialized GLEW/OpenGL 3.3
  context current on one thread. Destroy it before context teardown. Draw resets
  relevant GL state and leaves it changed; it is an application pass boundary.
- Returned texture IDs are borrowed. Subsequent draws replace their contents;
  resize/destruction invalidate IDs. Explicit `capture()` copies final RGBA8,
  opaque/composite HDR and nonlinear depth, in bottom-up OpenGL order. It requires
  the latest draw to have succeeded; failed draws cannot expose partial captures.

GPU uploads retain source ownership and validate indices/data. Partially constructed
resources unwind correctly, unused mesh assets are released between scenes, and
framebuffer replacement uses temporary ownership. Allocation/GL errors propagate.
There is one reusable output frame per renderer in this increment. Before camera
integration, add reusable per-view targets/shared scene preparation so differently
sized cameras and the UI do not repeatedly resize or duplicate GPU meshes/shadows.
Diagnostic `capture()` intentionally reads intermediate buffers; camera acquisition
will need selective readback instead of copying all of them.

## Retained pipeline

The implementation preserves original float geometry and uniform calculations,
submesh order, transparent-depth behavior, texture filtering and render formats:

1. 4096² depth24 shadow map, original orthographic light volume, bias/filtering and
   transparent-hull exclusion.
2. Conditional reflection at half dimensions with the original minimum size and
   water-plane clipping.
3. RGBA16F/depth24 scene: opaque meshes, then blended clear materials that still
   write front-cover depth.
4. Color/depth blit, followed by water composition without depth writes.
5. Quarter-resolution bright extraction and two blur passes.
6. Original edge smoothing, glare, exposure, bloom, tone mapping and gamma into RGBA8.

The scene/shadow/water/bloom/post shader pairs retain original source, except an
explicit `waterEnabled` guard in `scene.frag`. With water present the expressions
are unchanged; without it they skip underwater shading. Pool tiles, lane marks,
wall/deck/coping geometry and water quad retain original appearance.

Original headers use GLM 1.0.0. That pinned vendored distribution is retained as a
private implementation dependency with its license; the public API uses Eigen.
CPU mesh normalization explicitly preserves original float dot order and reciprocal
multiplication. Equivalent division changed a few shadow-bias decisions in debug
builds; preserving the original arithmetic restores exact reference output.

## Reference and installed validation

The offline harness compiles the pinned original renderer, original GLM/glad and
unchanged shaders without its main host, ROS or panels. It uses tiny source-format
fixture configs, the original body/rotors and hidden calibration board, then supplies
fixed poses, appearance and time. It records the exact camera matrices separately;
the new renderer consumes those matrices during comparison. Expected pixels are
never produced by platform rendering code. A shared harness-only timing helper
measures complete GPU draws and contains no scene or renderer math.

```sh
python3 tools/capture_render_reference.py ../src/riptide_simulator build/render-reference
python3 tools/check_rendering.py --reference build/render-reference
python3 tools/capture_render_reference.py ../src/riptide_simulator \
  build/render-reference-debug --optimization 0
python3 tools/check_rendering.py --preset rendering-asan --reference build/render-reference-debug
```

Capture additionally requires the original repository, OpenCV, yaml-cpp, Assimp,
GLFW and a C/C++ compiler. Ordinary builds, installed capture and contract checks
require no original workspace. Release comparison uses the original `-O2` build;
debug/sanitizer comparison uses its `-O0` build. Comparisons run on the same GL
backend; pixel equality across different GPUs/drivers/optimization modes is not
claimed. `tools/capture_render_reference.py` records sources,
outputs, native implementation hashes, backend and the measured release baseline.

Six 640×400 scenes cover above water, underwater, near-surface, close transparent
hull, indoor/outdoor, above-water reflections on/off and disabled shadows. All 30
final RGBA, opaque HDR/depth and composite HDR/depth buffers match byte for byte on
the current Apple M1 Pro backend. Depth remains unchanged by water composition.
Images were also inspected; numerical matches are not empty-frame comparisons.

The capture tool checks hostile caller state (front-face winding, scissor, masks,
packing, samplers, primitive restart, logic operations and clipping), deterministic replay, target
resize, time dependence, optional-water isolation, invalid-plane rejection and
recovery after failure. The installed check copies only renderer/asset/context-host
sources, builds and relocates an install, removes its source/build copy and then
runs both an external consumer and GPU captures. ASan/UBSan graphics checks disable
leak detection for driver process-global allocations, as for the existing viewer.

A local 1280×800 benchmark used ten warmups and 120 GPU-complete draws,
with reflection and shadows. Original mean/p99: 8.364/14.501 ms; extracted renderer:
8.246/14.639 ms. These are provisional body/pool measurements on this machine,
excluding UI, sensor readback, tasks, textures and ROS. They do not prove full-stack
frame-rate acceptance. Hidden GLFW contexts work; displayless EGL is not yet tested.

## Remaining integration

The optional [scene viewer](SCENE_VIEWER.md) now presents this renderer alongside
line overlays, using neutral scene documents and frozen source snapshots. The
original Talos assembly, rotor animation, LEDs, mechanisms and course assets still
need completion. Image
texture upload, generic cutout data, point/focus overlays and original operator UI
remain open. Camera/stereo acquisition and original full-stack/task acceptance
follow those dependencies. The current fixed scene is explicitly Talos body/rotors
and pool, not a complete robot, competition or simulator replacement.
