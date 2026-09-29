# Scene viewer

The optional scene build connects the original mesh/water renderer to the native
viewer. It runs independently of simulation and ROS, using the same source frame
contract for recordings and live pose sources. The supplied Talos example contains
the original body and eight rotors, plus the original pool geometry and water
passes. The live simulation example animates rotors from realized forces; the
recording example retains static rotors. [Original LED bars](INDICATORS.md) can
follow recorded/live source colors; simulation command ownership remains open.
Mechanism animation, complete course content, original
operator panels and camera products remain separate required work.

## Run a recording or simulation

Install the baseline viewer dependencies plus `libassimp-dev`, then:

```sh
cmake --preset scene-viewer
cmake --build --preset scene-viewer
build/scene-viewer/robotics-viewer content/visuals/scenes/talos_workspace.yaml \
  --shaders libraries/rendering/shaders
```

Play/seek the recording to move the model. Orbit/pan/zoom, fixed-frame selection,
source disconnect/reconnect and workspace saving use the existing controls.
**Reload scene** reloads mesh/document data after edits or a loading failure;
loading occurs on explicit open/reload, not every draw. The source clock controls
water time, so playback pause/seek is reproducible. A source-free scene uses time
zero. Missing source/frame bindings hide the entire affected group and show a
message; they never reuse a last-known pose.

For live simulation, compose the same document with the simulation application:

```sh
cmake --preset simulator-viewer -DRP_BUILD_SCENE_RENDERER=ON \
  -DRP_INSTALL_REFERENCE_VISUALS=ON
cmake --build --preset simulator-viewer
build/simulator-viewer/robotics-sim-view content/examples/talos_navigation_pool.yaml \
  --rotors content/visuals/scenes/talos_rotors.yaml \
  --scene content/visuals/scenes/talos_animated_pool.yaml --shaders libraries/rendering/shaders
```

This remains the finite scripted scenario worker, without interactive simulation
controls. `--scene` supplies presentation data only and cannot alter plant physics.
The optional rotor rig supplies moving frames beneath `cad`. Its named force
channels are independent of plant order; all original samples are integrated even
when the viewer drops updates. The simulation adapter supplies `world`→`com`→`cad` frames; the recording
fixture independently supplies its own body→CAD mount. The viewer knows neither
body origin conventions nor robot names.

```sh
cmake --install build/scene-viewer --prefix "$PWD/install/scene-viewer"
install/scene-viewer/bin/robotics-viewer \
  install/scene-viewer/share/robotics_platform/visuals/scenes/talos_workspace.yaml
python3 tools/check_viewer.py --preset scene-viewer --graphics
python3 tools/check_viewer.py --preset scene-viewer-asan --graphics
```

Installed shader discovery uses the Linux executable's sibling
`share/robotics_platform/shaders` directory. Build-tree runs specify `--shaders`.
The existing `viewer` build retains its smaller dependencies and line rendering;
a workspace requesting scene rendering reports that the feature is unavailable.
Graphics checks still require a working display/OpenGL context; hidden GLFW is
not displayless EGL.

## Neutral visual document, version 1

Workspace `scene` is optional and resolves relative to the workspace file. Save As
rebases it to the destination just like source paths. Mesh paths resolve relative
to the scene document. The document contains `version: 1` and a `groups` sequence:

```yaml
version: 1
groups:
  - id: chassis
    source: vehicle
    frame: cad
    instances:
      - mesh: chassis.glb
        pose: {position: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
        tint: [1, 1, 1, 1]
        material: asset
        radiance: 60
        casts_shadow: true
```

Each group has a unique ID and frame. A declared `source` must match the selected
source identity, be connected, and provide a transform from the group's frame to
the workspace fixed frame at the snapshot timestamp. Omitting `source` means a
static asset authored directly in the named fixed frame; that frame must equal
the workspace fixed frame. Frame names never imply alignment between sources.

An instance declares exactly one `mesh` or `box: [x, y, z]`. Boxes use centered
unit geometry scaled to positive metre dimensions. Optional `color_channel` binds
a named color history from the group source at the same snapshot time as its
frame; it multiplies RGB while preserving alpha/radiance. Missing color omits the
group. See [INDICATORS.md](INDICATORS.md) for timing and bounds.

Instance pose defaults to identity, tint to white, material to `asset`, radiance
to 60 and shadow casting to true. Supported overrides are `asset`, `clear` and
`emissive`; asset submesh transparency remains intact. Meshes are immutable and
shared within a loaded document. Textured meshes remain unsupported by the current
GPU renderer and produce a diagnostic rather than silently losing textures.

A group may additionally contain `pool` with `dimensions` (metres), `water_level`,
`deck_height` and optional horizontal `pose`. The sample document records the
original pool's dimensions and map placement. At most one pool/water surface is
supported. A source transform that tilts its surface out of Z-up omits that group
with a diagnostic. Pool lighting stays centered at its transformed pool center,
independent of the observer camera. Scenes without a pool use world origin for
the shadow volume center in this initial schema.

Unknown/duplicate fields, invalid orientations, nonfinite values and duplicate
IDs are rejected. Documents are limited to 1 MiB, 128 groups and 4096 mesh instances;
mesh loading has the existing asset limits. These are trusted local content files,
not a sandbox for adversarial assets. There is no simulation configuration,
transport topic, task dispatch or robot-name selection in this schema.

## Ownership and validation

`RoboticsPlatform::scene_view` composes visualization frames with rendering values
and privately uses YAML for loading. It owns no source, clock, window or runtime.
`resolve()` is stateless: one frozen snapshot supplies both meshes and line displays
for a viewport frame. Disconnect/reset/rewind/source changes cannot resurrect old
transforms. Groups resolve atomically; static content has its own explicit binding.

The desktop copies rendered color and composite depth into its own framebuffer
before drawing overlays. The renderer's borrowed texture handles have the same
resize/destruction lifetime; consumers must not modify them. Consequently viewer
lines do not modify renderer captures or a later camera provider's products.
Camera matrices are available separately as well as through the existing combined
projection API. Renderer/viewer destruction precedes desktop context teardown.

CPU tests cover source identity, disconnect/rewind, static and tilted-water frames,
relative resource loading, strict fields, camera matrices and saved scene paths.
The graphics contract verifies unchanged background color, occlusion behind the
floor, visible foreground lines, resizing and unchanged original captures. The
installed check removes copied source/build trees before loading the full example
with an external public-API consumer and the installed viewer. No simulator or ROS
sources are present in that install build. These checks establish the initial
viewer connection, not full Talos assembly or original UI/mission acceptance.
