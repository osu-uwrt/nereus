# Standalone viewer

The first viewer slice is a native C++ desktop application using Dear ImGui,
GLFW, and OpenGL 3.3. It opens an empty workspace without a robot, simulator,
Python, ROS, or course assets. The bundled local recording demonstrates the
same neutral pose/frame displays that future adapters can supply.

This is an initial visualization host, not an RViz replacement yet. It supports
line geometry: a grid, frame axes, a box-and-axes pose glyph, and pose trajectories.
Robot meshes/joints, markers, images, clouds, ROS/network connections, simultaneous
source overlays, and simulation attachment are not implemented. The local YAML
fixture format is deliberately small; it is not a general recording system.

## Build, run, and install

Tested baseline: Ubuntu 22.04, GCC 11, CMake 3.22+, Eigen, yaml-cpp, GLFW 3.3,
GLEW 2.2, OpenGL 3.3, and GoogleTest for tests. Dear ImGui v1.91.9b is vendored
with its MIT license. There are no configure-time or runtime downloads.

```sh
sudo apt-get install cmake g++ libeigen3-dev libyaml-cpp-dev libgtest-dev \
  libglfw3-dev libglew-dev libgl1-mesa-dev
cmake --preset viewer
cmake --build --preset viewer
ctest --preset viewer
./build/viewer/robotics-viewer workspaces/local_demo.yaml
```

Run without the final path for an empty workspace. The `viewer` preset sets
`RP_BUILD_SIMULATION=OFF`, `RP_BUILD_CLI=OFF`, and `RP_BUILD_VIEWER=ON`. It neither
builds nor installs simulation/sensor/config libraries or Python bindings.
An X11/Wayland desktop with an OpenGL 3.3 implementation is required, including
for `--hidden` captures. Missing graphics support produces an explicit error.

```sh
cmake --install build/viewer --prefix "$PWD/install/viewer"
./install/viewer/bin/robotics-viewer \
  install/viewer/share/robotics_platform/workspaces/local_demo.yaml
```

For the neutral libraries without graphics discovery, configure
`RP_BUILD_VISUALIZATION=ON`, `RP_BUILD_VIEWER=OFF`, `RP_BUILD_SIMULATION=OFF`,
and `RP_BUILD_CLI=OFF`. Existing simulation presets keep all viewer features off.

## Interaction

- Enter a workspace path and choose **Open**. Invalid workspace documents leave the existing workspace
  open. A missing source or display extension is reported locally; other sources
  and displays can still be used. **Empty** removes sources and restores the grid.
- Select a source to inspect. The two example sources use identical frame/stream
  names but maintain independent playback positions and generations. Only the
  selected source is drawn; there is no implicit clock or frame alignment.
- **Play**, **Pause**, **Rewind**, speed selection, and the timeline control local
  playback. The nanosecond input applies exact positioning on Enter; the slider is a
  convenience with floating-point UI precision. Wall pacing uses a steady clock,
  with each UI interval capped at 250 ms after stalls. Playback stops at the end.
- Select or enter a fixed frame (Enter applies typed text). Missing transforms
  hide affected geometry and show a diagnostic. Toggle displays independently.
- Drag the view to orbit, right-drag to pan, and scroll to zoom. With the scene
  focused, arrow keys orbit and `+`/`-` zoom. ImGui keyboard navigation is enabled.
  **Reset camera** restores the default orbit. Axes are X/red, Y/green, Z/blue.
- **Disconnect** invalidates only that source. **Reconnect** starts local playback
  at zero in a new generation. Viewing and camera actions issue no robot commands.
- **Save as** saves current source bindings, selected source, fixed frame, camera,
  and display settings to the entered path. Source paths are rebased relative to
  the destination. Playback position/speed are transient, not persisted. Saving
  uses an exclusive temporary file and atomic replacement on the supported POSIX
  platform; failed writes preserve an existing file. This is not a crash-durable
  transaction across multiple files.

The initial layout is fixed, with scrollable properties and a resizable viewport.
Arbitrary docking/layout persistence is deferred. A system DejaVu font is used
when available, with ImGui's bundled font as a fallback.

## Contracts and extension boundaries

All current C++ contracts are version 0.1, source-level interfaces; a stable binary
plugin ABI and runtime shared-library discovery are not provided. Extensions are
explicit trusted code registered at application construction, not sandboxed code.

| Target | Responsibility and dependencies |
| --- | --- |
| `RoboticsPlatform::visualization` | Geometry, integer time, frame tree, source/playback interfaces, displays, view camera. Eigen and standard C++ only. |
| `RoboticsPlatform::viewer_io` | Strict versioned YAML, local source construction, session composition, workspace saves. Visualization and yaml-cpp. |
| `RoboticsPlatform::rendering` | Line rendering into the caller's framebuffer. Visualization, OpenGL, GLEW. Exported when the viewer is enabled. |
| `robotics-viewer` | Window/context, ImGui controls, presentation pacing, framebuffer lifetime. No simulation or ROS. |

`Source::snapshot()` returns a source ID, generation, presentation cutoff time, and
shared immutable `SourceData`. Data identifies its clock and owns bounded pose
histories and an immutable frame graph. A null data pointer means disconnected.
Retained snapshots remain valid after source removal; the host uses the current
snapshot, never retained disconnected data. A source owns its subscriptions or
workers and releases them in its destructor; `disconnect()` must also stop its IO.
Adapters must validate incoming data before publishing a snapshot. Poses are
finite, use unit quaternions, and histories are strictly increasing in time.

`Playback` is a separate optional capability. A live source need not implement
seeking, duration, or simulation controls. `Connection` shares lifetime between
the source and any optional capabilities. `Sources` is an explicitly injected
factory map; a factory receives a source-specific configuration path and ID.
No global registry or robot-name dispatch exists. Reconnect is optional.

`Displays` registers ordinary functions from `DisplayContext` to `DisplayResult`.
The context contains a source snapshot, fixed frame, and validated settings;
the result contains neutral colored line segments and status. An unknown display
or throwing extension produces a local error. Display geometry is bounded to
20,000 segments per display and 200,000 per scene. New payload families can add
their own typed stream contracts; they should not impersonate pose samples.

The [external example](../examples/viewer/main.cpp) registers a live-style source
with no playback and a custom display against installed headers:

```sh
cmake -S examples/viewer -B build/viewer-extension \
  -DCMAKE_PREFIX_PATH="$PWD/install/viewer"
cmake --build build/viewer-extension
./build/viewer-extension/viewer_extension
```

All calls and graphics ownership are currently on one UI thread. No background
callback queue or synchronization is provided. A future asynchronous adapter must
introduce a bounded queue and explicit overflow/receipt-age reporting at that
boundary. Sources supply data; displays and the renderer do not drive physics.
The line renderer requires an initialized GLEW loader and a current OpenGL 3.3
context; its draw calls change GL state, so the caller brackets its render passes.
It is a small reusable backend, not a camera sensor renderer. It
uses float GPU coordinates, a 0.01–100,000 m clipping range, and a viewport capped
at 4096 pixels per dimension. Huge global coordinates should be rebased by adapters.

## Frames, time, and validity

- Time is nonnegative signed 64-bit integer nanoseconds within one named source
  clock. Sources never share a frame namespace implicitly, even if clock strings
  or frame names match. Multi-source overlay requires a future explicit alignment.
- Positions use metres in right-handed frames; quaternions are `w,x,y,z` and map
  child/local coordinates into parent coordinates. The viewer's orbit convention
  is Z-up. Adapters must convert foreign unit/frame conventions at the boundary.
- `FrameGraph` is an immutable tree with one root, one parent per child, no cycles,
  and at most 128 edges. Static edges have exactly one pose valid at every time.
  Dynamic edges interpolate translation linearly and orientation by shortest-path
  slerp between bracketing samples. Exact samples remain addressable above 2^53 ns.
  There is no extrapolation or substitution of the latest transform for history.
- Lookup resolves through the nearest common ancestor. Static relative transforms
  remain available even when unrelated upstream dynamic history is unavailable.
- Pose glyphs use the latest sample at or before presentation time, resolved at
  **that sample's timestamp**, and hide when older than `max_age_ns`. Trajectories
  resolve every historical sample at its own timestamp; missing transforms break
  segments. Historical trails remain visible independently of current-pose age.
- Frame axes intentionally follow the selected frame at **presentation time**.
  This differs from measured-time placement. The grid is a display-local XY plane
  in the chosen fixed frame; it does not imply a physical floor.
- Backward seek, disconnect, and reconnect change the source generation. This
  implementation rebuilds display geometry from the current snapshot, so future
  trail segments and stale source objects cannot survive a rewind. There are no
  mutable display caches. Future caches must key by source, generation, frame,
  and time, and invalidate on source removal.

## Local formats (version 1)

See [workspace](../workspaces/local_demo.yaml) and
[recording](../workspaces/local_motion.yaml) for complete examples.
Both parsers reject unknown/duplicate fields, unsupported versions, invalid
numbers/quaternions, and documents over 16 MiB. Diagnostics include the filename.

A workspace requires `version`, `sources`, `selected_source`, `fixed_frame`, and
`displays`. Optional `camera` contains `target`, `yaw`, `pitch` (radians), and
`distance` (metres). Source entries contain unique `id`, `type`, and `file`;
paths resolve relative to the workspace, not the process working directory.
There are at most 16 source bindings and 64 displays.

Displays require unique `id` and `type`; optional fields are `source`, `stream`,
`frame`, `enabled`, `history_limit` (1–10,000), `max_age_ns` (default 1 second),
`scale` (metres, default 0.5), and RGB integer `color` (0–255). `pose` and
`trajectory` bind an explicit source and pose stream; `axes` binds a source/frame.
`grid` requires no source. The pose glyph is a box of dimensions `2*scale`,
`scale`, `scale/2`, with axes; it is not a robot collision or mesh description.

A recording requires `version`, `clock`, `duration_ns`, `root`, `frames`, and
`streams`. Frame entries contain `parent`, `child`, `static`, and `samples` with
`time_ns`/`pose`. Stream entries contain unique `id` and samples with `time_ns`,
`frame`, and `pose`. Every pose has `position` and `orientation_wxyz`.
Pose sample times must lie within `[0, duration_ns]`. Unknown pose frames remain
visible diagnostics at display time, allowing missing-transform fixtures.
There are at most 64 streams, 10,000 samples per stream/edge, and 100,000 total
pose samples and 100,000 total transform samples. Exceeding a limit fails loading;
there is no silent recording decimation. A display's shorter trail limit reports
that truncation in its status.

## Checks

```sh
python3 tools/check_viewer.py --tidy --graphics
python3 tools/check_viewer.py --preset viewer-asan --graphics
```

`--graphics` requires a desktop (or a configured Xvfb/Mesa display). Without it,
the command still verifies native contracts and the installed external extension.
It builds a copy that physically omits simulation/Python sources and content,
relocates the install, removes copied source/build trees, then consumes the public
libraries and optionally runs the installed UI. Captures are under `build/viewer/`.
GUI sanitizer runs disable LeakSanitizer for driver process globals; CPU contract
and installed-extension tests retain it. Address/undefined-behavior checks remain
enabled. See STATUS.md for the actual local results and hardware used.

For a reproducible capture:

```sh
./install/viewer/bin/robotics-viewer workspaces/local_demo.yaml \
  --hidden --frames 3 --time-ns 6000000000 --screenshot build/viewer.ppm
```

PPM captures exercise framebuffer output; they are not a recording/export feature.
The GUI paces frames to approximately 60 Hz with swap synchronization. No claim
of large-cloud performance or full RViz workflow coverage is made by this slice.
