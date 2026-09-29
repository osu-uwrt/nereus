# Source-driven indicators

The standalone viewer can display the original three Talos LED bars using ordinary
box instances, emissive materials and named source color observations. Placement,
dimensions, radiance 240 and disabled shadow casting come from the pinned original
status-light configuration. The renderer contains no LED IDs, command modes, target
masks, clocks or ROS types. The original `Indicator` state calculations remain in
the neutral visualization library; see [ANIMATION.md](ANIMATION.md).

This increment supplies geometry, observation delivery and recording playback.
Simulation-side command ownership, live Talos LED commands and ROS target routing
are still required. `SimulationPosePublisher` rejects configured color channels
until a composed provider owns them; it cannot silently fill unknown state.

## View the bars

```sh
cmake --preset scene-viewer
cmake --build --preset scene-viewer
build/scene-viewer/robotics-viewer content/visuals/scenes/talos_indicator_workspace.yaml \
  --shaders libraries/rendering/shaders
```

The small recording deliberately demonstrates off, broadcast red and individual
red/green/blue observations at source seconds 0, 1 and 2. It is synthetic example
data, not robot telemetry or a command-stream reference. Seek/rewind/disconnect
exercise the same bindings as live sources. Rotors in this recording are static.
The separate live simulation example in [SCENE_VIEWER.md](SCENE_VIEWER.md) animates
rotors but does not yet receive indicator commands.

Reproduce the imported bar document and its body/pool composition with:

```sh
python3 tools/import_talos_visuals.py ../src/riptide_simulator
python3 tools/import_talos_visuals.py ../src/riptide_simulator --check
```

The importer verifies 19 outputs, including original config provenance, neutral
`talos_indicators.yaml`, and the composed `talos_indicator_pool.yaml`. The manifest
records pinned source hashes, importer hash and the local pool-composition input
hash. Transport fields and target masks remain in the historical provenance copy;
they are absent from runtime scene data.

## Observation and scene contracts

`SourceData::colors` maps names to `ColorHistory`. Samples contain explicit source
nanoseconds and normalized linear RGB floats. Histories have strictly increasing
nonnegative times. `colorAt()` holds the preceding sample between observations,
including the sample exactly at the query time. It neither interpolates across
state transitions nor extrapolates before/after the observed range. A recording
must include an endpoint sample when it wants a held color through its duration.
This does not reconstruct a command history or evaluate flashing between samples;
the owning producer must sample `Indicator` at the required acquisition times.

A version-1 recording may add:

```yaml
colors:
  - id: status
    samples:
      - {time_ns: 0, rgb: [0, 0, 0]}
      - {time_ns: 1000000000, rgb: [1, 0, 0]}
      - {time_ns: 2000000000, rgb: [1, 0, 0]}
```

There are at most 64 color channels, 10000 samples per channel and 100000 combined
pose/color samples per recording. `LivePoseOptions::color_channels` declares the
ordered color batch in each `PoseUpdate`. Body, moving poses and all colors are
validated and accepted as one timestamped observation. Queue loss and history
trimming operate on complete batches. Producer resets discard the old generation;
reconnect seeds the newest complete batch. Retained snapshots stay immutable.
Configured live frame/color histories share a 100000-sample bound. All packet
vectors are preallocated; history construction stays outside the producer lock.

A scene instance can use a relative mesh path or a centered unit-box primitive
scaled to the supplied positive dimensions, and optionally bind its RGB:

```yaml
version: 1
groups:
  - id: status-emitter
    source: robot
    frame: housing
    instances:
      - box: [0.012, 0.055, 0.003]
        pose: {position: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
        color_channel: status
        material: emissive
        radiance: 240
        casts_shadow: false
```

The group must declare a source. Color and frame lookup use the same frozen
snapshot timestamp and identity. Missing color, unavailable frames and disconnects
omit the affected group with a diagnostic. Resolved color multiplies the authored
RGB tint while preserving alpha and radiance. Black is zero emission, not invisible
geometry: the bars still participate in depth, and water scattering can affect
their visible surface. `rendering::makeBoxMesh()` exposes shared CPU unit geometry;
no window or simulation is needed to construct it.

## Independent visual acceptance

```sh
DISPLAY=:0 python3 tools/capture_render_reference.py ../src/riptide_simulator build/indicator-reference
DISPLAY=:0 python3 tools/check_rendering.py --reference build/indicator-reference
```

The unchanged original renderer receives its original status-light configuration.
The capture adds three fixed close-ups to the existing six body/rotor/pool cases:
off, red and individual RGB. It exports the exact local geometry/radiance and view
inputs; the native capture constructs neutral emissive instances from those inputs.
The original six cases and their benchmark remain on a scene without emitters.
Scene parsing, imported config conversion and source binding are checked separately
from this renderer comparison.

All 45 buffers match byte for byte on the recorded backend: final RGBA, opaque
HDR/depth and composite HDR/depth for nine cases. LED color changes preserve depth;
red and individual RGB differ from off in 3460 and 3452 final pixels respectively.
The reference and the interactive recording were visually inspected, including
emission through the original clear hull. This covers the selected fixed views,
not every possible camera, animated image or complete robot-stack timing.

The source/output hashes and backend are recorded in
[indicator_rendering_baseline.json](reference/indicator_rendering_baseline.json).
The numerical 402-sample original indicator-mode reference remains separate from
this geometry/shader check. Full ROS routing, acquisition-time sampling and reset
integration must be validated when their owning adapters are implemented.
