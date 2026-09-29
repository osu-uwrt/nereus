# Rotor and indicator state

The neutral visualization library exposes `RotorAnimator`, `pivotRotation()` and
`Indicator` in `robotics/visualization/animation.hpp`. They reproduce the original
rotor/LED state calculations without robot identifiers, ROS message types, resource
loading, graphics or wall clocks. The simulation adapter now publishes moving
rotor frames for the interactive scene. Original LED bars now consume named
color observations in standalone scenes; live simulation command ownership is
still pending. See [INDICATORS.md](INDICATORS.md).

## Rotor timing and ownership

Construct `RotorAnimator` from an explicit `RotorAnimation`: two four-coefficient
signed force-to-RPM curves, a force deadband, speed scale, sample timeout, input
count and a list of input-index/direction bindings. Force inputs are float newtons,
matching the original observation representation; angle and timing calculations
use doubles. Each curve computes
`c0 + c1*f + c2*tanh(f) + c3*abs(f)^0.25`. The deadband includes its boundary, and
curve output cannot reverse the sign of the input because of a fitted intercept.

`receive(forces, seconds)` integrates the preceding sample up to replacement time
before accepting the new sample. `advance(seconds)` integrates only until the
sample's expiry, wraps phase with remainder into [-π,π], and freezes for a paused
clock. There is no artificial speed or frame-step cap. Backward time resets phases
and forces before establishing the new clock position. Explicit `reset()` also
clears timing state. Configurations contain 1–256 inputs and rotor bindings.

The owner must process all realized-force samples independently of presentation
consumption, then publish owned phase/pose observations. Do not integrate forces
from a lossy viewer queue or from render-frame wall time. This same ownership rule
will let sensor rendering observe animation at acquisition time without a desktop.
`angles()` borrows the current internal vector; copy it when publishing snapshots.
Instances own all mutable state and have no shared clock/cache.

`pivotRotation(pivot, axis, radians)` returns the rigid transform rotating a mesh
about its local shaft. It accepts a finite nonzero axis, normalizes it, and preserves
both the pivot and points on the shaft. Compose this transform beneath the body/CAD
mount exactly once. It uses Eigen/double quaternion arithmetic, whereas the original
uses float GLM matrices and casts the angle to float. The current 1e-6 matrix
comparison is numerical parity, not proof of byte-identical animated images.

## Moving frames and simulation composition

`LivePoseOptions::moving_frames` declares parent/child edges. Each `PoseUpdate`
contains exactly one local pose for each edge in that order, alongside the body
pose at the same timestamp and producer generation. The entire batch is validated
before delivery. Queue overflow, reconnect and reset apply to complete batches;
retained snapshots stay immutable. Pending, drain and latest packet buffers are
allocated at construction. Frame histories are rebuilt on the consumer outside the
producer mutex. The existing 128-edge/100000-total-sample frame-graph bounds apply
to the combined topology and configured history capacity.

`RotorRig` binds named force inputs, animation settings and local shaft mounts.
The strict version-1 YAML loader is `viewer::loadRotorRig()`. It contains no robot
physics, mesh paths or transport topics. The imported Talos rig preserves original
curve coefficients, input order, directions, pivots and axes; reproduce it with
`tools/import_talos_visuals.py` and verify using `--check`.

`integrations::SimulationPosePublisher` maps the rig's input names to declared
plant channels and integrates every realized-force observation before lossy viewer
delivery. Construct it as the sole producer for its `LivePoseSource`. Animated
observations must have consecutive ticks and increasing source timestamps within a
generation. A new generation clears phase. Missing ticks, malformed forces and
invalid geometry reject before committing animation. Two preallocated animator
buffers make this transactional without allocating a new candidate each tick.
Neither graphics polling nor connection state affects integration or simulation.

The application composes this adapter with `--rotors RIG.yaml` and the scene's
moving-frame groups. See [SCENE_VIEWER.md](SCENE_VIEWER.md) for the animated Talos
command. Frame history provides rigid-pose interpolation, not unwrapped rotor
phase reconstruction; fast spins between retained samples can alias. The live
scene resolves at its newest complete batch. Future camera acquisition must observe
animation directly at acquisition time, independently of display history.

## Indicator commands

`Indicator::command(rgb, mode, seconds, pulse_duration)` clamps finite RGB to [0,1].
Modes are solid, slow flash (two-second period), fast flash (half-second period),
breath (three-second period), and a temporary pulse. Flash/breath phase uses the
supplied source time, matching paused-clock behavior. `color(seconds)` is a pure
query of the retained command state, with no sampling/rendering side effects.

A pulse overlays the underlying mode over `[start, end)`. Another pulse replaces
it; an underlying mode/color command during a pulse takes effect after the pulse
ends. `reset()` clears the underlying state and pending pulse. Source owners must
reset on generation changes and replay commands when seeking; retaining a future
pulse from another generation is not allowed. This model does not reconstruct a
command history itself. Routing target masks to emitter IDs belongs in the content
and transport adapter, not the renderer or this per-indicator model.

All time arguments are finite nonnegative seconds in a caller-declared clock.
Invalid configurations, packets, modes and values throw before mutation. This is
an intentional boundary improvement over original silent rejection. Computation
that would overflow also reports failure instead of generating nonfinite states.
Pulse duration must be finite and positive. Clamping finite RGB preserves the
original input policy; malformed nonfinite RGB never changes state.

## Reference and validation

```sh
python3 tools/capture_animation_reference.py ../src/riptide_simulator
python3 tools/check_viewer.py --preset viewer
python3 tools/check_viewer.py --preset viewer-asan
```

The offline capture compiles unchanged original `thruster_visuals.hpp`,
`status_lights.hpp`, `camera.hpp` and their pinned dependencies/configurations in a
temporary directory. It requires a C++ compiler, PyYAML, yaml-cpp and OpenCV headers
from the original helper's include graph, but no ROS node or graphics context.
Expected outputs are never generated by the new implementations. CSV fixtures and
source archive/driver/output hashes are recorded in
the output of `tools/capture_animation_reference.py` (not kept in the repo).

The reference covers 169 rotor events across all eight original bindings, including
replacement between observations, pauses, timeout, direction/deadband and rewind.
Double phases compare at 1e-10 radians and all local matrix elements at 1e-6. The
402 indicator samples cover mode phases/boundaries, pulse override/replacement,
exclusive expiry and input clamping; float output channels match the original.
Additional tests cover rejected-packet timeout preservation, explicit reset,
independent instances and shaft invariance. The installed viewer extension exercises
both models without a window or simulation.

Next: simulation-side indicator command ownership and sampling, followed by
optional ROS routing and animated whole-assembly comparisons. Original LED
geometry/color bindings and fixed-image comparisons are now covered in INDICATORS.md. ROS target routing, complete Talos mechanisms,
camera acquisition and full-stack acceptance remain open. The original source-clock
formulas are preserved; whole-stack delivery/scheduling is not yet validated here.
