# Shared poses and rigid mounts

`RoboticsPlatform::spatial` owns small Eigen-based transform values and immutable
named rigid mounts. It has no simulation, graphics, clock, middleware or robot
configuration dependency. Visualization reuses these values; its separate
`FrameGraph` still owns timestamped histories/interpolation and missing-data rules.

`Pose` maps child coordinates into its parent: `p_parent = translation + rotation *
p_child`. Translation is SI metres; quaternion rotation is child-to-parent.
`compose(parent, child)`, `inverse`, and `apply` require validated poses. Validation
requires finite translation bounded to1e12 metres per axis and a unit quaternion
within1e-8. This bound prevents unbounded values in scene transforms; it is not a
physical operating range for models.

`FixedFrames(root, edges)` copies up to4,096 `FixedFrame{parent, child, pose}` values,
validates IDs/poses/cycles/parent connectivity, normalizes accepted quaternion
rounding, and resolves every pose once. Declaration order is retained; parents need
not precede children. It does not retain references to caller-owned values.
`fromRoot(name)` returns the frame-to-root pose; `lookup(target, from)` returns the
from-to-target pose. Unknown names throw. The C++ tree is immutable; references
returned from it remain tied to its lifetime. Python returns detached copies.

## Robot content and sensor mounts

A native robot may declare `frames`. Its root denotes the simulated COM, regardless
of the chosen label. For example:

```yaml
frames:
  root: center
  transforms:
    - parent: center
      child: housing
      position_m: [0.1, 0.0, 0.02]
      orientation_wxyz: [1, 0, 0, 0]
    - parent: housing
      child: pressure_mount
      position_m: [0.0, 0.0, -0.1]
      orientation_wxyz: [1, 0, 0, 0]
```

Sensors choose either their existing explicit `mount` or `mount_frame: pressure_mount`.
The latter resolves into COM-relative model geometry during profile loading. Unknown
frames and both alternatives together are errors. `frame` remains the measurement's
label; integrations may qualify source-local names when publishing transports.
Named frames are also available as `Scenario::body_frames`, and Python
`scenario.body_frames`. Without `frames`, the tree contains only the `com` root.

This does not alter the physical state origin: initial poses and plant observations
still locate COM. Renderer assets can select CAD/base/mount frames explicitly;
placement policy must convert a requested frame pose into COM separately. Talos
frame equivalence is tested using pinned CAD COM/base offsets under a rotated body.
That check is not yet a delivered Talos visual assembly or sensor calibration.

## Viewer composition

`LivePoseOptions.fixed_frames` accepts source-owned static transforms alongside the
one live body pose. The complete source graph is validated at construction using
viewer history limits (at most128 total edges, hence127 static frames). Each immutable snapshot combines
these mounts with exactly the retained body history, including reset/reconnect.
Old snapshots retain their original transforms. No simulation types enter this API.

`robotics-sim-view` selects the scenario frame root as its live body frame and passes
its rigid mounts to the source. This application reserves `world` as its external
frame name; conflicting robot frame names are rejected. A standalone viewer source can supply the same
values from a live robot or other producer. Sensors without named frame declarations
do not implicitly create transforms from their transport labels.
