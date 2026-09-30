# Adding a robot

A robot is one folder, `content/packs/robots/<id>/`, with `robot.yaml` and its mesh assets. Everything the
simulator needs about the vehicle is in that file; your ROS topics are wired separately in a
[bridge](bridge.md). Start by copying `robots/talos/` and editing it.

Units are SI, angles in radians unless the key says `_deg`, orientations are `[w, x, y, z]` quaternions, frames
follow ROS (x forward, y left, z up).

## Skeleton

```yaml
schema_version: 1
kind: robot
id: my_auv
description: My AUV.
reference_frame: base_link      # the frame reported as "the robot pose" (TF, tasks, placement)

assets:                         # every file the pack uses
- {id: body_mesh, path: assets/body.glb, source: "CAD export 2026-09", required_from_step: 3, status: present}

body: {...}                     # mass, inertia, hydrodynamics
frames: {...}                   # rigid frame tree rooted at the centre of mass
collision_boxes: [...]
thrusters: [...]
safety: {...}
sensors: [...]
mechanisms: []                  # may be empty
scoring_envelope: {collision_boxes: [hull], points: []}
visuals: [...]                  # optional: meshes for the viewer and cameras
```

Point a [scenario](course.md) at the folder and run `python3 -m nereus.packs validate
<scenario>`; errors name the file and key.

## Physics: `body`

```yaml
body:
  type: marine_6dof
  parameters:
    mass_kg: 32.0
    inertia_matrix: [[1.63, 0, 0], [0, 0.73, 0], [0, 0, 1.63]]   # about the COM, body axes
    added_mass_matrix: [[33.8, 0, 0, 0, 0, 0], ...]             # 6x6
    linear_damping_matrix: [[34.9, 0, 0, 0, 0, 0], ...]         # 6x6
    quadratic_damping: [128.9, 96.2, 50.3, 2.8, 1.5, 5.1]       # per axis
    damping_center_m: [0, 0, 0]
    displaced_volume_m3: 0.0324                                  # buoyancy = density * g * volume
    buoyancy_center_m: [0.008, 0, 0.01]                          # relative to the COM
    buoyancy_radii_m: [0.175, 0.415, 0.275]                      # ellipsoid used for partial submersion
    command_timeout_s: 0.5                                       # thrusters coast to zero without commands
```

The same Fossen model is used by the MPC controller, so these numbers matter.

## Frames

```yaml
frames:
  root: com                      # the physics body; everything else hangs off it
  transforms:
  - {parent: com, child: cad, position_m: [0.157, -0.04, 0.048], orientation_wxyz: [1, 0, 0, 0]}
  - {parent: cad, child: base_link, position_m: [-0.14, 0.03, -0.09], orientation_wxyz: [1, 0, 0, 0]}
  - {parent: cad, child: imu_mount, position_m: [...], orientation_wxyz: [...]}
  - {parent: cad, child: ffc_mount, ...}
```

Name a frame for every sensor, mechanism, visual and camera optical frame. How these names appear in ROS TF is
set by the bridge's `frame_names`.

## Thrusters

List them in actuator order (the bridge maps your ROS array order onto it). Positions and directions are
relative to the COM.

```yaml
thrusters:
- id: VUS
  type: lagged_force
  position_m: [0.023, -0.409, 0.29]
  direction: [1, 0, 0]               # unit thrust axis
  parameters: {delay_s: 0.1, rise_time_s: 0.08, fall_time_s: 0.06, slew_rate_n_s: 300,
               forward_limit_n: 28, reverse_limit_n: 28, deadband_n: 0, forward_scale: 1, reverse_scale: 1,
               efficiency: 1, propeller_radius_m: 0.05}
```

## Safety

```yaml
safety:
  initially_killed: true
  kill_stops_thrusters: true
  commands_while_killed: zero_force        # or rejected
  kill_disarms_mechanisms: true
  arming: {initially_armed: false, arm_rejected_while_killed: true, applies_to: [claw, dropper]}
```

## Sensors

Every sensor has `id`, `type`, `frame` (where the measurement is expressed), `mount_frame`, `period_ns` (a
multiple of the scenario timestep, 2 ms by default), optional `latency_ns`, and `parameters`.

| Type | Measures | Key parameters |
| --- | --- | --- |
| `ahrs` | IMU + attitude (VectorNav-style) | `inertial.acceleration_noise`, `inertial.gyro_noise`, `attitude.angle_stddev_rad` |
| `imu`, `attitude` | the two halves separately | as above |
| `fog` | single-axis rate gyro | `axes`, `gyro_noise` |
| `reference_velocity` | DVL velocity (as the UWRT DVL driver reports it) | `velocity_noise`, `reported_variance` |
| `reference_altitude` | depth (as the UWRT depth driver reports it) | `target_position_body_m`, `noise` |
| `dvl`, `pressure` | raw bottom-track DVL / pressure sensor models | see `libraries/sensors` |
| `stereo_camera` | RGB, depth, point cloud | `resolution_px`, `intrinsics_left/right`, `baseline_m`, `right_frame`, `outputs` |

```yaml
- id: ffc
  type: stereo_camera
  frame: ffc_left_optical
  mount_frame: ffc_mount
  period_ns: 66666667                       # 15 Hz
  parameters:
    baseline_m: 0.05
    right_frame: ffc_right_optical
    resolution_px: [1920, 1200]
    intrinsics_left: {fx: 1864, fy: 1864, cx: 956, cy: 585}
    intrinsics_right: {fx: 1864, fy: 1864, cx: 956, cy: 585}
    outputs: [rgb_left, depth_left, camera_info, point_cloud]
```

Cameras render only while something subscribes to them. Set `enabled: false` on a sensor to leave it out.

## Mechanisms

| Type | What it does |
| --- | --- |
| `launcher` | fires projectiles from `slots` (torpedoes) |
| `dropper` | releases projectiles downward (markers) |
| `claw` | two jaws that close on props; grasps are simulated with contacts (`pads` name convex collision meshes) |
| `magnet` | a tip point the course can react to (magnet targets) |

Each has `id`, `type`, `frame` and `parameters`; copy a Talos entry for the full parameter set. Tasks can require
a mechanism type (`tasks.yaml` `requires`).

## Visuals

```yaml
visuals:
- {asset: body_mesh, frame: cad, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
```

`.glb`, `.dae` and `.obj` load; textures are PNG next to the mesh. The viewer's rotor animation and status
lights are set in `content/viewer/*_thruster_visuals.yaml` and `*_status_lights.yaml`.

## Collisions and scoring geometry

```yaml
collision_boxes:                 # COM-relative boxes that hit the pool walls and course
- {id: hull, size_m: [0.35, 0.83, 0.55], center_m: [0, 0, 0.07], orientation_wxyz: [1, 0, 0, 0]}
scoring_envelope:                # what tasks test for "passed through the gate", "surfaced", ...
  collision_boxes: [hull]
  points:
  - {id: magnet_tip, frame: magnet_mount, position_m: [0, 0, 0.05]}
```
