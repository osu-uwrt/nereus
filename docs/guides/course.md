# Pool and course layout

Two packs set up the world: a **pool** (`content/packs/pools/<id>/pool.yaml`) describes the water and walls, and
a **scenario** (`content/packs/scenarios/<id>/scenario.yaml`) picks the robot, pool, tasks and bridge and places
everything. To lay out a different competition course you usually only edit the scenario.

## Pool

```yaml
kind: pool
id: my_pool
description: 25 m practice pool.
type: rectangular_pool
parameters:
  length_m: 25.0
  width_m: 12.0
  depth_m: 2.0
  deck_height_m: 0.3            # deck above the water surface (drawn by the viewer)
  water_level_m: 0.0
  water_density_kg_m3: 998.2
  surface_pressure_pa: 101325
  current_m_s: [0, 0, 0]
  current_oscillation_amplitude_m_s: [0, 0, 0]
  current_oscillation_frequency_hz: 0.1
collision_boxes:                # pool-local: origin at a floor-plan corner on the surface, +x along length
- {id: floor, size_m: [25, 12, 1], center_m: [12.5, 6, -2.5], orientation_wxyz: [1, 0, 0, 0]}
- {id: wall_x0, size_m: [1, 12, 4], center_m: [-0.5, 6, -1], orientation_wxyz: [1, 0, 0, 0]}
# ...one box per wall
water_optics:                   # what the cameras and viewer see
  tint_rgb: [0.025, 0.22, 0.29]
  absorption_per_m_rgb: [0.648, 0.145, 0.025]
  scattering: 0.458
  distance_scale: 1.88
  distance_power: 0.4
  clear_distance_m: 0.0
lighting:
  profile: outdoor              # or indoor
  direct_light: 1.0
  ambient_light: 0.8
  sun_azimuth_deg: 225.0
  sun_elevation_deg: 55.0
  glare: 0.5
```

The robot collides with `collision_boxes`; the viewer draws the floor, walls, deck and coping from `parameters`.

## Scenario

```yaml
kind: scenario
id: my_practice
description: My AUV in the practice pool.
robot: ../../robots/my_auv           # folders, relative to this file
pool: ../../pools/my_pool
tasks: ../../tasks/robosub_2026
bridge: ../../bridges/my_stack        # optional: only the ROS simulator needs it
seed: 7                               # all sensor noise is reproducible from this
timestep_s: 0.002                     # physics step
sensor_noise: true
world_frame: map

pool_placement: {position_m: [0.0, 12.0, 0.0], yaw_deg: -90.0}   # pool-local frame in the world
task_placements:                                                 # each task's local frame in the world
- {task: gate, position_m: [4.8, -2.2, -0.75], yaw_deg: 170.6}
- {task: slalom, position_m: [8.3, 0.56, -1.3], yaw_deg: -152.7}
# ...one entry per task in the task pack

contacts: {model: box_scene, restitution: 0.1, friction: 0.4}
initial:                                                         # start pose of the robot
  frame: com
  position_m: [0, 0, -1]
  orientation_wxyz: [1, 0, 0, 0]
  linear_velocity_m_s: [0, 0, 0]
  angular_velocity_rad_s: [0, 0, 0]
run:
  auto_start: false                  # wait for "Start run" in the viewer
  options: {role: repair, heading_coin: true, role_coin: true}   # defaults of the tasks' run_options
```

Every task in the task pack needs exactly one placement; to run only some tasks, use a task pack whose
`tasks:` lists just those files. Positions are in the world frame (`map`); a task's local frame (its origin and
+x, see [tasks](tasks.md)) is placed with `position_m` and `yaw_deg`.

Tip: match your stack's mapping priors. The Talos scenario places each task exactly where
`riptide_mapping`'s `init_data` puts the corresponding `*_frame`, so the simulated course and the robot's map
agree before any detection.

## Try it

```sh
PYTHONPATH=python/src python3 -m nereus.packs validate content/packs/scenarios/my_practice
ros2 launch integrations/uwrt/launch/sim.launch.py scenario:=$PWD/content/packs/scenarios/my_practice
```
