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

### Sloped floor

A floor that changes depth along the pool (a dive well, a shallow end) is a `floor_profile` in `parameters`:
depth below the water at points along one axis, constant across the other, joined by a monotone cubic so flat
stretches stay flat and a curve never overshoots its end depths.

```yaml
parameters:
  length_m: 25.0
  depth_m: 5.1816               # the deepest point of the profile
  floor_profile:
    along: x                    # or y
    points_m: [[0, 5.1816], [9.3, 5.1816], [15.3, 4.2672], [25, 4.2672]]   # [position, depth], 0..length
  # ...
collision_boxes:                # walls only: the floor's contact boxes are generated from the profile
- ...
```

A floor that also rises toward a side wall takes a list of profiles; at each point the floor is the shallowest
of them, so a deep flat area can slope up toward several walls (here toward the shallow end and the far side):

```yaml
  floor_profile:
  - {along: x, points_m: [[0, 5.18], [16, 5.18], [25, 4.27]]}
  - {along: y, points_m: [[0, 5.18], [14, 5.18], [17, 4.6]]}
```

The same floor is used by the viewer and cameras (the floor mesh, lines draped over it, walls meeting it), by
the DVL's range to the bottom, by vehicle and prop contacts, and by the task runtime's floor checks. Validation
rejects a profile that does not span the pool, a `depth_m` that is not the floor's deepest point, a flat floor box
alongside it, and `sphere_pool` contacts (use `box_scene`).

### Lane lines and finish

Painted lines are data, not part of the renderer: a pool without `markings` has a plain tiled floor. Everything
is in the pool frame, in metres. Floor stripes go `from`/`to` `[x, y]`; wall stripes go `from`/`to`
`[coordinate along the wall, z relative to the water surface]` on wall `x_min`, `x_max`, `y_min` or `y_max`.
The same stripes appear in the viewer, the simulated cameras and the course map.

```yaml
markings:
  color_rgb: [0.093, 0.14, 0.16]   # defaults for every stripe below
  width_m: 0.254
  lane_grid:                       # optional: evenly spaced lines
    along_x: {count: 8, spacing_m: 2.7432}               # parallel to +x, centred across the width
    along_y: {count: 17, spacing_m: 2.7432, first_m: 3}  # parallel to +y, first line at x = 3
    inset_m: 2.0                   # stop short of the end walls
    ends: t                        # a bar across both ends (t_length_m, default 1 m)
    on_walls: true                 # continue each line up both end walls to the surface
  lines:                           # any other floor stripe, at any angle
  - {from: [4, 2], to: [12, 5], ends: [t, none], width_m: 0.3, color_rgb: [0.6, 0.1, 0.1]}
  wall_lines:
  - {wall: y_max, from: [10, -1.8], to: [10, -0.6], ends: t}
surface:                           # optional; these are the defaults
  tile_rgb: [0.68, 0.85, 0.87]
  tile_size_m: 0.1524              # grout pitch, 0 for a plain liner
  waterline_rgb: [0.065, 0.2, 0.27]
  waterline_band_m: [-0.13, 0.04]  # wall band around the waterline; [0, 0] for none
```

Validation rejects stripes that leave the surface they are painted on (T bars may overhang).

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
