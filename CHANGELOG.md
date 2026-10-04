# Changelog

## 0.1.0 — unreleased

First version for the team.

- `nereus-sim`: ROS 2 simulator driven by a resolved scenario. 6-DOF marine dynamics with delayed,
  rate-limited thrusters; IMU/FOG/DVL/depth sensors; stereo cameras with depth and point clouds;
  launcher, dropper, claw and magnet mechanisms; Bullet prop physics for the table task; task
  observers and compiled competition scoring (RoboSub 2026).
- `nereus-viewer`: pool viewer and operator panels (motion, autonomy, mapping, vision, run
  scorecard, camera cards, point clouds). Drives either the simulator or the real robot.
- Packs: robot, pool, tasks, bridge and scenario YAML with JSON-schema and semantic validation
  (`python -m nereus.packs validate|resolve`). The Talos robot, RoboSub 2026 course and UWRT
  bridge ship as packs.
- UWRT launch files for the simulator and for the real robot.
- Pool packs describe their own lane lines and finish (`markings`, `surface`): floor and wall stripes
  at any position, angle, width and colour, with optional T ends and an evenly spaced `lane_grid`
  shorthand. The renderer no longer assumes the RoboSub layout, and the course map draws the lines.
- Sloped pool floors: `floor_profile` in a pool's parameters (depth at points along one axis, joined by
  a monotone cubic; a list of profiles takes the shallowest at each point). Rendering, cameras, the DVL, vehicle and prop contacts and task floor checks all
  follow it; flat pools behave exactly as before.
- Pool `fixtures`: meshes from the pool's assets, solid boxes (optionally sloped-sided or colliding),
  and recesses cut into the walls and deck (stair wells), drawn by the viewer and cameras.
- `lane_grid.targets`: a plus or T target on both end walls above every lane line.
- `rpac_divewell` pool pack: Ohio State's RPAC dive well (25 m x 56 ft, 17 ft deep), with its eight
  lap lines, staggered cross lines, diving lines and end-wall targets.
- `talos_uwrt_rpac` scenario: the RoboSub 2026 course at its prior-map poses, set in the RPAC dive
  well (`ros2 launch ... sim.launch.py scenario:=content/packs/scenarios/talos_uwrt_rpac`).
- Robot packs can be measured from any datum: `frames.body` names the centre-of-mass frame in a tree
  rooted elsewhere, thrusters can sit at a `frame` (thrust along its +X), collision boxes can name
  the `frame` they are measured in and the altitude sensor can report a `target_frame`. Resolving re-roots the tree at the body, so the runtime is unchanged. Talos is
  now measured from `origin` (the camera-cage screw `talos.yaml` uses, formerly the `cad` frame), with
  `com` as one of its frames.
- Equipment packs: a team's own gear (`content/packs/equipment/<id>`): meshes with optional named frames,
  selected by a scenario with `equipment:` and placed with `equipment_placements`. The viewer and the simulated
  cameras draw them. UWRT's AprilTag calibration board (now a 3 mm dibond sign with a `tag` frame in
  apriltag_ros's axes) moves there from the viewer config, so the simulator's published camera images contain
  the tag.
- Scenario placements can be relative to each other: `relative_to` names `world`, `pool`, a task, an equipment
  placement or an item frame; `rpy_deg` gives full orientations; `world_placement` places the world itself and
  makes the pool the root. Resolving writes everything in world coordinates. The RPAC scenario hangs the board on
  the deep-end wall over lap line 3 and places the map from the tag as the UWRT stack does, so the course moves
  with the board; the robot starts 2 m out from it.
- Scenario `thruster_overrides`: retune the robot's thrusters per scenario without editing the robot pack
  (applied when resolving, so the resolved robot carries the result).
- Talos sensors from the 2026-10-03 pool bags: IMU at 400 Hz with VN-100-like (nearly noise-free) attitude,
  gyro/FOG/DVL/depth noise from the bags.
