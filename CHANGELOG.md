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
- Viewer window layout: every panel, robot camera, the course map and the tool windows (Scene settings, Display,
  TF frames, Simulation, run scorecard) is a dockable window: drag tabs to rearrange, split, tab or float them.
  Built-in layouts (Standard, Wide view, Camera wall), named saved layouts, Lock layout, `Ctrl+Space` to
  maximize the pool view, and the last arrangement restored at start (`~/.config/nereus/`, `--layout NAME`).
  Enable / KILL moved to an always-visible command bar; menu bar with View, Windows, Layout and Help. Panels
  take `dock:` in the composition. Dear ImGui is now its docking release (v1.91.9b-docking).
- Viewer title bar, toolbar and themes: the menus live in the viewer's own title bar (drag to move, double-click to
  maximize, edges resize; `--system-title-bar` for the desktop's). Any window can be pinned to the pool view
  toolbar (right-click its tab, or the toolbar's +) and any toolbar button hidden; saved with the layout. Themes:
  Abyss (default), Midnight, Daylight, High contrast (View > Theme, remembered; `--theme`). Enable / KILL is also
  at the top of the Motion panel; tab close boxes show on hover.
- Viewer side panels snap shut again: drag the border nearly to the window edge (or View > Left / Right panels,
  `Ctrl+[` / `Ctrl+]`); drag the pool view's edge back out to show them. Theme is a View submenu.
- Viewer: right-click any button, checkbox or dropdown in a panel to pin a working copy to the toolbar (saved with
  the layout). The title bar follows the desktop's display scale like other applications'; View > Interface scale
  (100-200 %, Match desktop; `--ui-scale`) sizes the rest. Five more themes: Ocean, Arctic, Ember, Sonar, Paper.
- Viewer theme Classic: Qt's Windows-style grey with bevelled controls and the Ubuntu font, like the desktop's Qt
  tools (RViz).
