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
- Nereus logo (a trident rising out of sonar pings): `docs/images/logo.svg` and the viewer's window icon
  (`content/viewer/icons`, regenerated by `make_icons.py`).
- Viewer design pass (from an Impeccable critique): map editing's 2D view drawn as a chart in the theme's colours
  (prop footprints and badges in each prop's colour, scale bar, labelled origin); every theme keeps secondary,
  status and accent text and lit-button labels at 5:1 or better; pool-view overlays and the course map follow the
  theme; menus use the interface scale and fit the window; Motion keeps its actions in view on a short panel;
  toolbar groups with dividers (`type: separator`) and labelled dropdowns; one wording for "waiting for pose" and
  for preview mode; the pool chip shows the pool's name. Polish: map Reload asks before dropping unsaved edits and a
  save says it was checked; editable x / y / z / yaw cells in the map's object table; the inspector names the stored
  pose and the resulting map position; empty preview states say what a panel holds; Calibrate tag; Pose gizmo
  toggle explained; a scannable Help; themed Vision legend; visible inactive tabs; a real Fit pool button.
- Viewer theme **Heat sheet** (the new default) and its night twin **Timing board**, one world from a swim meet's results
  sheets and the pool timing board:
  - White (by day) or charcoal (by night) sheets with ruled heads and hairline rows.
  - An ink or black board for the menu and command bars, carrying the run clock and score as large figures.
  - Barlow TF fonts with tabular figures, shipped in `content/viewer/fonts`.
  - Lane blue for selection, amber for the running clock, and touch red for KILL.
  - Shared changes in every theme:
    - Themes can carry their own board palette, spacing, fonts and ruled heads.
    - Undocked windows get edges and shadows.
    - Layouts widen with the interface scale.
    - Camera windows are titled "Forward camera" and "Downward camera".
    - Enable is go green in every theme.
    - No pool chip over the 3D view.
    - The camera status band is drawn in the board's colours.
    - Motion has **Zero roll & pitch** beside *Pose gizmo*.
    - Mapping's tag calibration reads **Calibrate**, with a progress bar.
    - The title bar carries the Nereus mark (trident and sonar), is sized like VS Code's, and has a hairline over the
      command bar. The command bar leads with the robot and pool.
    - The interface scale follows the desktop by default, and the window opens at its configured size in the
      desktop's units.
    - Camera images carry no overlay: Truth / ROS sits beside RGB / Depth, and the live rate is in the header.
    - The course map draws the props and the robot as top-down images baked once from their meshes (no live
      rendering), with the task names beside them and a halo that keeps thin props visible zoomed out. The bins
      and octagon labels show again: the host config now uses the 2026 pack's names (`bins`, `surface`).
    - Themes are data: one YAML file each in `content/viewer/themes/` (the code keeps only the legibility floor
      and a built-in fallback).
    - Board telemetry (FOG, CPU, batteries) shows as readouts (dot, label, bold value) instead of boxed chips.
    - A command centre in the title bar (Ctrl+P), as VS Code's: fuzzy search over windows, layouts, themes, scales,
      focus targets, camera views, toggles and pools (Enable and KILL are left out on purpose).
    - Theme changes apply between frames (picking one could leave a mix of two themes), tab styles are part of
      theme files, Classic draws switches and status panels with Qt's bevels, robot-frame map origins drag along
      their X / Y arrows, and camera cards no longer jitter at a width right at the edge.
    - Two-way choices are sliding switches (RGB / Depth, Truth / ROS, Position / Feedforward, 3D / 2D, AprilTag /
      Robot frame).
    - The Motion table shows plain figures (no highlight or bold), and "Dive in place" no longer jumps rows while
      the panel is resized.
- Viewer design pass, round 2: a type ramp (bold section titles and table headers, run time and score as large
  figures); status chips are outlined capsules with a status dot, no longer shaped like buttons; input boxes,
  checkboxes and buttons stand out from their panel in every theme; table headers on a neutral band; numeric
  columns right-aligned (Motion, map objects); "not connected" said once, in the command bar, with panels saying
  what they show once connected; the course map turns to fill a tall window and its labels no longer overprint;
  map-chart labels in bold with their prop's colour as an edge, placed clear of each other and the view's chips;
  camera status on a band across the image's foot; 3D landmark plates fit their names.
- Viewer map editing (Edit map in the command bar, `Ctrl+M`; the bar turns the accent colour with Save / Done): the
  Dead Reckoning tool in 3D, with its own layout (Map objects, Inspector), a 2D top-down view (default) for quick
  x / y moves, the simulator paused while editing, and a robot-frame origin drawn and dragged as the robot. The robot's riptide_mapping
  `config.yaml` laid out in the pool view as RViz meshes; select, drag, turn, nudge or type poses, re-parent, swap
  sides or classes, add, duplicate, rename and delete props; place the map origin on an AprilTag line / wall spot or
  as a free robot frame; undo / redo; and a save that rewrites only the changed values (comments and the rest of
  the file untouched). `prior_map.config` in the host config, `--prior-map FILE`, `--workspace map`.
- Viewer View > Pool: switch between the scenario packs' pools. In sim, `sim.launch.py` now runs nereus-sim under
  `sim_supervisor.py`, which restarts it in the chosen pool (`<ns>/simulator/load_scenario`, status on
  `<ns>/simulator/supervisor`; records go to `<output>-2`, ...) while the robot stack keeps running; otherwise the
  viewer resolves the pack itself and reloads. The Windows menu is now generated from the window list.
