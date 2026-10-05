# Using the viewer

`nereus-viewer` is the operator interface for both the simulator and the real robot. The sim launch
starts it for you; for the real robot use `ros2 launch integrations/uwrt/launch/robot.launch.py`.

![Viewer with the robot in the pool](images/robot.jpg)

- **Title bar**: the viewer draws its own, with the menus in it: **View** (camera, focus, overlays, theme),
  **Windows** (show or hide any window), **Layout** (built-in and saved layouts) and **Help** (controls and
  shortcuts, `F1`). Drag its empty part to move the window, double-click it to maximize, drag the window's edges
  to resize; minimize, maximize and close are at its right end. `--system-title-bar` (or `window.title_bar:
  system` in the host config) uses the desktop's title bar instead.
- **Command bar**, always visible: **Enable / KILL** and the robot's state, then on the right the FOG and CPU
  temperatures and both batteries' charge (see below), a red `REC` chip per camera while it records, and the
  pill that says where the robot pose comes from (`PHYSICS CONNECTED` in sim, `ROBOT (ESTIMATE)` on the real
  robot).
- **Pool view**, in the middle: the 3D scene with its toolbar (camera, focus and display toggles; lit buttons are
  on). The toolbar is yours to arrange, see [The toolbar](#the-toolbar).
- **Windows** around it: the robot panels (Motion, Autonomy, Mapping, Vision, Recording, Electrical, Actuators),
  one window per robot camera, and the course map. Every one of them can be moved, see
  [Arranging windows](#arranging-windows).

## Moving around

| | |
| --- | --- |
| Left drag | orbit |
| Right / middle drag | pan (detaches Follow) |
| Scroll | zoom |
| `F` | focus on the point under the cursor |
| `F3` | frame-time stats |
| `F1` | controls and shortcuts |
| `Ctrl+Space` | maximize the pool view (close every other window); again to restore them |
| `Ctrl+Shift+1` / `2` / `3` | Standard / Wide view / Camera wall layout |
| `Ctrl+[` / `Ctrl+]` | snap the left / right panels shut, or bring them back |
| View → *Free camera* | click for mouse look, `WASD` move, `Space`/`Shift` up/down, `Ctrl` fast, `Esc` release |
| View → *ffc* / *dfc*, or **Main view** on a camera window | look through a robot camera |

The camera and focus dropdowns lead the pool view's toolbar. **Focus** jumps to the vehicle, a mechanism (Claw,
Payloads) or a course element; **Follow** keeps the camera on the robot. **Labels** names course elements; **TF**
draws frame axes (its arrow opens the TF frames window: names, axis length, the frame tree); **MPC path** draws the
MPC's predicted path; **Thrust** draws the commanded thruster forces (`thruster_forces`) as red arrows from each
thruster along its axis, like RViz's thruster wrenches (0.05 m per newton; nothing while the controller is not
publishing). The same toggles are in the **View** menu.

## Switching pools

**View → Pool** lists every scenario pack in `content/packs/scenarios` by its pool (RoboSub 2026, the RPAC dive
well); the current one is ticked. The course keeps its map coordinates; the pool, the calibration board and the
start pose change with the pack.

- **Simulator** (started with `sim.launch.py`): the simulator restarts in the chosen pool. The run starts over in a
  new record folder (`<output>-2`, `-3`, ...), the robot jumps to the new start pose, the robot stack keeps running
  and ROS time keeps going forward. The view reloads when the new simulator publishes its scenario. A simulator
  started some other way can't be switched (the menu says so).
- **Real robot or a preview** (the viewer was given `--scenario`): only this view reloads in the chosen pool; the
  robot is not affected.

The prior map editor keeps its map origin per pool. If the new pool's water differs (density, level, current), a
note warns that an MPC model generated for the first pool is stale.

## Arranging windows

Every panel, camera, the course map and the tool windows (Scene settings, Display, TF frames, Simulation, the
run scorecard) is a window you can place:

- **Move** a window by dragging its tab. Drop it on another window's centre arrow to add it as a tab there, on an
  edge arrow to split that space, or anywhere else to float it. Drag the borders between windows to resize.
- **Close** a window with the x on its tab; reopen it from **Windows**. Closed windows give their space to the
  others and come back where they were.
- **Layout** → *Standard* (panels left, cameras and map right), *Wide view* (cameras and map in a strip under the
  pool view) or *Camera wall* (large camera feeds) rebuilds the arrangement and reopens the default windows.
- **Layout** → *Save current as* keeps the arrangement (positions and which windows are open) under a name;
  *Load* and *Delete* manage them. Saved layouts are files in `~/.config/nereus/layouts/`, so they can be copied
  to another machine.
- **Layout** → *Lock layout* stops windows being undocked, docked or resized by accident during a run.
- `Ctrl+Space` (or View → *Maximize pool view*) hides everything but the pool view; press it again to get the
  same layout back.
- **Snap a side shut**: drag the border between the left (or right) panels and the pool view toward the window
  edge; past a point the panels snap shut while you are still dragging, and dragging back out reopens them. Once
  closed, drag the pool view's edge out again (it shows a line when the pointer is over it) to bring them back
  at that width. View → *Left panels* / *Right panels* and `Ctrl+[` / `Ctrl+]` do the same.

A tab's close box shows while the pointer is over the tab.

## The toolbar

Right-click the pool view's toolbar, or click its **+**, to choose what it shows:

- **Toolbar buttons**: untick any built-in button (Run tracking, Scene, ...) to hide it.
- **Pinned controls**: right-click any button, checkbox or dropdown in a panel or tool window (Fire torpedo, Pause,
  Water, Lighting, ...) and choose **Pin to toolbar**: a working copy appears in the toolbar, showing the control's
  current state, and it keeps working with its window closed. Right-click the copy to unpin it.
- **Pinned windows**: tick a window to give it a toolbar button. You can also right-click a window's tab and choose
  **Pin to toolbar**. A pinned window's button is lit while the window is in front; clicking it shows the window,
  brings it in front of the other tabs, or hides it when it is already in front.
- **Reset toolbar** returns to the configured buttons with nothing pinned.

The toolbar is saved with the layout (and with named layouts); the built-in layouts leave it as it is.

## Themes

View → *Theme* ▸ **Abyss** (dark teal, the default), **Midnight** (neutral dark with a blue accent), **Daylight**
(light, for a bright room or a sunny pool deck), **High contrast** (black, white and yellow), **Ocean** (navy and
blue), **Arctic** (slate and frost blue), **Ember** (charcoal and orange), **Sonar** (green on black), **Paper**
(warm light) and **Classic** (Qt's Windows-style grey with bevels and the Ubuntu font, like RViz on this desktop).
The choice is remembered (`~/.config/nereus/viewer.yaml`); `--theme NAME` or the host config's `theme:` set it too. The pool view itself looks the same in every theme.

## Interface scale

The title bar follows the desktop's display scale (GNOME at 200 %: twice the size), like other applications'
title bars. Everything else is at the viewer's own 100 % unless you choose View → *Interface scale* ▸ 100–200 % or
*Match desktop*; the choice is remembered, and `--ui-scale 1.5` / `--ui-scale auto` or the host config's
`interface_scale:` set it too.

The viewer saves your arrangement when it closes (`~/.config/nereus/viewer_layout.ini`) and starts with it next
time. `--layout NAME` starts with a built-in layout (`standard`, `wide`, `cameras`), a saved layout's name or an
`.ini` file instead; with no saved arrangement the host config's `layout:` is used.

## Robot temperatures and batteries

The header chips show the FOG temperature (`gyro/status`) and the hottest CPU core (the computer monitor's
`Core Temperature` in `/diagnostics_agg`): cyan is fine, amber a warning (FOG 55 °C, CPU 70 °C), red an error (FOG
75 °C, CPU 85 °C, or a fault the FOG driver or the computer monitor reports). Grey means no data: never received,
or nothing for 2 s (FOG) / 10 s (CPU), when the last value stays shown. Hover a chip for the details.

`PORT` and `STBD` are each battery's state of charge (`state/battery`), as in the RViz overlay: amber below 50 %,
red below 20 %, grey after 10 s without a reading. Hover for the pack voltage, current, time to discharge and
cell. In sim none of these publishers run, so all four chips stay grey.

## Driving the robot (Motion)

1. **Enable** (top of the Motion panel, or the command bar). The enable latches: the robot keeps running if the viewer closes or the tether drops. **KILL** (same button, or
   the physical kill) stops it.
2. Pick **Position** or **Feedforward**.
3. Drag the gizmo in the 3D view (arrows move, rings rotate; `Esc` during a drag restores the start), or type a
   target in the table and press **Command**. **Current** copies the robot's pose into the table; **Dive in
   place** targets the current x/y and heading, level, at the configured depth (`dive_z`).

If the viewer freezes or the pose goes stale, it releases manual control and the robot holds its last command;
it never kills on its own. Another operator sending on the same kill switch (for example RViz's control panel)
does kill: use one operator at a time.

## Autonomy, Mapping, Actuators

- **Autonomy**: pick a tree, **Start** / **Stop**; the execution stack shows the running nodes. Manual control is
  blocked while a tree owns the robot.
- **Mapping**: tag calibration, reset, and the mapping target (**Lock map**).
- **Actuators**: arm/disarm, fire torpedoes, drop markers, open/close the claw, reload.

## Recording

- **SVO recording**: the path is on the robot (`~` is `/home/ros`). Each camera records to its own file,
  `<path>_<date>_<time>_<camera>.svo2` (untick *Add date and time* to drop the timestamp). **Start** calls the ZED
  node's `start_svo_rec` (lossless); **Stop** calls `stop_svo_rec`. The viewer knows only the recordings it
  started, so **Stop** works whenever the service is up, including for a recording started before a restart.
  Starting needs `zed_msgs` when the viewer is built; without it the panel says so and only Stop works.
- **Capture image**: the picture taker (`capture_image`) saves the newest front camera frame on the robot; the
  reply lists the saved files.

## Electrical

The RViz electrical panel's tools, on the real robot:

- **Power**: one button per `command/electrical` command. The red ones (cycle computer, cycle robot, kill robot
  power) ask for confirmation first.
- **IMU (VectorNav)**: **Mag cal** runs the driver's calibration (turn the robot slowly through every orientation;
  the bar fills as the deviation shrinks), **Cancel** stops it. Register edit: type a register number, **Read**
  fills the value, **Write** sends the value, **Save to flash** keeps the settings across power cycles.
- **FOG**: **Tare gyro** with the sample count and timeout; hold the robot still. An aborted tare shows why.
- **Pinger**: the enable state is re-sent every second, as RViz did, so a rebooted board picks it up. The
  frequency buttons choose the broker's frequency; the highlighted one is what the board reports.
- **IVC**: send a status (first two headers) or a raw 0-31 command (other headers); the log shows sends, receives
  and acknowledgements with their decoded names.

Topics, the command list and the IVC names are in the `electrical` provider of `talos_uwrt_panels.yaml`.

## Vision

- **Detections**: the detector's markers, placed where the object was seen. *Placement* (sim only) chooses
  simulator truth, the localization estimate, or both. *Keep detections* lets markers live out their lifetime
  instead of clearing each frame.
- **Point clouds**: one checkbox per cloud in the viewer config (YOLO detections, front and down camera); a cloud
  is only subscribed while its box is ticked. Newest message only, like RViz.

## Camera windows

**RGB / Depth** switches the image; **Main view** shows that camera in the pool view (click again for the orbit
camera). The image keeps its aspect at any window size. In sim the label under the image is a button: **truth
pose** shows the viewer's own render from the true camera pose (fast, no noise); click it for **ROS (stack)**, the
images your stack actually receives.

## Course map

Scroll zooms, drag pans, clicking a task focuses the pool view on it; **Fit pool** resets the view. Make the window
bigger (or float it) for full-size labels.

## Display window

The toolbar's **Display** button (or Windows → Display) opens it; it stays open while you change things.

- **Water, Pool walls & deck, Pool floor, Surface reflections**: visibility (camera cards always see the pool).
  **Pool tiles** off draws the walls and floor plain, keeping the lane lines (this view only).
- **AprilTag board**: the scenario's equipment pack (the calibration board) in this view only; camera cards always
  show it. `equipment_visible` in the host config sets it at startup.
- **Course**: *Auto* shows the simulator's course in sim and the mapping estimate on the real robot; *Mapping
  (RViz)* draws `riptide_meshes` models at the mapping TF frames, exactly as RViz does. *Mapping ghost* (sim)
  overlays the mapping estimate translucent on the true course. *Meshes* lists every mapping mesh by its TF frame:
  untick one to hide it (course and ghost alike); `mapping_markers.hidden` in the host config sets which start
  hidden.
- **Localization estimate** (sim only): *Robot ghost* draws the EKF estimate as a translucent robot. *Control
  gizmo* and *Follow* each centre on the estimate or the true robot. Commands always go to the controller in its
  estimate frame.
- **Lighting**: the observer's lighting, independent of what the cameras see.

**Scene** (the Scene settings window) has the simulator's mechanism buttons, lighting and water appearance;
**Simulation** sets the speed (pause, 1x, faster), **Sync sim** to the estimate and **Reset sim**. Both are
windows: dock them if you use them often.

## Editing the prior map (Dead Reckoning in 3D)

**Edit map** in the command bar (`Ctrl+M`) edits riptide_mapping's `config/config.yaml` in the pool, as the Dead
Reckoning tool does. While editing, the command bar turns the accent colour and reads *EDITING PRIOR MAP ·
config.yaml* (with *unsaved* when there are changes), with **Save** and **Done** at its right end; Enable / KILL stay
where they are. Editing has its own window layout: **Map objects** on the left (the file, Open / Reload / Save, Undo
/ Redo, *+ Add* and the object table: the tree with lock / hide and, toggled by **x y z**, each prop's x / y / z / yaw
relative to its parent as the file stores it), the **Inspector** on the right (the selected prop, or the map origin when
nothing is selected) and the pool view between. **Done** goes back to operating with the layout as it was.

The map is edited against a still scene: a running simulator is paused (the bar says *simulator paused*; Done
resumes it), Follow is off until Done, and the robot is drawn level (its position and heading only). Each prop is drawn as its RViz mesh at its pose under the map origin; a prop
with no mesh gets a box, unless it is a frame on a meshed assembly (the bin's targets, the torpedo's holes: their
labels show them).

The pool view's toolbar holds the map tools: **2D / 3D**, *Fit* (2D), **Display**, *Place origin*, labels (none /
roots / all), *Hide sim course* and the pool. **Display** is map editing's own look, separate from Operate's: water,
walls & deck, floor, *Pool tiles* (off: plain walls and floor, the lane lines stay), reflections, the AprilTag board,
and the lighting (preset, shadows, exposure, ambient). It starts
with the water off and Sterile lighting (even light, no shadows or caustics) so the floor and its lines read plainly.

- **2D** (the default) looks straight down at the pool (the same render, without perspective), for quick moves
  across a flat floor: drag a prop to move it in x / y (its height stays), the yellow ring turns it. Drag the floor
  (or right / middle drag) to pan, scroll to zoom about the pointer, `F` or *Fit* for the whole pool, double-click a
  prop (or its row in the tree) to centre on it.
- **3D** orbits; the selected prop has red / green / blue arrows for the map's X / Y and height, and the ring.

A click picks what is under the pointer: a prop's mesh (its bounds), its label or its origin dot.
- **Select** a prop by clicking it in the view (or its label), or in the object tree. Locked props are
  click-through in the view, as in Dead Reckoning; the tree still selects them.
- **Move** it by dragging the prop across the pool, its arrows (3D), or the yellow ring to turn it (`Shift` snaps
  to 15 degrees). `Esc` during a drag puts it back. Keys:
  arrows nudge 1 cm along the pool (`Shift` 10 cm), `PgUp` / `PgDn` height, `Q` / `E` turn 1 degree (`Shift` 15),
  `L` lock, `H` hide, `Delete`, `Esc` deselect. Children ride along with their parent.
- **Type** a pose relative to the parent (as the file stores it) or in the map; the pool position and depth are
  shown below. The inspector also sets the parent (re-parenting keeps the prop where it is), the
  `lock_orientation_to_config` / `point_yaw_at_parent` flags, `class` and covariance; swaps poses or classes with
  a sibling (the two gate sides, fire / blood); adds a child, duplicates, renames (children follow) and deletes
  (children move to the map where they are).
- **Map origin** (nothing selected): *AprilTag* puts it on a wall where a floor line meets it (or a corner), +X into
  the pool; *Place origin* then click near a line end. *Robot frame* puts it anywhere in the pool with a free
  heading: it is the robot's start pose, so the robot is drawn there; drag the robot in the view to move the origin
(in 3D click it first), its yellow ring to turn it. *Turn ±90* and *yaw offset* turn it too. *Pin props to the pool* keeps the props where they are when the
  origin moves (their map poses change instead). It starts where the scenario has the map (the calibration
  board).
- **Undo / Redo** (`Ctrl+Z`, `Ctrl+Shift+Z`) cover every edit. **Save** (`Ctrl+S`) writes only the values that
  changed: comments, order, other robots' sections and the deprecated entries stay byte for byte, and the result
  is read back and checked before the file is replaced. The namespace list switches robots in a file with several.

Locks, hidden props, the origin (per pool) and the view options are the editor's own, kept per file in
`~/.config/nereus/prior_map/`; the Map layout is `~/.config/nereus/map_layout.ini` (Layout → *Reset map layout*).
`prior_map.config` in the host config (or `--prior-map FILE`) names the file opened at start; `--workspace map`
starts editing the map. Quitting with unsaved changes asks first.

## Scoring a run (sim)

Open **Run tracking** (toolbar) for the scorecard window:

1. Set the run options (coin-flip role, heading and role coin flips) and **Start run**. The timer runs on
   simulation time; **Stop run** ends it.
2. The status lines show the role of the gate side the robot took and the coin-flip bonuses earned.
3. **Score adjustment**: type a signed amount and **Add** for subjective points; **Clear** removes them.
4. **Reset run & tasks** puts every prop and payload back.

A rejected command (for example Start while a run is running) shows as *Command rejected: ...*.

## Configuration

All of it is YAML in `content/viewer/`:

| File | Controls |
| --- | --- |
| `talos_uwrt_host.yaml` | window size and `title_bar`, default `layout` and `theme`, topics, pose source and delays, estimate ghost/anchors, detections, `point_clouds`, `mapping_markers`, `prior_map`, `thrust_arrows`, focus presets |
| `talos_uwrt_panels.yaml` | the pool view `toolbar:`, command bar `header:` and the `panels:` windows (order, titles, `dock:` area in the built-in layouts, which start open or as the shown tab), and the providers they talk to (telemetry readings and thresholds, recording services) |
| `talos_uwrt_thruster_visuals.yaml`, `talos_uwrt_status_lights.yaml` | rotor animation and LED bars |

Toolbar, header and panel items are listed by type; reorder or remove entries to change what is offered. A
panel's `dock:` is `left_top`, `left`, `right`, `right_bottom`, `bottom` or `floating`; panels sharing an area are
tabs, and `open: false` puts one behind the others. `visible: false` starts it closed. For command-line
options run `build/ros-viewer/nereus-viewer --help`.
