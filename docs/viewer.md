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
- **Windows** around it: the robot panels (Motion, Autonomy, Mapping, Vision, Recording, Bagging, Electrical,
  Actuators),
  one window per robot camera, and the course map. Every one of them can be moved, see
  [Arranging windows](#arranging-windows).

## Moving around

| | |
| --- | --- |
| Left drag | orbit |
| Right / middle drag | pan (detaches Follow) |
| Scroll | zoom |
| `F` | focus on the point under the cursor |
| `Ctrl+F` | Follow: keep the camera on the focus target as it moves |
| `F3` | frame-time stats |
| `F1` | controls and shortcuts |
| `Ctrl+Space` | maximize the pool view (close every other window); again to restore them |
| `Ctrl+Shift+1` / `2` / `3` | Standard / Wide view / Camera wall layout |
| `Ctrl+[` / `Ctrl+]` | snap the left / right panels shut, or bring them back |
| View → *Free camera* | click for mouse look, `WASD` move, `Space`/`Shift` up/down, `Ctrl` fast, `Esc` release |
| View → *ffc* / *dfc*, or **Main view** on a camera window | look through a robot camera |

The camera and focus dropdowns lead the pool view's toolbar, which is grouped (camera, overlays, windows, task
preview) by thin dividers. **Focus** jumps to the vehicle, a mechanism (Claw,
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

- **Move** a window by dragging its tab. Drop it on another window's center arrow to add it as a tab there, on an
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

## File menu

- **Pool**: switch the pool (in the simulator, the supervisor restarts it there).
- **Prior map**: Open (a riptide_mapping `config.yaml` by path, or Browse... for the desktop's file picker), Open
  recent, Save (Ctrl+S, from anywhere), Reload (asks first over unsaved edits). Map editing (Edit map, Ctrl+M) is
  where it is changed; its Map objects window has the same path field and Browse....
- **Capture**: Save screenshot (F12: the whole window, a PNG in `~/Pictures/Nereus/`), and that folder.
- **Folders**: the simulator's run records (`/tmp/nereus_sim/`) and the viewer's settings (`~/.config/nereus/`:
  preferences, the saved arrangement, named layouts), in the desktop's file manager.
- **Quit** (Ctrl+Q), asking first when the prior map has unsaved edits.

## Search (Ctrl+P)

The box in the middle of the title bar (or Ctrl+P) searches everything the menus do: windows, layouts, themes,
interface scales, focus targets, camera views, view toggles, pools, map editing and Help; and every button,
checkbox, dropdown choice and switch in every window (the controls that can be pinned to the toolbar), listed by
window ("Display  Water", "Motion  Mode: Feedforward"), even windows that are closed: running one does what a
click on it would. Type a few letters in
order (`cwall` finds the Camera wall layout), choose with the arrows and Enter; Esc or a click elsewhere closes it.
Enable and KILL are deliberately not in it.

### The robot and autonomy from the keyboard

Type a move and Enter sends it (Operate workspace; as the Motion panel's **Command**, only while the robot is
enabled and not in Feedforward control, its pose is fresh and no tree or other operator is driving it). Moves start from the pose last commanded,
so they add up while the robot travels; the first row reads back the move and where it ends up.

| Typed | Does |
|---|---|
| `forward 0.5`, `back`, `left`, `right`, `up`, `down` | meters along the heading (level with the water; up is +z) |
| `turn 30`, `turn right 45` | yaw by degrees, left positive |
| `x 2`, `y 1`, `z -1.5`, `roll 0`, `pitch 0`, `yaw 90` | that axis to the value |
| `go 2 1 -1.5 90` (or `go x y z`, `go x y z roll pitch yaw`) | the whole pose |
| `level` | roll and pitch to zero |

Several in a row run in order (`turn 90 forward 1`, `x 2 yaw 180`); units are optional (`0.5m`, `30deg`).
The Robot entries (*Move…*, *Turn…*, *Go to…*, *Set…*) put the first word in the box for the rest to be typed.

**Drive with the keyboard** (in the search, or **Drive with keys** in the Motion panel; Position control only)
moves the target one step a key press: W / S forward / back,
A / D left / right, Space / Shift up / down, Q / E turn; `[` / `]` choose the step (5 cm / 2° up to 1 m / 45°, 25 cm /
15° to start). Holding a key does not repeat. The pool view's key strip shows the keys and step while it is on;
Esc, the Map workspace, Feedforward, a kill or a tree starting ends it. Keys go to a text field when one has the keyboard, and
to the free camera while it has the mouse.

**Run <tree>** starts an autonomy tree, under the same rule as the Autonomy panel's Start (the robot it drives
enabled); **Stop** stops the running one, **Refresh the tree list** asks again.

## Plots

Plot any number from any ROS topic, live, in dock windows like the rest (**Windows › Plots**, *New plot*).

- **Lanes**: a plot stacks lanes on one time axis. Each lane has its own value axis and a results table: the series,
  its value now (or under the cursor), and its min and max over the span, with the topic and field under each name.
  A narrow window drops Min / Max and then moves the tables under the plot.
- **Adding**: the **+** at the right of a lane's head (or *Add series…* in its right-click menu) searches for a
  field to add to that lane: type part of its name (`odom z`); Shift-click adds several. **+ Lane** in the toolbar
  starts a new lane the same way. Or drag a field from **Topics** (Windows › Topics) onto a lane or onto the *Drop
  here for a new lane* strip; double-click a field to add it to the focused plot. Quaternions show as roll / pitch / yaw in degrees; arrays list their elements. Values and rates show
  while a topic is open in Topics or plotted; nothing else is subscribed.
- **Plot this**: right-click a row of the Motion table (*Plot Z* gives actual and commanded, then the error; *Add Z
  to ▸*; *Plot all six axes*) or a temperature / battery readout in the command bar. Pointing at one of those figures
  shows its last 30 s in the tooltip.
- **Ctrl+P**: the search lists New plot and your saved plots. Fields come only after `plot ` (or *Plot a topic
  field…*, which types it): `plot odom z`, `plot motion yaw`. Enter opens a new plot; Shift+Enter adds to the
  focused one.
- **Time**: every plot window shares one timeline. **Live / Paused** freezes them all while recording goes on;
  drag a plot to scrub (it pauses), Ctrl+scroll to zoom around the pointer, double-click to go back to live. The
  wheel alone scrolls a plot window that holds more lanes than fit. The cursor is one rule through every lane and
  every plot, and each table reads its time. *··· › Own time span* takes one window off the shared timeline (a long
  battery plot beside short depth plots).
- **Time from**: *Header* (the default) plots each sample at its message's `header.stamp`, so truth and estimate
  line up; messages without a header use the time they arrived. *Receipt* uses arrival time for everything. On the
  real robot a header stamp is the Orin's clock: a row that says `clock offset 1.2 s` means the clocks disagree.
- **Moving a series**: drag its row onto another lane (its plot or its table), onto the new-lane strip, or into
  another plot window. A lane the move empties goes, so dragging a lane's only series onto another merges them.
- **A series**: point at its row for the remove box (Del also removes it; Ctrl+Z brings back the last series or lane removed); click its
  line key to hide or show it; right-click for its color (the theme's eight, or *Custom…* with a contrast check
  for day and night themes), *Move to lane*, *Draw as* steps or lines, *Rename*, *Difference with* (a new lane of
  one series minus another) and *Remove from plot*. Past eight series in one lane the colors repeat, dashed.
  Right-click a lane's head to rename it, set its unit or its limits (drawn as its outer ticks), or remove it.
- **Rows say why** a line stopped: `stale 4.2 s`, `no publisher`, or *Can't read this topic* when its message type is
  not installed here (source the robot workspace before starting the viewer).
- **Saving**: *··· › Save as preset* keeps the plot's lanes and series (not the data) as
  `~/.config/nereus/plots/<name>.yaml`; open it again from Windows › Plots or Ctrl+P, and manage the saved ones
  from *Manage saved plots…*. Plot windows are also part of the layout (and of named layouts). Closing a plot
  window hides it and it keeps recording; *··· › Delete plot* removes it. *··· › Export visible span as CSV* writes
  `~/Documents/Nereus/plots/<plot>-<time>.csv`.
- Each theme can set its eight plot colors (`plot: series:` in its YAML); without one the plots use a validated set
  for light or dark windows.

## Themes

View → *Theme* ▸ **Heat sheet** (the default) and **Timing board** are one world in a day and a night variant,
taken from a swim meet's results sheets and the pool's timing board:

- **Sheets**: panels are white sheets on a rule-gray ground by day, charcoal by night, with square corners. Section
  and column heads carry a heavy ink rule; table rows are ruled with hairlines.
- **Figures**: every number is set in Barlow TF, Barlow with tabular figures (`content/viewer/fonts`, SIL OFL; rebuilt
  by `make_fonts.py`), so columns line up.
- **The board**: the menu and command bars are ink by day and black by night, with white type. The run clock and
  score sit on it as large figures, lit amber while a run is on. In map editing, a lane-blue rule runs along its foot.
- **Color jobs**: lane blue is selection and "on". Amber lights the running clock. Touch red is KILL. Enable is go
  green (in every theme), never the blue of a lit toggle; its label says Enable or KILL, so color is never the only
  cue.
- **Switches**: a choice between two (RGB / Depth, Truth / ROS, Position / Feedforward, 3D / 2D, AprilTag / Robot
  frame) is one joined control whose thumb slides to the chosen side. Pinned to the toolbar, it shows as a dropdown.

The other themes: **Abyss** (dark teal), **Midnight** (neutral dark with a blue accent), **Daylight** (light), **High
contrast** (black, white and yellow), **Ocean** (navy and blue), **Arctic** (slate and frost blue), **Ember** (charcoal
and orange), **Sonar** (green on black), **Paper** (warm light) and **Classic** (Qt's Windows-style gray with bevels
and the Ubuntu font, like RViz on this desktop).

The choice is remembered (`~/.config/nereus/viewer.yaml`); `--theme NAME` or the host config's `theme:` set it too.

Each theme is one YAML file in `content/viewer/themes/`; add a file to add a theme (no code). A file gives `id`,
`label`, `description`, `order` (the menu's order, lowest first, the first is the default) and optional `fonts`
(`family` and `points` for a desktop font, or `regular` / `strong` / `figures` files in `content/viewer/fonts`). Its
colors either come from four with `derive: {background, text, muted, accent, rounding}` (every surface is derived)
or are listed in `surfaces` (window, frame, button, tab, ...) and `palette` (text, muted, accent, active, danger,
warn, error, robot states, bar, enable, ...), as `"#rrggbb"` or `"#rrggbbaa"`. `shape` and `spacing` set corners,
borders, padding and the tabs' look (`tab_rounding`, `tab_border`, `tab_bar_border`, `tab_overline`); a
`border_shadow` surface makes the theme bevelled (Qt-style raised buttons, sunken status panels, toggle switches as
pressed buttons); `style: {ruled: true}` rules the heads; a `board` block gives the menu and command bars their
own colors; a `plot` block lists the eight plot line colors (`series`). Unknown keys are refused, and the viewer reports a file it cannot use and carries on. Whatever a file
says, the viewer holds every theme's text to the legibility floor below. The
3D scene looks the same in every theme. Its overlays (labels, the controls strip) and the course map follow the
theme. The course map draws each prop's top-down footprint (true size and heading) under the task dots. Camera
images carry no overlay: the Truth / ROS switch beside RGB / Depth shows the source (in the simulator), and the
header line shows the camera, its resolution and the live rate. Every theme keeps secondary, status and accent text and the labels of lit buttons at
5:1 contrast or better against what they sit on, for a sunny pool deck. Undocked windows carry a hairline edge and a soft shadow.
At a larger interface scale, the side columns of the built-in layouts widen.

## Interface scale

The title bar (the Nereus mark, the menus and the window buttons) follows the desktop's display scale (GNOME at
200 %: twice the size), sized like VS Code's. The rest follows the desktop too (*Match desktop*, the default), or a
fixed View → *Interface scale* ▸ 100–200 %. The choice is remembered; `--ui-scale 1.5` / `--ui-scale auto` or the
host config's `interface_scale:` set it too. The window opens at the configured size in the desktop's units, or
maximized when that does not fit.

The viewer saves your arrangement when it closes (`~/.config/nereus/viewer_layout.ini`) and starts with it next
time. `--layout NAME` starts with a built-in layout (`standard`, `wide`, `cameras`), a saved layout's name or an
`.ini` file instead; with no saved arrangement the host config's `layout:` is used.

## Robot temperatures and batteries

The header chips show the FOG temperature (`gyro/status`) and the hottest CPU core (the computer monitor's
`Core Temperature` in `/diagnostics_agg`): cyan is fine, amber a warning (FOG 55 °C, CPU 70 °C), red an error (FOG
75 °C, CPU 85 °C, or a fault the FOG driver or the computer monitor reports). Gray means no data: never received,
or nothing for 2 s (FOG) / 10 s (CPU), when the last value stays shown. Hover a chip for the details.

`PORT` and `STBD` are each battery's state of charge (`state/battery`), as in the RViz overlay: amber below 50 %,
red below 20 %, gray after 10 s without a reading. Hover for the pack voltage, current, time to discharge and
cell. In sim none of these publishers run, so all four chips stay gray.

## Driving the robot (Motion)

1. **Enable** (top of the Motion panel, or the command bar). The enable latches: the robot keeps running if the viewer closes or the tether drops. **KILL** (same button, or
   the physical kill) stops it.
2. Pick **Position** or **Feedforward**.
3. Drag the gizmo in the 3D view (arrows move, rings rotate; `Esc` during a drag restores the start), or type a
   target in the table and press **Command**. **Current** copies the robot's pose into the table; **Dive in
   place** targets the current x/y and heading, level, at the configured depth (`dive_z`). **Zero roll & pitch**
   (beside *Pose gizmo*) holds the current position and heading and levels the robot where it is.

If the viewer freezes or the pose goes stale, it releases manual control and the robot holds its last command;
it never kills on its own. Another operator sending on the same kill switch (for example RViz's control panel)
does kill: use one operator at a time.

## Autonomy, Mapping, Actuators

- **Autonomy**: pick a tree, **Start** / **Stop**; the execution stack shows the running nodes. Manual control is
  blocked while a tree owns the robot.
- **Mapping**: tag calibration (**Calibrate**, with a progress bar of the samples taken), reset, and the mapping
  target (**Lock map**).
- **Actuators**: arm/disarm, fire torpedoes, drop markers, open/close the claw, reload.

## Recording

- **SVO recording**: the path is on the robot (`~` is `/home/ros`). Each camera records to its own file,
  `<path>_<date>_<time>_<camera>.svo2` (untick *Add date and time* to drop the timestamp). **Start** calls the ZED
  node's `start_svo_rec` (lossless); **Stop** calls `stop_svo_rec`. The viewer knows only the recordings it
  started, so **Stop** works whenever the service is up, including for a recording started before a restart.
  Starting needs `zed_msgs` when the viewer is built; without it the panel says so and only Stop works.
- **Capture image**: the picture taker (`capture_image`) saves the newest front camera frame on the robot; the
  reply lists the saved files.

## Bagging

`ros2 bag record` on the robot or on this computer: pick **Record on** (Robot / This computer), then **Record**.

- **Robot**: runs on the machine in the **ssh** field, prefilled with `ros@orin2`: type another `user@host` and press
  Enter to record there instead (key login, as `ssh <it>` from a terminal; locked while a bag records). The bag keeps
  recording if the viewer closes or the link drops; a restarted viewer, or another laptop, shows it and can stop
  it. The line under the switch says how much disk the robot has left (yellow under 5 GB), or why it can't be
  reached.
- **This computer**: runs where the viewer runs and stops when the viewer closes.
- **Folder** and **Name**: the bag is the folder `<folder>/<name>_<date>_<time>` (untick *Add date and time* to
  drop the timestamp; an existing bag is never overwritten).
- **Topics**: *All* records every topic, including ones that appear later; *Leave out* takes a regex (`-x`), for
  example `image|point_cloud` to skip camera streams. *Selected* records the ticked topics (the live list, with a
  filter and the configured presets); a ticked topic that is not published yet is recorded once it appears.
- **Stop** sends Ctrl-C so the recorder closes the bag properly (writes `metadata.yaml`). A recorder that is still
  closing after 10 s shows **Kill recorder**; a killed bag keeps its data and `ros2 bag reindex` rebuilds its
  metadata.
- While a bag records, the command bar shows a red **BAG** chip with its time (hover: where it is and its size).
  The last bag's path stays in the panel; **Copy path** copies it as `ros@orin2:/home/ros/bags/...` for
  `scp -r` or `rsync`.

Each machine runs at most one bag from the viewer at a time; its state is in `~/.local/state/nereus/bag/` there
(`record.log` is the recorder's output). Targets, their `setup` (what is sourced before `ros2`), extra
`record_args` and the presets are in `talos_uwrt_panels.yaml` under the `bags` provider.

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
  gizmo* and *Follow* each center on the estimate or the true robot. Commands always go to the controller in its
  estimate frame.
- **Lighting**: the observer's lighting, independent of what the cameras see.

**Scene** (the Scene settings window) has the simulator's mechanism buttons, lighting and water appearance;
**Simulation** sets the speed (pause, 1x, faster), **Sync sim** to the estimate and **Reset sim**. Both are
windows: dock them if you use them often.

## Editing the prior map (Dead Reckoning in 3D)

**Edit map** in the command bar (`Ctrl+M`) edits riptide_mapping's `config/config.yaml` in the pool, as the Dead
Reckoning tool does. While editing, the command bar turns the accent color and reads *EDITING PRIOR MAP ·
config.yaml* (with *unsaved* when there are changes), with **Save** and **Done** at its right end; Enable / KILL stay
where they are. Editing has its own window layout: **Map objects** on the left (the file, Open / Reload / Save, Undo
/ Redo, *+ Add* and the object table: the tree with lock / hide and, toggled by **x y z**, each prop's x / y / z / yaw
relative to its parent as the file stores it; click a value, type, Enter to change it), the **Inspector** on the right (the selected prop, or the map origin when
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

- **2D** (the default) looks straight down at the pool (the same render, without perspective), drawn as a chart in
  the theme's colors: a flat floor with quiet lane lines, each prop outlined in its own color (the color of its
  dot in the table) with a notch toward +X, props without a mesh as badges with a heading triangle, a scale bar and
  the labeled map origin (*Plan colors* in Display turns the chart colors off). For quick moves across a flat
  floor: drag a prop to move it in x / y (its height stays), the yellow ring turns it. Drag the floor
  (or right / middle drag) to pan, scroll to zoom about the pointer, `F` or *Fit* for the whole pool, double-click a
  prop (or its row in the tree) to center on it.
- **3D** orbits; the selected prop has red / green / blue arrows for the map's X / Y and height, and the ring.

A click picks what is under the pointer: a prop's mesh (its bounds), its label or its origin dot.
- **Select** a prop by clicking it in the view (or its label), or in the object tree. Locked props are
  click-through in the view, as in Dead Reckoning; the tree still selects them.
- **Move** it by dragging the prop across the pool, its arrows (3D), or the yellow ring to turn it (`Shift` snaps
  to 15 degrees). `Esc` during a drag puts it back. Keys:
  arrows nudge 1 cm along the pool (`Shift` 10 cm), `PgUp` / `PgDn` height, `Q` / `E` turn 1 degree (`Shift` 15),
  `L` lock, `H` hide, `Delete`, `Esc` deselect. Children ride along with their parent.
- **Type** a pose in the table or the inspector. The inspector's **Stored** pose is what the file holds: relative
  to the prop's parent (for a top-level prop the parent is the map, so it is simply *Pose in the map*). A child also
  shows its **Resulting map position** (its parents' poses and its own combined), editable too: the stored pose
  follows. The pool position and depth are shown below. The inspector also sets the parent (re-parenting keeps the prop where it is), the
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

### The Sim course

The switch at the top of **Map objects** picks what is edited: the **Robot's map** (what the robot believes, the
riptide_mapping file above) or the **Sim course** (what the simulator runs: the scenario's task placements, the
tasks' loose objects and the fixed run options). The bar then reads *EDITING SIM COURSE*. The robot's map is drawn
translucent for reference while the course is edited.

- **Tasks** move and turn as props do (drag, arrows, ring, keys, the table and inspector), in the world. The pool
  view draws each task where it is being put. A task's **loose objects** (the table's pill, bandage, nut and bolt and
  plug) are its children, positioned on the task.
- **Options** (inspector, nothing selected) are the tasks pack's fixed run options: the four bin vinyls' classes.
- **Save** writes what changed into the scenario pack's `scenario.yaml` (`python -m nereus.packs set-course`:
  comments and untouched entries stay as written, and the edit is checked by resolving first). The simulator then
  restarts on it, the stack keeps running. **Discard** goes back to the course as saved. There is nothing to add,
  rename or delete: the tasks are the task pack's.
- **Copy from the robot's map** (Sim course) and **Copy from the Sim course** (Robot's map) carry the linked tasks'
  poses, their loose objects and the classes across, as one undo step. The host config names the links:
  `prior_map.course_links` maps each task to the prop at its origin (`slalom: slalom_parent`, `surface: octagon`).
  Loose objects match props of the same name, and a prop's class sets the `<prop>_class` option. `prior_map.floating`
  lists tasks whose height is never copied (the octagon floats).

`--open sim-course` starts on the Sim course. **Save** in the command bar (`Ctrl+S`) saves both layers; quitting with
unsaved changes in either asks first.

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
| `talos_uwrt_panels.yaml` | the pool view `toolbar:`, command bar `header:` and the `panels:` windows (order, titles, `dock:` area in the built-in layouts, which start open or as the shown tab), and the providers they talk to (telemetry readings and thresholds, recording services, bag targets and presets) |
| `talos_uwrt_thruster_visuals.yaml`, `talos_uwrt_status_lights.yaml` | rotor animation and LED bars |

Toolbar, header and panel items are listed by type; reorder or remove entries to change what is offered, and put a
`{id: <unique>, type: separator}` between toolbar groups for a divider. A
panel's `dock:` is `left_top`, `left`, `right`, `right_bottom`, `bottom` or `floating`; panels sharing an area are
tabs, and `open: false` puts one behind the others. `visible: false` starts it closed. For command-line
options run `build/ros-viewer/nereus-viewer --help`.
