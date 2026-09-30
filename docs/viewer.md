# Using the viewer

`nereus-viewer` is the operator interface for both the simulator and the real robot. The sim launch
starts it for you; for the real robot use `ros2 launch integrations/uwrt/launch/robot.launch.py`.

![Viewer with the robot in the pool](images/robot.jpg)

- **Top bar**: view, focus and display toggles.
- **Left sidebar**: robot control panels (Motion, Mapping, Vision, Actuators, Autonomy). Click a header to fold it.
- **Centre**: the 3D scene. The pill at the top right says where the robot pose comes from (`PHYSICS CONNECTED`
  in sim, `ROBOT (ESTIMATE)` on the real robot).
- **Right**: camera cards and the course map.

## Moving around

| | |
| --- | --- |
| Left drag | orbit |
| Right / middle drag | pan (detaches Follow) |
| Scroll | zoom |
| `F` | focus on the point under the cursor |
| `F3` | frame-time stats |
| View → *Free camera* | click for mouse look, `WASD` move, `Space`/`Shift` up/down, `Ctrl` fast, `Esc` release |
| View → *ffc* / *dfc* | look through a robot camera |

**Focus** jumps to the vehicle, a mechanism (Claw, Payloads) or a course element; **Follow** keeps the camera on
the robot. **Labels** names course elements; **TF** draws frame axes; **MPC path** draws the MPC's predicted path.

## Driving the robot (Motion)

1. **Enable**. The enable latches: the robot keeps running if the viewer closes or the tether drops. **KILL** (or
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

## Vision

- **Detections**: the detector's markers, placed where the object was seen. *Placement* (sim only) chooses
  simulator truth, the localization estimate, or both. *Keep detections* lets markers live out their lifetime
  instead of clearing each frame.
- **Point clouds**: one checkbox per cloud in the viewer config (YOLO detections, front and down camera); a cloud
  is only subscribed while its box is ticked. Newest message only, like RViz.

## Camera cards

**RGB / DEPTH** switches the image. In sim the label under the image is a button: **truth pose** shows the
viewer's own render from the true camera pose (fast, no noise); click it for **ROS (stack)**, the images your
stack actually receives.

## Pool Viewer menu

- **Water, Pool walls & deck, Pool floor, Surface reflections**: visibility (camera cards always see the pool).
- **Course**: *Auto* shows the simulator's course in sim and the mapping estimate on the real robot; *Mapping
  (RViz)* draws `riptide_meshes` models at the mapping TF frames, exactly as RViz does. *Mapping ghost* (sim)
  overlays the mapping estimate translucent on the true course.
- **Localization estimate** (sim only): *Robot ghost* draws the EKF estimate as a translucent robot. *Control
  gizmo* and *Follow* each centre on the estimate or the true robot. Commands always go to the controller in its
  estimate frame.
- **Lighting**: the observer's lighting, independent of what the cameras see.

**Scene settings** has the simulator's mechanism buttons, lighting and water appearance; **Simulation settings**
sets the speed (pause, 1x, faster), **Sync sim** to the estimate and **Reset sim**.

## Scoring a run (sim)

Open **Run tracking**:

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
| `talos_uwrt_host.yaml` | topics, pose source and delays, estimate ghost/anchors, detections, `point_clouds`, `mapping_markers`, focus presets |
| `talos_uwrt_panels.yaml` | `toolbar:` and sidebar `panels:` (order, titles, which are open), and the providers they talk to |
| `talos_uwrt_thruster_visuals.yaml`, `talos_uwrt_status_lights.yaml` | rotor animation and LED bars |

Toolbar and sidebar items are listed by type; reorder or remove entries to change the layout. For command-line
options run `build/ros-viewer/nereus-viewer --help`.
