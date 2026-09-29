# Old pool_viewer inventory (feature parity checklist)

Scope: `src/riptide_simulator/camera_faker` (package `camera_faker`, node `pool_viewer`). Camera-faker role
(ZED publishers, point-cloud/depth publishing, `camera_processor`, `camera_cuda.cu`) is out of scope: the new
bridge renders cameras. Robot `talos`, so relative names expand to `/talos/...`.
Abbreviations: `M` = `src/pool_viewer/main.cpp`, `R` = `src/pool_viewer/renderer.cpp`, `P/` = `src/pool_viewer/panels/`,
`H/` = `include/pool_viewer/`, `C` = `src/riptide_simulator/c_simulator`, `T` = `C/tasks/2026`, all under
`/home/ubuntu/osu-uwrt/release/src/riptide_simulator/` (`camera_faker/` prefix implied for M, R, P/, H/).
Old physics node = `C/src/physics_simulator.cpp` (`PS`), old task node = `T/behavior/node.py` (`TN`), scoring = `T/behavior/scoring.py` (`SC`).

## 1. User-visible features

Window: GLFW 1480x940 "Riptide | RoboSub Pool" (M:909), ImGui dark theme (M:930-955), fonts DejaVu (M:921-929),
vsync off, `render_rate` 30 Hz cap (M:147, 787). One fullscreen ImGui window "Riptide" (M:2035).

### 1.1 Header / layout

| Feature | Shows / does | Source | Data needed |
|---|---|---|---|
| Title | "RIPTIDE / ROBOSUB SIMULATION" (hardcoded) | M:2038-2045 | none |
| Connection pill (top right) | `SCENE PREVIEW` (demo, amber) / `WAITING FOR PHYSICS` / `POSE STALE` (no new truth pose stamp for >1 s wall) / `PHYSICS CONNECTED` (cyan) | M:960-989, 2049 | TF `map -> simulator/talos/base_link` |
| Left operator sidebar | Pinned Kill/Enable + collapsible panels; width 300-600 px, fraction .29 (`sidebar_width_fraction`), drag-resizable edge with show/hide handle (`PanelEdge`) | M:2052-2075, H/panel_layout.hpp, P/composition.cpp:227 | composition yaml |
| Right camera sidebar | Camera cards + mini course map + footnote; 300-600 px, resizable/hideable | M:2064-2080, 2380-2392 | cameras |
| Viewport | Main 3D view (orbit / free / sensor cam) with 30 px footer help strip, pool-name badge (top-left: `<world.id> / L x W m`), run readout (top-right, only if score received: `"%.1f pts / MM:SS.s [RUNNING]"`) | M:2258-2305 | `world.{id,length,width}` (scene yaml), `run_score.{total,elapsed,running}` |
| Footnote | demo: "Scene preview..." else "Images follow physics TF. Observer controls do not move the vehicle." | M:2385 | none |

### 1.2 Toolbar (above viewport, wraps via `sameLineIfFits`, H/panel_layout.hpp)

| Control | Behaviour | Source |
|---|---|---|
| Scene settings (button) | toggles floating "Scene settings" window (1.6) | M:2091 |
| `tools` with `slot: settings` | e.g. "Simulation settings" popup button (1.4) rendered right after Scene settings | P/composition.cpp:219, M:2093 |
| Pool Viewer (popup) | observer-only render toggles: Water, Pool walls, Surface reflections (default off), Shadows, Lighting combo (Scene/Indoor/Outdoor/Sterile), Exposure .4-2x, Brightness 0-4x, Ambient 0-3x, Reset lighting. Never touches sensor renders (`ObserverSettings::apply`, H/renderer.hpp:111-145) | M:2096-2122 |
| Panels (popup) | checkbox per panel instance to show/hide in sidebar | P/composition.cpp:202 |
| Collapsed-sidebar toolbar buttons | each panel's `toolbar()` (Motion: Enable/KILL 90 px) when sidebar hidden | P/composition.cpp:210, motion_panel.cpp:84 |
| View combo | Orbit / Free camera / one entry per sim camera (`ffc`, `dfc`); sensor view promoted to main viewport keeps sensor aspect | M:2126-2147 |
| Focus combo | list = `ui.focus` of task doc (default `Course, Vehicle`) | M:2148-2153, 1212 |
| Follow checkbox | camera target tracks focus target; disabled and forced off for `Course` | M:2154-2161, 1310-1315 |
| Labels checkbox | 3D labels above each `ui.focus` landmark (cyan dot + boxed name, hidden for Payloads/Claw focus, mode<2 only) | M:2163, 2282-2296 |
| TF (popup) | see 1.9 | M:2166-2181 |
| Detections checkbox (non-demo) | see 1.8 | M:2183-2188 |
| MPC path checkbox (non-demo) | see 1.8 | M:2189-2195 |
| `tools` default slot | e.g. "Run tracking" button after MPC path | M:2196 |
| Preview-task combo (demo only) | `ui.demo_targets`; `previewPose()` puts vehicle 2 m in front of landmark, depth from `ui.previews.<n>.depth`, camera offset from `ui.previews.<n>.camera` | M:2198-2207, 1188 |

### 1.3 Camera controls, follow/focus, mouse and keyboard

| Input | Effect | Source |
|---|---|---|
| Orbit mode: LMB drag | orbit (yaw -= dx*.005, pitch clamp +-1.55) | M:1313-1325 |
| RMB/MMB drag, or Shift+LMB | pan in camera plane (`orbitPan`, H/viewer_input.hpp:8); detaches Follow | M:1300-1323 |
| Scroll | zoom, distance *= exp(-wheel*.1), clamp .15..75 m | M:1328 |
| `F` (hover viewport, not typing, not gizmo drag) | focus on cursor: pick TF axes / detection quad / arrow / MPC segment first, else depth-buffer point, else focus-plane point; detaches Follow | M:2247, 1348-1399, H/viewer_input.hpp:35-98 |
| Free camera: click viewport | capture mouse (raw motion), look; WASD move, Space up, LShift down, Ctrl fast (8 vs 2.5 m/s), Esc or window blur releases; entering Free continues from current view incl. sensor roll | M:1262-1298, 2133-2145 |
| Footer strip text | orbit: "LEFT DRAG orbit RIGHT / MIDDLE DRAG pan SCROLL zoom F focus cursor"; free: "CLICK mouse look WASD move SPACE / SHIFT up / down CTRL fast ESC release" | M:2297-2304 |
| Focus presets | `Course`: target pool centre, dist = max(L,W)*.65, pitch .64 yaw -2.5, no follow. `Vehicle`: dist 1.9 pitch .32 yaw = vehicle heading, follow. `Claw`: target claw mount+(0,0,.06), dist .65. `Payloads`: mean of payload mount positions, dist .48, pitch -.18. Landmark name: dist 3.4 (`table` 3.7; `magnet_target*` dist .32 pitch .6), yaw = landmark facing | M:1212-1253 |
| Focus first-pose fix | on first truth pose, re-runs Vehicle/Payloads/Claw focus | M:970-973 |
| Orbit focus dot | while orbit-dragging/zooming without Follow, a screen-space disc drawn at target (`focus.frag`, R:956-967) | R:956 |
| Trail | last 1200 vehicle positions (>6 cm apart) drawn on course map | M:976-982, 1626 |
| Initial view | `initial_focus` param (default `Vehicle`), fov 53 deg, near .05 far 100 | M:590, 1341 |

### 1.4 Operator panels (left sidebar, from composition yaml; validated, no ROS in `demo`)

Common: each panel = `pinned()` (always visible strip), `toolbar()` (button when sidebar hidden / tool slot), `draw()` (collapsible section with `disclosureHeader`), `drawWindows()` (floating). Provider state fed by a separate node `viewer_panels` (ns `/talos`, own executor thread, `use_sim_time` param) (P/ros_runtime.cpp:8-30).

| Panel type | UI | Source | Provider needed |
|---|---|---|---|
| `motion` | Pinned full-width **KILL / Enable** button (red when killable: enabled/pending/blocked/competing/observed-not-killed; teal Enable otherwise) + line "Robot: killed / enabled / state unknown". Body: frame label, status message, Position / Feedforward mode buttons (FF disabled if unsupported), 6-row table X Y Z Roll Pitch Yaw x {Actual, Commanded, Error (rem 360 for angles), Target (editable, right-aligned)}, buttons Current / Command / **Dive in place** (only if `dive_z`; sets z=dive_z, level roll/pitch, Position), hint "Target edited / not sent" or "Drag an axis or ring in the pool view", per-overlay checkboxes ("Robot controls"). Options `dive_z`, `dive_max_depth_z` (validated only; `dive_max_depth_z` unused in code) | P/motion_panel.cpp:84-219 | `Motion` (uwrt.motion or ros.pose) |
| `autonomy` | status text; searchable tree combo (friendly file name, full path tooltip), Refresh, Start (needs linked motion enabled+fresh, `mayStart`), Stop, "Tree: <name>", stack list (index + node text, last colored) "Execution stack / stale / Last execution stack" | P/autonomy_panel.cpp:16-79 | `Autonomy` (uwrt.autonomy) |
| `mapping` | Tag calibration: Parent frame, Tag frame, Samples inputs (defaults from options `parent_frame`,`tag_frame`,`samples`); button Tag cal / Cancel tag cal; "Samples: n / N"; Mapping target: Current (or "Automatic"), Map locked/unlocked, target text input, Lock map checkbox, Set mapping target, Reset mapping | P/mapping_panel.cpp:18-70 | `Mapping` (uwrt.mapping) |
| `actuators` | status message ("Actuators armed/disarmed/status stale"), one full-width button per configured command (label swaps to `armed_label` when armed; disabled if stale or `requires_armed` and disarmed), "Status" readings: `Torpedoes remaining`, `Markers remaining` | P/actuator_panel.cpp:10-26, uwrt_actuators.cpp:60-97 | `Actuators` (uwrt.actuators) |
| `run` | Sidebar: "MM:SS.s / N points", Detailed scorecard button, schema-driven run options (bool checkbox / number / choice combo) disabled while running, Start run, Stop run, Reset run & tasks, schema `actions[]` buttons, message, `status_fields`, `message`/`ended_reason`, note "Timer uses simulation time...". Floating window (also toolbar "Run tracking", or `show_scorecard`): same controls + "Task status" (task_score JSON text), "Magnet targets" GREEN/RED list, sim readings ("Jaw gap: N mm"), "Recent events" (last 5), "Inspect scene" buttons from `ui.run_inspections` (call focus()), Awards table (`rows[label, points]`), Manual adjustment, TOTAL, `score_fields`, manual adjustment input+Apply (if `ui.manual_adjustment`), `score_note` | P/run_panel.cpp:31-228 | `Run` (sim.run) + task doc `ui` |
| `simulation` (tool, popup "Simulation settings", 340 px) | message, "Selected speed: %.2fx", Speed input + Apply, Pause/Resume (preserves resume rate), 1x, "Speed range: >0 to N x", Sync sim ("move sim to estimated pose, keep velocities"), Reset sim ("start pose, at rest, thrusters cleared, re-seed estimator"), operation message | P/simulation_panel.cpp:15-95 | `Simulation` (ros.simulation_rate) |
| `pose_gizmo` overlay (title "Robot controls") | On Position-mode + enabled + fresh + not blocked: 3 body-axis arrows (X/Y/Z), centre square (XY-plane drag), 3 rings (roll/pitch/yaw, continuous multi-turn), size `size_metres` (.3 m), hit `hit_pixels` (20); yellow highlight, tooltip "<handle> / drag commands immediately / Esc restores start"; screen ray captured at drag start; Esc restores start pose; consumes input so orbit does not move | P/pose_gizmo.cpp:32-266, H/panels/pose_math.hpp | `Motion` provider |

Ownership link (`ownership: [{motion, autonomy}]`): running tree => motion `blocked` ("Autonomy owns motion"), gizmo/targets disabled; Kill also stops the tree; Start requires motion enabled (P/composition.cpp:162-200, uwrt_autonomy.cpp:29-53).
Motion safety semantics (P/ros_runtime.cpp:70-200): enable = publish KillSwitchReport(asserting_kill=false) then mode request `setTeleop(data=false)`; auto-kill if pose stale (>1 s), UI not drawn for >.75 s (`ui_timeout`), request >3 s, another sender on kill topic; heartbeat `report()` every 50 ms while a session exists; kill on destruction; setpoint TF published while a command exists (1.10).

### 1.5 Camera cards (right sidebar)

| Feature | Detail | Source |
|---|---|---|
| Card per sim camera | title `01  FORWARD CAMERA` if name==`ffc` else `02  DOWNWARD CAMERA` (hardcoded), RGB/DEPTH toggle button, `ZED X MINI / WxH` (hardcoded), preview WxH, live texture, "Awaiting vehicle pose" overlay when not ready, status `CONNECTED / NO SENSOR OUTPUT / PREVIEW ONLY`, measured Hz, `METRES / RECTIFIED RGB`, calibration tooltip | M:1412-1448 |
| Side preview rendering | side cards render at <= `camera_preview_width` (480); full sensor resolution only when a topic is subscribed / view promoted / screenshot | M:712-720 |
| Depth preview | depth texture (colourised metres) after noise model; "Show both depth maps" button | M:1013-1035, 1531 |
| Mini course map | `Expand` button; scaled pool rect, 5 m grid, trail, cyan landmark dots (`gate, slalom_front, slalom_back, torpedo, bin, table, octagon` hardcoded list; `slalom_back` label hidden, `slalom_front` shown as "slalom"), vehicle dot + 1.3 m heading tick + robot name | M:1594-1660 |

### 1.6 "Scene settings" window (tabs) (M:2306-2378)

| Item | Detail | Source |
|---|---|---|
| Mechanism buttons | one button per `task.mechanism_controls[]` (`label`,`topic`,`value`), publishes `std_msgs/Bool`; disabled in demo (Talos: none defined, `robot.yaml` has no `controls`) | M:477-483, 2310-2319 |
| Lighting tab | Calibration board checkbox (`look.tag`), Caustics 0-1, Surface, Shadows, Lighting Indoor/Outdoor (presets bright/ambient 1.0/.9 indoor, 1.4/.6 outdoor), Brightness 0-4, Ambient 0-2, outdoor: Sun azimuth 0-360, Elevation 5-89, Glare 0-2 | M:2323-2359 |
| Water appearance tab | Water tint colour picker; presets Clear blue / Pool / Green-murky; sliders Haze/scattering 0-1, Distance strength 0-5, Distance exponent .25-3, Clear distance 0-10 m, R/G/B absorption 0-1. Applied via `set_parameters_atomically` on the viewer node's own params `water.*` (affects both sensor cameras) | M:1450-1505, 195-260 |
| Depth sensor tab | Noise enabled, Show both depth maps, Reset noise, **Point cloud in pool view** (Off/ffc/dfc/All), cloud px size 1-8, tint 0-1 (orange ffc / cyan dfc), sliders Base sigma, Range coeff, Range exponent, Bias, Min/Max range, Missing pixels, Range/Edge dropout, Outliers, Spatial correlation, Patch size, sigma readout at 1/3/6 m. Params `depth_noise`, `depth_model.*` | M:1507-1592 |

### 1.7 Scene / robot visuals (see section 5 for sources)

Pool (floor, 4 walls, deck, coping), animated water surface + underwater fog/absorption/scattering/caustics, procedural sun/indoor light + 4096^2 shadow map, HDR bloom post, surface reflections (observer only), calibration board, mapped course meshes, hanging octagon, crates, claw (4 moving parts), magnet-light targets (LED red/green), robot magnet, payload launcher + loaded/released projectiles, thruster rotors spinning, status LED bars, point-cloud overlay.

### 1.8 Overlays in the pool view (screen-space, drawn over main texture)

| Overlay | Behaviour | Source | Data |
|---|---|---|---|
| Detections | `visualization_msgs/Marker` CUBE (filled quad+outline, sx x sy) and ARROW (rviz-style shaft to 77 %, head) in marker colour; placed ONCE at acquisition pose: render-pose history (`Camera::renders`, matched by exact stamp ns, kept 600) else TF at header stamp; never follows later TF; markers expire by `lifetime` even when hidden; DELETE/DELETEALL honoured | M:1729-1837, H/detection_pose.hpp | topic `yolo_orientation/visualization_marker_array` |
| MPC path | subscribed only while checkbox on; poses re-rooted: `truth(map<-sim base) * est(base<-path frame @stamp) * pose`; orange polyline + dots + heading tick every 5th stage; cleared if >0.5 s old | M:1839-1909 | `controller/mpc/predicted_path`, TFs |
| TF axes | RGB axes length `tf_axis_length` (.02-1 m slider), frame names with shadow, caption "ROS TF in map: N frames, M unavailable", extra line "ROS vs sim: X cm / Y deg; forward Z cm" (estimate vs truth base_link at common stamp) | M:1981-2020, 1662-1727 | all TF |
| Point cloud (Depth tab) | GL points from the rendered camera at true camera pose | R `pointCloud`, M:1013-1030 | camera renders |
| Landmark labels | see toolbar Labels | M:2282 | `landmarks` |

### 1.9 TF popup

Show TF frames, Frame names, Axis length; frame tree table (Frame | Only this | With children (tri-state)); Show all / Hide all; context menu Show/Hide branch; "(unavailable)" tag for frames that cannot reach `fixed_frame`; TF list built from `Buffer::_getFrameStrings/_getParent`; `TfTree` in H/tf_tree.hpp. Demo shows configured `base_link, origin, torpedo_link, torpedo_{0,1}_link, droppers_link, magnet_link` (M:262-278). Source M:1911-1979.

### 1.10 Modes

| Mode | Behaviour | Source |
|---|---|---|
| `demo:=true` (scene preview) | no ROS sensor/TF output, no ROS providers (panels validate only), fixed preview vehicle pose (3,-2,-.75, yaw -.14), all payloads shown in mounts, landmark picker | M:140, 585-591, 1188, P/composition.cpp:135 |
| `headless:=true` | hidden window (GL still required); renders only if a subscriber/screenshot; sleeps to `render_rate` | M:744 (loop) |
| `exit_after_frames`, `screenshot_path` | render N frames, save full-window PNG + one PNG per camera (`<stem>-<cam>.png`), exit; throws on ImGui validation errors | M:2395-2430, 782-788 |
| `show_scorecard` | opens run window at start | M:142 |
| `show_tf`, `detections`, `mpc_path`, `depth_preview`, `point_cloud_overlay`, `profile` | initial state of toggles / timing logs | M:137-147 |
| `operator_panels` auto/true/false | auto disables panels when `with_rviz` | launch:96-103 |

Setpoint telemetry: while a Motion command exists (any owner) `RosMotion::tick` broadcasts TF `map -> ghost/base_link` (option `setpoint_frame`) every 50 ms (P/ros_runtime.cpp:171-190).

### 1.11 Status lights (LED bars) and thruster visuals

| Feature | Detail | Source |
|---|---|---|
| Status LEDs | 3 emissive boxes ("Status light/<id>", HDR radiance 240) under port hull; mode: Solid, SlowFlash (off first 1 s of 2 s, ROS clock), FastFlash (off first .25 s of .5 s), Breath (sin, 3 s), SINGLETON_FLASH => `flash_duration` .15 s pulse then resume; `target` bitmask AND `target_mask` (ALU=2 in config; TARGET_ALL=3); LedCommand target > 3 rejected; ColorRGBA alternative input (solid, rgb*clamp(a)) | H/status_lights.hpp, M:428-470, R:508-514 |
| Thruster rotors | 8 rotor meshes (`rotors/*.glb`) rotate about pivot/axis; RPM = signed polynomial of realized force (`force_to_rpm` forward/reverse, tanh & |F|^.25 terms), deadband .01 N, timeout .5 s then stop, angle integrated on ROS clock, resets on clock rewind, `speed_scale`, `direction` +-1 | H/thruster_visuals.hpp, M:422-428 |
| Claw | left/right carriers+pads translate +-Y by joint values (m), static assembly; jaw gap readout = `min_gap + j0 + j1` (mm) in run window | R:652-664, sim_run.cpp:65-73 |
| Magnet lights | `<target>/LEDs` tint green (.002,1,.004) / red (1,.001,.002), radiance `magnet_lights.led_radiance` | R:708, M:516-526 |
| Task props | table props (pill, bandage, nut_and_bolt, plug) as meshes, world or attached to `talos/base_link` when held | M:531-548 |
| Payloads | loaded rounds drawn at launcher slots (`payloadMounts`), released rounds at world pose (msg scale) each frame from a full snapshot (occupancy + free) | M:555-578, H/payload_mounts.hpp |

Tests that define expected behaviour (reusable as parity tests): `test/` camera_geometry, depth_noise, detection_pose, frustum (culling only), gizmo_input, operator_panels, panel_composition, panel_ros, status_lights, tf_tree, thruster_visuals, viewer_input (C++); panel_ui_smoke.py (X11 UI), ros_detection_lifetime_smoke, ros_magnet_smoke, ros_score_memory_smoke (YAML retention), ros_status_lights_smoke, ros_thruster_visuals_smoke (Python). Camera-only: camera_processor, ros_camera_*.
Note: there are NO camera-frustum overlays (Frustum = view culling only); no `T` key (stale comment in `T/config/scene.yaml:9`; the board is the Lighting-tab checkbox).

## 2. ROS interfaces used by the old viewer

Names as seen by the code (relative names resolve under `/talos`). "cfg" = key in yaml; "hard" = hardcoded. Viewer nodes: `pool_viewer` and `viewer_panels`, both ns `/talos`.

### 2.1 Main node `pool_viewer` (M)

| Name | Type | Dir | Feature | Name from |
|---|---|---|---|---|
| TF `map`(param `fixed_frame`) -> `simulator/talos/base_link` | tf2 | read | truth pose, follow, run-state pill | hard prefix `simulator/` + param `robot`, M:964 |
| TF `map -> simulator/talos/<cam>_camera_link` | tf2 | read | physics-owned camera pose (only cams with `truth_tf_owner: physics`, i.e. ffc) | hard, M:685 |
| TF `map -> <any frame>` incl. `talos/base_link`, `world`, `odom` | tf2 | read | TF overlay/tree, ROS-vs-sim diff, MPC re-root, detections fallback | hard |
| TF static `simulator/talos/base_link -> simulator/talos/{dfc}_camera_link -> ...optical`, `simulator/talos/origin` | tf2 static | write (non-demo) | truth camera frames for DFC + CAD origin marker | hard, M:361-392 |
| TF static `talos/<cam>_camera_link -> talos/<cam>_left_camera_optical_frame` | tf2 static | write | only if `publish_camera_optical_tf` and no `zed_wrapper` | param, launch:14-20 |
| `simulator/actual_thruster_forces` | std_msgs/Float32MultiArray | sub | rotor animation | cfg `thruster_visuals.yaml: topic` |
| `command/led` | riptide_msgs2/LedCommand | sub | LED bars | cfg `status_lights.yaml: input.topic` (`input.type` selects LedCommand or std_msgs/ColorRGBA) |
| `simulator/magnet_lights` | visualization_msgs/MarkerArray | sub | bin LED colours | hard M:517 |
| `simulator/claw_joints` | std_msgs/Float64MultiArray (2) | sub | claw jaws | hard M:528 |
| `simulator/task_objects` | MarkerArray | sub | table props poses | hard M:533 |
| `simulator/projectiles` | MarkerArray | sub | torpedo/dropper visuals | hard M:555 |
| `yolo_orientation/visualization_marker_array` | MarkerArray | sub | Detections overlay | hard M:552 |
| `controller/mpc/predicted_path` | nav_msgs/Path | sub (only while shown) | MPC overlay | hard M:1845 |
| `<mechanism_controls[].topic>` | std_msgs/Bool | pub | Scene-settings buttons | cfg task doc `mechanism_controls` (from `robot.yaml: controls`) |
| params on `/talos/pool_viewer`: `water.{tint,absorption,scattering,distance_scale,distance_power,clear_distance}`, `depth_noise`, `depth_model.{enabled,base_sigma,range_exponent,min_range,max_range,bias,dropout,range_dropout,edge_dropout,outliers,correlation,patch_size}` | rcl params (settable at runtime) | set/serve | Water + Depth tabs (also `ros2 param set`) | hard M:160-260 |
| `use_sim_time` | param | - | all stamps/lights/rotors on `/clock` | launch |
| camera publishers `<topic_root>/{rgb,left}/image_rect_color[/compressed]`, `depth/depth_registered`, `*/camera_info`, `point_cloud/cloud_registered` | Image/Compressed/CameraInfo/PointCloud2 | pub | camera-faker role: DROP | `sim_cameras[].topic_root` |

### 2.2 Panel node `viewer_panels` (P/)

| Name | Type | Dir | Provider / feature | Name from (cfg key in `robots/talos/config/viewer.yaml`) |
|---|---|---|---|---|
| `controller/linear`, `controller/angular` | riptide_msgs2/ControllerCommand (mode POSITION/FEEDFORWARD/DISABLED; `setpoint_vect` / `setpoint_quat`) | pub + sub (sub only observes autonomy) | uwrt.motion, gizmo, Target table | `linear_topic`, `angular_topic` |
| `command/software_kill` | riptide_msgs2/KillSwitchReport (`kill_switch_id`=1, `sender_id`=`viewer_<host>_<pid>_<n>`, `switch_asserting_kill`, `switch_needs_update`=true) | pub + sub (competitor detect) | Kill/Enable | `kill_topic`, `kill_switch_id` |
| `state/kill` | std_msgs/Bool (SensorDataQoS) | sub | "Robot: killed/enabled" | `kill_state_topic` |
| `setTeleop` | std_srvs/SetBool (request data=false) | client | mode request before Position/FF | `mode_service` |
| TF `map -> talos/base_link`, `world -> map`(command frame `world`) | tf2 | read | Actual pose, command frame | `base_frame`, `command_frame` |
| TF `map -> ghost/base_link` | tf2 | write | setpoint telemetry | `setpoint_frame` |
| `autonomy/run_tree` | riptide_msgs2/action/ExecuteTree (goal.tree; result.returncode==2 success; feedback.stack.stack) | action client | Autonomy Start/Stop | `action` |
| `autonomy/run_tree/_action/status` | action_msgs/GoalStatusArray (transient_local) | sub | external-run detection | derived from `action` |
| `autonomy/list_trees` | riptide_msgs2/srv/ListTrees | client | tree combo/Refresh | `list_service` |
| `autonomy/tree_stack` | riptide_msgs2/TreeStack | sub | stack list | `stack_topic` |
| `map/model_tf` | chameleon_tf_msgs/action/ModelFrame (monitor_parent, monitor_child, samples; feedback sample_count; result success/err_msg) | action client | Tag cal | `calibration_action` |
| `mapping/reset_mapping` | std_srvs/Trigger | client | Reset mapping | `reset_service` |
| `mapping_target` | riptide_msgs2/srv/MappingTarget (target_info.target_object, lock_map) | client | Set target | `target_service` |
| `state/mapping` | riptide_msgs2/MappingTargetInfo | sub | Current target/lock | `status_topic` |
| `command/actuator/arm` (Bool, toggle), `.../torpedo` (Empty), `.../dropper` (Empty), `.../claw` (Bool true/false), `.../claw_move_s` (Float32 0), `.../notify_reload` (Empty) | topics | pub | Actuators buttons | `commands[].topic/type/value/toggle_armed` |
| `state/actuator/status` | riptide_msgs2/ActuatorStatus (uses `actuators_armed`, `torpedo_available_count`, `dropper_available_count`) | sub (SensorDataQoS) | Actuators | `status_topic` |
| `simulator/run_command` | std_msgs/String JSON | pub | Start/Stop/Adjustment/`actions[]` | `command_topic` |
| `simulator/reset_tasks` | std_msgs/Empty | pub | "Reset run & tasks" | `reset_topic` |
| `simulator/run_score` | std_msgs/String JSON | sub | scorecard, viewport readout | `score_topic` (required) |
| `simulator/task_score` | String JSON | sub | "Task status" text | `task_score_topic` |
| `simulator/task_events` | String JSON | sub | "Recent events" (last 5) | `events_topic` |
| `simulator/magnet_lights` | MarkerArray | sub | "Magnet targets" list | `lights_topic` |
| `simulator/claw_joints` | Float64MultiArray | sub | "Jaw gap" reading | `joints_topic` |
| `/talos/physics_simulator/{get,set}_parameters` param `real_time_factor` (double, 0 = pause, resume rate remembered, polled 2 Hz) | rcl_interfaces | client | Simulation speed/pause | `node`, `parameter`, `max_rate` (10) |
| `sync_sim_to_estimate`, `reset_sim_to_start` | std_srvs/Trigger | client | Sync sim / Reset sim | `sync_service`, `reset_service` |
| generic `ros.pose` alternative (not used by Talos): `control/pose` PoseStamped pub, `control/enable` SetBool, `control/enabled` Bool | | | example only | `standard_panels.example.yaml` |

## 3. Provided by whom now

Legend: BRIDGE = new bridge (`bridge.yaml`, `robotics_platform_ros`); STACK = unchanged UWRT nodes; MISSING = only the old simulator/viewer provided it.

| Interface | Provider now | Evidence / gap |
|---|---|---|
| TF `map -> simulator/talos/base_link` (100 Hz, truth) | BRIDGE | `tf.publish` (bridge.yaml:76) |
| TF `simulator/talos/{ffc,dfc}_camera_link`, `..._optical_frame`, `simulator/talos/origin` | MISSING | old: PS:1229-1284 (ffc, physics) + M:361-392 (dfc/origin). Bridge publishes only truth base_link; new viewer knows its own render poses, so only needed if TF tree must show them |
| TF `talos/<cam>_camera_link -> optical` (+right cam) | BRIDGE static | bridge.yaml:88-92 |
| TF `map/world/odom/talos/*`, estimate | STACK (EKF, chameleon_tf, robot_state_publisher, navigation.launch) | bridge `tf.lookup` |
| `/clock` | BRIDGE | bridge.yaml `clock` (500 Hz) |
| `command/software_kill`, `state/kill` | BRIDGE (kill id 1 only) / STACK (`electrical_monitor`, `firmware_monitor` also publish `state/kill`) | streams `software_kill`, `firmware_kill` |
| `controller/linear|angular`, `setTeleop`, `controller/mpc/predicted_path` | STACK (controller_overseer.py:420, mpc_controller_node.cpp:183/210) | none needed from bridge |
| `autonomy/run_tree`, `list_trees`, `tree_stack` | STACK (DoTask.cpp:95, UWRTLogger.hpp:27) | |
| `map/model_tf`, `mapping/reset_mapping`, `mapping_target`, `state/mapping` | STACK (chameleon_tf mobile_tf.cpp, riptide_mapping2 mapping.py) | |
| `yolo_orientation/visualization_marker_array` | STACK (tensor_detector yolo_orientation.py) | |
| `command/led` | STACK publishers (HeadlessInterface.py:81, mapping.py, pressure_monitor.py, autonomy_lib.hpp:26) | bridge has no `command/led` stream; needed only if LED rendering lives in the bridge/sim process rather than a ROS-subscribing viewer |
| `command/actuator/{arm,torpedo,dropper,claw,claw_move_s,notify_reload}` topics + services | BRIDGE | streams `*_topic`, services `arm/fire_torpedo/fire_dropper/reload/claw` |
| `state/actuator/{status,busy,cmd_status}` | BRIDGE (50 Hz) | streams `actuator_*` |
| `sync_sim_to_estimate`, `reset_sim_to_start`, `set_sim_pose` | BRIDGE | services |
| `simulator/reset_tasks` **Trigger service** | BRIDGE (new) | old viewer publishes **std_msgs/Empty topic** of the same name (TN:229-231 supports both) -> viewer must call the service or bridge must add the topic |
| `simulator/reset_scenario` (new full reset) | BRIDGE (new) | no old equivalent, viewer may use |
| `simulator/ground_truth` Odometry | BRIDGE | stream `ground_truth` |
| `simulator/actual_thruster_forces` | MISSING | see 4.5 |
| `simulator/run_command`, `run_score`, `task_score`, `task_events` | MISSING | tasks run in-session; `execution.json` says "operator run commands are not bridged"; results only written at exit (`tasks.json`, `__main__.py:165-172`) |
| `simulator/magnet_lights`, `claw_joints`, `task_objects`, `projectiles` | MISSING | mechanism/task state is native; no stream |
| `real_time_factor` parameter (pause / speed) | MISSING | bridge reads `clock.real_time_factor: 1.0` from yaml once (`core.py:183`, `node.py:172`); no ROS parameter, node is `robotics_platform_bridge` not `physics_simulator` |
| `simulator/time` (Float64), `simulator/state` (Pose), `simulator/collisionMarkers`, `simulator/enable`, `acoustics/delta_t` (Vector3Stamped) | MISSING (not used by viewer; `acoustics/delta_t` is consumed by the acoustics stack, PS:110) | old PS:104-112 |
| `simulator/reset_magnet_lights`, `simulator/reset_table` services | MISSING (not used by viewer) | TN:232-233 |
| Camera topics `ffc|dfc/zed_node/...` | BRIDGE (compressed rgb, camera_info, depth only; no raw rgb, `left/*`, point cloud) | out of scope here |
| viewer params `water.*`, `depth_noise`, `depth_model.*` | MISSING as ROS params | the bridge camera products take noise/water from packs; a runtime editing path from the new viewer to camera renders must be designed |

## 4. Missing interfaces: exact old formats

QoS for all: old publishers use default QoS depth 10 (reliable, volatile); the viewer subscribes depth 10 default.

### 4.1 `simulator/run_score` (std_msgs/String, JSON) - TN:612-621, SC:245-266, published from a 50 Hz timer (TN:237)
```
{"running": bool, "elapsed": float_s, "scoring_open": bool, "ended_reason": str, "intended_role": str, "role": str,
 "target_class": str, "gate_passed": bool, "total": float, "adjustment": float,
 "rows": [{"key": str, "label": str, "points": num}, ...],   // one per score key, order of RunScore.points
 "basket_count": int, "pinger": {...}, "time_bonus_eligible": bool,
 "message": str,            // run_message e.g. "Run started; pass the gate first", "Run stopped", "Command rejected: ..."
 "ui": {...}, "year": "2026", "config_id": ..., "runtime": {...}}
```
Viewer requires keys `running, elapsed, total` (else "Invalid run score"), `rows[].label/points` numeric, finite `total/elapsed/adjustment`; stale after `status_timeout` 1 s => controls disabled ("Run tracking unavailable / stale"). `elapsed` = sim time since start while running else frozen (SC:246). Row keys/labels (SC:14-33): gate, slalom_front, slalom_middle, slalom_back, bins, lights, torpedoes, sequence, distance, surface, facing, objects_surface, objects_drop, baskets, basket_count, home, pinger_first, pinger_second. Extra scalar fields displayed through `ui.status_fields` (`intended_role`, `role`) and `ui.score_fields` (`basket_count`, `time_bonus_eligible`) plus `message`, `ended_reason`.
New runtime has `task_runtime.snapshot()` (`python/src/robotics_platform/task_runtime.py:250`: `run{running,ended,started_ns,options,seed}, scores{}, history[], environment, latched`) but no per-row labels, elapsed, total or adjustment.

### 4.2 `simulator/run_command` (std_msgs/String, JSON) - handled TN:302-345
`{"action":"start","role":"repair|rescue","heading_coin":bool,"role_coin":bool}` (viewer builds JSON from `ui.run_options`; start = reset_tasks + `run.start(now, role, heading_coin, role_coin)`, rejected if running or no judge), `{"action":"stop"}`, `{"action":"adjustment","points":<finite float>}`, `{"action":"pinger_select","task":..,"random":bool}`, `{"action":"pinger_switch"}` (only reachable via `ui.actions`). Errors set `run_message` "Command rejected: <e>". Run options come from `T/competition.yaml ui.run_options` (role choice repair/rescue, heading_coin, role_coin; defaults repair,true,true) with scenario `run_defaults` override (`C/riptide_sim_config/profiles.py:360-364`). `simulator/reset_tasks` Empty = `reset_tasks()` (TN:270-291): clears payloads, reloads, zeroes score dict, claw/magnet/run/judge reset, publishes event `{"kind":"tasks","result":"reset","target":"","time":t}`.

### 4.3 `simulator/task_score` (String JSON) - TN:741-742
`{"success":int,"wrong_target":int,"blocked":int,"miss":int}` (payload/claw outcome counters, key set TN:212); shown verbatim in "Task status".

### 4.4 `simulator/task_events` (String JSON) - TN:285, 405-415, 460-463, 490-493
Payload/claw event: `{"id": <int|prop key>, "kind": "torpedo|dropper|claw", "result": "released|success|wrong_target|blocked|miss", "target": str, "time": sim_s, "slot": int}` (claw events omit `slot`). Magnet: `{"kind":"magnet","result":"activated","target":"magnet_target1","time":t}`. Reset: kind `tasks`. Viewer keeps last 5 as `kind / result / target`; `tasks/reset` clears history (P/sim_run.cpp:28-43).

### 4.5 `simulator/actual_thruster_forces` (Float32MultiArray) - PS:1259-1262
`data[i]` = `robot.realizedThrusters()` in newtons after thruster dynamics, in `talos.yaml` thruster order `[VUS, VUP, HUS, HUP, HLS, HLP, VLS, VLP]` (= `input_index` 0-7 in `models/talos3/thrusters.yaml`), no layout dims, published at `STATE_PUB_TIME` = 0.01 s sim time (`C/include/c_simulator/settings.h:8`). Viewer rejects wrong length/non-finite, zeroes after `timeout` .5 s.

### 4.6 `simulator/magnet_lights` (MarkerArray) - TN:622-646 (every 50 ms)
One marker per target (`magnet_target1`, `magnet_target2`): `header.frame_id="map"`, `ns=<target>`, `id=index`, `type=SPHERE`, `action=ADD`, pose = face pose of the bin light (map frame), scale (.002,.044,.044), colour `r=1,g=0` (red) or `r=0,g=1` (green), `a=1`. Viewer: green iff `color.g > color.r`; DELETE/DELETEALL also honoured by the run panel. Trigger logic: `MagnetLights.step` activates when robot magnet within `trigger_distance` .1524 m for `activation_time` .5 s (config `T/config/tasks.yaml`); event `magnet/activated`.

### 4.7 `simulator/claw_joints` (Float64MultiArray) - TN:668-675
`data=[left_m, right_m]` = `claw.joints()`: jaw travel offsets from the closed pose (viewer translates left carrier `+Y*left`, right `-Y*right`; gap = `min_gap`(.0012, `claw.min_gap`) + left + right). Length must be 2. Published 50 Hz while a claw exists. Also consumed by physics (`setJaws`, PS:128).

### 4.8 `simulator/task_objects` (MarkerArray) - TN:676-705
Per claw prop (`pill, bandage, nut_and_bolt, plug`): `header.frame_id="map"` (or `talos/base_link` with pose relative to base_link while held), `ns=<prop key>`, `id=index`, `type=MESH_RESOURCE`, `mesh_resource=package://riptide_meshes/meshes/<props.<key>.mesh>/model.dae`, `mesh_use_embedded_materials=true`, scale 1, `a=1`, `action=ADD`; viewer keys objects `"<ns>_frame"`. Also consumed by physics (PS:133-152; base_link offset added when attached).

### 4.9 `simulator/projectiles` (MarkerArray) - TN:743-798
Each publish: first marker `DELETEALL`, then one MESH_RESOURCE marker per in-flight payload (`ns="torpedo"|"dropper"`, `id`=payload id) and one per still-loaded round (`ns="torpedo_loaded"|"dropper_loaded"`, `id`=slot index of fired-count..count), all `frame_id="map"`, `mesh_resource=package://camera_faker/models/payloads/projectile.obj`, pose = position + orientation (rotation matrix -> quaternion, keeps fin roll), scale = (length, 2*radius, 2*radius) from `T/config/tasks.yaml torpedo|dropper`, colour (.65,.025,.035,1). Viewer: loaded set = `(ns,id)` matching a mount key; everything else drawn at world pose.

### 4.10 `real_time_factor` parameter on `/talos/physics_simulator` (double) - PS:69, 1342-1352, applied every physics loop iteration (PS:275)
`>= 0` accepted (invalid => warn, ignored); 0 pauses simulated time (clock stops, `/clock` stops); viewer max 10 (`ros.simulation_rate.max_rate`). New bridge fixes it to yaml `clock.real_time_factor` (1.0) with no runtime path; a pause/speed mechanism (parameter or service) is required to keep the Simulation panel.

### 4.11 LED state
Not simulator-produced: the stack publishes `riptide_msgs2/LedCommand{red,green,blue:uint8, mode:{0 SOLID,1 SLOW_FLASH,2 FAST_FLASH,3 BREATH,4 SINGLETON_FLASH}, target:{0 NONE,1 CCB,2 ALU,3 ALL}}` on `command/led`; the old viewer rendered it. Only the missing piece is a consumer (`docs/INDICATORS.md` in this repo covers native indicators).

## 5. Composition / configuration mechanism

Launch: `launch/pool_viewer.launch.py` -> `riptide_sim_config.launching.run()` resolves robot+year+scenario into a folder (`selection.yaml`, `vehicle.yaml`, `task.yaml`, `mapping.yaml`, `markers.yaml`, `scene.yaml`) and passes paths as node params (launch:130-190). Panel selection order: launch arg `panels_config` > robot profile `viewer.panels_config` (`C/robots/talos/robot.yaml:34`); `tools_config` default `camera_faker/config/viewer_tools.yaml`; `operator_panels` gate. Composition schema (P/composition.cpp:56-135, strict `keys()` validation, unknown key = exception, `{namespace}` and `{fixed_frame}` are the only substitutions, others throw):

| Top-level key | Meaning |
|---|---|
| `schema_version` | must be 1 |
| `sidebar_width` (300-600 px) xor `sidebar_width_fraction` (0-1, default .29), `sidebar_visible` (default true) | layout |
| `providers: {id: {type, options}}` | ROS-facing state sources; instantiated once each (one `RosRuntime`) |
| `panels: [{id, type, provider, title?, visible?, open?, options?}]` | sidebar sections |
| `tools: [{..., slot: settings|overlays}]` | toolbar buttons/popups/windows outside the sidebar |
| `overlays: [{id, type, provider, title?, visible?, options?}]` | viewport overlays (checkbox appears in the motion panel) |
| `ownership: [{motion, autonomy}]` | provider ids; blocks manual motion while a tree runs |

Provider types (kind must match panel type): `uwrt.motion` {base_frame, command_frame, setpoint_frame, linear_topic, angular_topic, kill_topic, kill_state_topic, mode_service, kill_switch_id (1..255), sender_prefix, pose_timeout 1, ui_timeout .75, request_timeout 3, heartbeat_period .05}; `ros.pose` {base_frame, command_frame, setpoint_frame, pose_topic, enable_service, enabled_topic, timeouts}; `uwrt.autonomy` {action, list_service, stack_topic, request_timeout 3, stack_timeout 5}; `uwrt.mapping` {calibration_action, reset_service, target_service, status_topic, request_timeout 3, calibration_timeout 60, status_timeout 2}; `uwrt.actuators` {status_topic, commands[{id,label,armed_label,topic,type bool|float32|empty,value,requires_armed,toggle_armed}], status_timeout 1}; `sim.run` {command_topic, reset_topic, score_topic, task_score_topic, events_topic, lights_topic, joints_topic, status_timeout 1, profile}; `ros.simulation_rate` {node, parameter, max_rate, request_timeout, sync_service, reset_service}. Panel types: `motion` {dive_z, dive_max_depth_z}, `autonomy`, `mapping` {parent_frame, tag_frame, samples}, `actuators`, `run` {profile}, `simulation`. Overlay: `pose_gizmo` {size_metres, hit_pixels}. Registry: P/panel_registry.cpp, P/ros_runtime.cpp:53-60.
`profile: task` binds to document `context.documents["task"]` = resolved `task.yaml`: `ui{title, run_options, focus, demo_targets, previews, run_inspections, status_fields, score_fields, manual_adjustment, score_note, actions}`, `scoring_enabled`, `claw`, `magnet_lights`, `torpedo`, `dropper`, `crate`, `octagon`, `world`, `mechanism_controls`, `equipment` (M:463-508).

Config files:

| File | Keys / role |
|---|---|
| `C/robots/talos/config/viewer.yaml` | Talos providers/panels/overlay/ownership (topics in section 2.2) |
| `camera_faker/config/viewer_tools.yaml` | tools `run` (provider `sim.run`, profile task) + `simulation` (slot settings) |
| `camera_faker/config/empty_panels.yaml`, `standard_panels.example.yaml` | empty / generic `ros.pose` example |
| `C/robots/talos/robot.yaml` | `model`, `viewer.{panels_config, thruster_visuals_config, status_lights_config, payload_model, launcher_model, claw_model, camera_settings}`, `cameras[]{name,config,topic_root,truth_tf_owner}`, `capabilities`, `controls` (absent) |
| `C/robots/talos/config/status_lights.yaml` | `input{type,topic}`, `flash_duration`, `lights[{id,target_mask,pose[6],size[3],radiance}]` |
| `camera_faker/models/talos3/thrusters.yaml` | `topic`, `timeout`, `force_deadband`, `speed_scale`, `force_to_rpm{forward,reverse}[4]`, `rotors[{id,input_index,pivot,axis,direction,mesh,source_geometries}]` |
| `camera_faker/config/cameras.yaml` | camera-faker/depth/water startup (`camera_compute`, `ffc|dfc.resolution_scale`, `depth_noise`, `depth_model.*`, `water.*`); depth+water portions are viewer-side visual settings |
| `riptide_descriptions/config/talos.yaml` (vehicle) | `base_link`, `cameras[]`, `torpedoes{pose,baseline}`, `droppers{pose}`, `claw{pose}`, `magnet{pose}`, `thrusters[]` (count for rotor validation) |
| `T/competition.yaml` | task `ui` block (focus, previews, run options, inspections), `required_frames`, `mesh_root` |
| `T/config/{tasks,markers,mapping,scene}.yaml` | task geometry, marker/mesh placement, course frames (`init_data`, `map_origin_pool`), scene entities/`april_tag` |
| `C/worlds/*.yaml` | `world{id,length,width,depth,water_level,deck_height,...}` |

Viewer node params (launch args): `robot`, `fixed_frame`, `robot_model`, `payload_model`, `launcher_model`, `claw_model`, `panels_config`, `operator_panels`, `tools_config`, `status_lights_config`, `thruster_visuals_config`, `task_config`, `depth_preview`, `show_tf`, `show_scorecard`, `lighting.{profile,brightness,ambient,sun_azimuth,sun_elevation,glare}` (launch `lighting` indoor/outdoor, default outdoor), `demo`, `demo_task`, `initial_focus`, `headless`, `use_sim_time`, `camera_*`, `render_rate`, `profile`, `exit_after_frames`, `screenshot_path`, `detections`, `mpc_path`, `point_cloud.{enabled,rate,stride,overlay}`, `vehicle_config`, `shader_folder`, `texture_folder`, `riptide_mesh_folder`, `marker_config`, `mapping_config`, `scene_config`.

## 6. Scene content and origin

| Content | Where drawn | Source data |
|---|---|---|
| Pool floor, near/far/end walls, deck slabs, coping | R:336-360 (procedural boxes) | `world` (`length` 50, `width` 22.86, `depth` 2.1336, `water_level`, `deck_height`) from `C/worlds/competition_pool.yaml` via `scene.yaml world:`; walls hideable (poolBoundary) |
| Water surface + fog/absorption/scatter/caustics/glare, sun or indoor light, shadow map 4096^2 (33 m ortho), bloom, surface reflection 640x400 | R:361-366, 730-1000, `shaders/pool/*.{vert,frag}` | `Look` params (Lighting/Water tabs) |
| Pool-to-map transform | R:398-403 | mapping yaml `/**/zed_faker.ros__parameters {config_frame: tag, map_origin_pool [x,y,yaw_deg]}` |
| Course landmarks (frames) `landmarks[name]` | R:405-430 | mapping `/<robot>/riptide_mapping2.init_data` (`parent`, `pose{x,y,z,yaw}`), resolved recursively; `_frame` suffix stripped |
| Course meshes | R:430-465 | `markers.yaml /**/marker_publisher.markers[].{mesh,frame,pose,scale}`; mesh file `<mesh_root>/<name>/model.dae` (`riptide_meshes` package: gate, gate_repair, gate_rescue, slalom, torpedo, bin, bin_vinyl, bin_magnet, table, table_*, helmet, warning, compass, hammer_and_wrench, buoy, sos, octagon signs...). `cube/sphere/arrow` skipped. Special-cased by literal mesh name: `torpedo` (panel holes cut from `task.torpedo.holes` uv/radius_uv, backing sheet split, R:271-306, max 4), `bin_vinyl` (texture `Task3_Blood_Fixed.png` / `Task3_Fire_Fixed.png` chosen from `init_data.<frame>.class` blood|fire, R:455-462), `bin_magnet` (adds `housing/cover/LEDs` GLB from `models/magnet_lights` with `magnet_lights.face_pose`, red/green from `task.magnet_lights.targets`), `table_pill` and `table_nut_and_bolt` (unmapped-UV faces recoloured, R:247-268) |
| Scene entities | R:466-477 | `scene.yaml entities[{id, frame, pose, size, mesh|color}]` (2026 file has none) |
| Octagon PVC ring + hanger tubes for compass/hammer_and_wrench/buoy/sos signs | R:478-481, 543-586 (procedural) | `task.octagon{apothem,pipe_radius,surface_z}`; literal sign keys |
| CleverMade crates (lattice + liner) at `bin_vinyl1..4` | R:482, 587-650 | `task.crate{outer_width,inner_width,outer_height,base_thickness,liner_thickness,liner_height_fraction}` |
| Calibration board ("April Tag.jpg", 0.61x0.91 m at (.002,0,-.345) map) toggle | R:519-528, `textures/objects/April Tag.jpg` | `scene.yaml april_tag.visible`, Lighting tab |
| Robot body "Vehicle" | R:503-505 | `robot_model` = `models/talos3/Talos3_body.glb` (`robot.yaml model`), transform `body * modelOffset` (`baseToOrigin(vehicle)` from `base_link`) |
| Thruster rotors | R:506-507 | `thruster_visuals.yaml rotors[].mesh` (`models/talos3/rotors/*.glb`) |
| Status LED boxes | R:508-514 | `status_lights.yaml lights[]` |
| Payload launcher (`launcher.glb`), payload mesh (`projectile.glb`), loaded mounts | R:484-487, M:498-514 | `viewer.payload_model/launcher_model`, `vehicle.torpedoes{pose,baseline}`, `vehicle.droppers{pose}`, `task.torpedo|dropper{count,length,radius,slot_offsets}` (`payloadMounts`, H/payload_mounts.hpp) |
| Claw: `gripper.glb`+`gripper_right.glb` (pads), `assembly_static/left/right.glb` | R:488-502 | `viewer.claw_model` folder, mount `task.claw.pose` else `vehicle.claw.pose`, `claw.min_gap` |
| Robot magnet (`robot_magnet.glb`) | R:516-518 | mount `vehicle.magnet.pose` * `task.magnet_lights.robot_tip_offset` |
| Table props (pill, bandage, nut_and_bolt, plug) | via `simulator/task_objects` | `task.claw.props.<key>.mesh` |
| Projectiles in flight | `simulator/projectiles` | 4.9 |
| Point cloud | `pool_viewer` renderer points | camera products |

Hardcoded robot / task / year / camera names that must become data:

| Where | Literal | Should come from |
|---|---|---|
| M:909 window title; M:2038-2045 header | "Riptide | RoboSub Pool", "RIPTIDE / ROBOSUB SIMULATION" | scenario/pack title |
| M:1421 card titles; M:1425 subtitle | `ffc` -> "01 FORWARD CAMERA", else "02 DOWNWARD CAMERA"; "ZED X MINI" | camera display name/model in robot pack |
| M:290, 304-308, 685 | `ffc` = physics-owned camera, `simulator/` TF prefix, `<name>_left_camera_optical_frame`, `_camera_link` | bridge `frame_names`/camera ids |
| M:1683 (`mapCanvas`) | course-map landmark list `gate, slalom_front, slalom_back, torpedo, bin, table, octagon`; "slalom" label; hiding `slalom_back` | task pack `ui.map_landmarks` |
| M:1212-1253 focus() | literal presets `Course`, `Vehicle`, `Claw`, `Payloads`, `table` (dist 3.7), `magnet_target*` (dist .32) | per-target focus data (`ui.focus` list only holds names) |
| M:2282-2296 | labels skipped when focus is `Payloads`/`Claw` | same |
| M:264-277 | `torpedoes`, `droppers`, `torpedo_link`, `droppers_link`, `torpedo_{0,1}_link`, `magnet_link`, `claw` vehicle keys | robot pack mounts |
| M:498-514 | `torpedo`, `dropper` kinds, `_loaded` ns suffix | mechanism ids |
| M:1023 (cloud colours) | slot 0 orange, 1 cyan | camera index colour |
| M:552, 1845 | `yolo_orientation/visualization_marker_array`, `controller/mpc/predicted_path` (MPC "riptide_mpc" text) | overlay config (topic list) |
| R:247, 271, 292, 440, 455-461, 577 | mesh names `table_pill`, `table_nut_and_bolt`, `torpedo`, `bin_magnet`, `bin_vinyl`, `Task3_Blood_Fixed.png`, `Task3_Fire_Fixed.png`, signs `compass hammer_and_wrench buoy sos`, `bin_vinyl1..4` | task-pack asset/prop metadata |
| R:396-403, 406 | mapping keys `/**/zed_faker`, `/<robot>/riptide_mapping2`, `config_frame == tag` | pool/task pack |
| R:799 | hides objects named "Roof beam"/"Lighting" above z 4.5 (no such objects exist any more) | drop |
| R:481, 533, 505 | `riptide_mesh` key default = robot name; `Vehicle`, `Payload`, `Calibration`, `Robot magnet`, `Claw *` object names | asset ids |
| P/uwrt_actuators.cpp:60-97 | readings "Torpedoes remaining"/"Markers remaining" from `torpedo_available_count`/`dropper_available_count` | mechanism ids |
| P/run_panel.cpp:15,28 | default title "Run scorecard"; "Timer uses simulation time; pauses with physics. Stop is manual." | ui block |
| P/ros_runtime.cpp:12 | node `viewer_panels`; sender prefix `viewer` | generic |
| `viewer_tools.yaml` | `profile: task`, `node: physics_simulator`, all `simulator/*` topics | bridge/pack config |
| Talos config | `world` command frame, `ghost/base_link`, `kill_switch_id: 1`, `setTeleop`, `map/model_tf`, `estimated_origin_frame`, `dive_z -0.75` | robot/bridge pack |
| `T/competition.yaml ui` | 2026 role choices repair/rescue, coin flips, torpedo/bin/table previews (`ui.previews.bin.camera: dfc`) | already data (task pack) |

## 7. Notes for the rebuild

- The old viewer used TF for the truth pose and detections; it depended on `simulator/talos/*` truth TF plus stamp-exact render history to place detections. New viewer/bridge can use its own render poses, but must keep "place once at acquisition time" semantics and lifetime expiry independent of visibility.
- `Simulation` panel semantics to keep: speed range (0, 10], pause remembers resume speed, Sync = plant to estimate keeping velocity, Reset = start pose at rest and re-seed EKF (bridge `reset_sim_to_start`/`sync_sim_to_estimate` cover both).
- Score/run protocol and `ui` schema (task doc) are data-driven in the old viewer (competition-specific fields live in the profile and score publisher); keep that split.
- Preview/demo (`demo:=true`) and headless/screenshot modes are used by the tests and docs images (`camera_faker/docs/*.jpg`).
