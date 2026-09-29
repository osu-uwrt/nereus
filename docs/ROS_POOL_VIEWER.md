# ROS pool viewer (`robotics-pool-viewer`)

ROS 2 client GUI for the simulator bridge (`robotics_platform_ros`). It reproduces the old
`camera_faker` `pool_viewer` window (operator panels, live pool view, camera cards, overlays) minus its
camera-faker role: the bridge renders and publishes the camera images, and the viewer **displays** them.
Parity checklist: [reference/POOL_VIEWER_INVENTORY.md](reference/POOL_VIEWER_INVENTORY.md) sections 1, 2.1, 5, 6.
Optional integration (`integrations/ros2/viewer`, `RP_BUILD_ROS_VIEWER`); nothing in `libraries/` depends on it.

## Build and run

```
cmake --preset ros-viewer && cmake --build build/ros-viewer --target robotics-pool-viewer -j2
# live: bridge in one shell, viewer in another (same RMW / ROS_DOMAIN_ID)
python3 -m robotics_platform_ros content/packs/scenarios/talos_uwrt --output /tmp/run
build/ros-viewer/robotics-pool-viewer                      # scene from the latched scenario topic
build/ros-viewer/robotics-pool-viewer --scenario /tmp/run/resolved.json   # scene from a file, ROS still live
build/ros-viewer/robotics-pool-viewer --demo --scenario /tmp/run/resolved.json   # scene preview, no ROS
build/ros-viewer/robotics-pool-viewer --hidden --frames 60 --screenshot out.png [--view ffc] [--open tf] ...
```

`--help` lists every flag. `--scenario` documents without `asset_paths` resolve assets against the scenario pack
found under `content/packs/scenarios` (or `--pack-dir`). ROS launch wrapper: `launch/robotics_pool_viewer.launch.py`.
Capture mode saves the full window PNG plus `<stem>-<camera>.png` from the last received camera image.

## Data flow (nothing reads old simulator config files)

| Source | Used for |
|---|---|
| `/talos/simulator/scenario` (String JSON, transient_local): `resolved.json` layout + `asset_paths` | pool, tasks with UV cutouts, robot visuals, frames, mechanisms, cameras (topics/intrinsics), thruster order, frame names, truth TF name, `ui` |
| TF `map -> <truth base link>` (`bridge.tf.publish`, `reference_pose`) | vehicle pose, follow, connection pill; camera poses/detection acquisition = truth pose composed with pack frames |
| other TF, `/clock` | TF overlay/tree, "ROS vs sim", MPC re-rooting, all animation clocks |
| bridge camera topics (from `bridge.streams`, `sensor:<id>.rgb_left/depth_left/camera_info`) | camera cards (JPEG decoded with libjpeg at DCT scale, depth lazily subscribed) |
| `simulator/actual_thruster_forces`, `command/led`, `simulator/claw_joints`, `simulator/magnet_lights`, `simulator/task_objects`, `simulator/projectiles`, `yolo_orientation/visualization_marker_array`, `controller/mpc/predicted_path` | rotors, LED bars, claw, magnet lights, props/projectiles (MESH_RESOURCE `file://` or `package://`), detections, MPC path. Absent publishers are simply absent. |
| `content/viewer/talos_uwrt_host.yaml` | branding, camera card names, focus presets, landmark map, extra robot visuals, claw/payload/magnet mapping, topic names, calibration board, fallback task `ui` |
| `content/viewer/talos_uwrt_thruster_visuals.yaml`, `..._status_lights.yaml` | rotor pivots/axes/RPM fit, LED geometry and input topic |
| `content/viewer/talos_uwrt_panels.yaml` | operator panel composition (`--panels FILE|none`); `profile: task` = document `task` = scenario `ui` |

## Source map (`integrations/ros2/viewer/src`)

`scenario` (document parsing), `scene_model` (renderer scene composition like `pack_cameras.py`), `ros_side`
(subscriptions/TF/detections/MPC/feeds; single-threaded, driven by `spin()`), `app` (window, toolbar, overlays, cards,
map, panels host), `overlay_draw`, `window` (GLFW/GLEW/ImGui, PNG), `jpeg_decode`. Ported headers (see
[PROVENANCE.md](PROVENANCE.md)): `viewer_input`, `tf_tree`, `detection_pose`, `status_lights`, `thruster_visuals`,
`camera_geometry`. Tests `tests/host_*` (gtest): viewer_input, tf_tree, detection_pose (+ truth-base acquisition),
status_lights, thruster_visuals, camera_geometry (+ frame graph), jpeg, scenario, plus `host_demo_capture` (needs a display).

## Parity status by inventory section

| Section | Status |
|---|---|
| 1.1 header, pill, sidebars (resizable/hideable), viewport badge, run readout, footnote | done (pill: SCENE PREVIEW / WAITING FOR PHYSICS / POSE STALE / PHYSICS CONNECTED; plus WAITING FOR SCENARIO) |
| 1.2 toolbar (Scene settings, tool slots, Pool Viewer, Panels, View, Focus, Follow, Labels, TF, Detections, MPC path, preview combo) | done |
| 1.3 orbit / pan / zoom / F picking / free camera / footer / focus presets / trail / focus dot | done (presets are data: `focus_presets`) |
| 1.4 operator panels | done through `rp_viewer_panels` (Composition, pose gizmo overlay, providers) |
| 1.5 camera cards (title, model, WxH, Hz, RGB/DEPTH, status), mini map + expand | done; "Show both depth maps" button omitted with the Depth tab |
| 1.6 Scene settings: mechanism buttons, Lighting, Water appearance | done as **observer-only** settings; Depth sensor tab omitted (sensor rendering and noise live in the bridge) |
| 1.7 scene / robot visuals | done for everything the packs describe (pool, water, shadows, course meshes with cutouts, robot, rotors, claw, calibration board); octagon ring and crates are procedural in the old renderer and appear only when a task pack provides visuals; launcher/projectile/magnet meshes render once those pack assets exist (currently `status: missing`) |
| 1.8 overlays: detections (acquisition-time placement, lifetime expiry), MPC path, TF axes + "ROS vs sim", labels | done; point-cloud overlay **missing** (bridge publishes no cloud; depth image only) |
| 1.9 TF popup / tree | done (demo shows the robot-pack frames) |
| 1.10 modes: demo, capture (`--hidden --frames N --screenshot`), scorecard, initial toggles | done; `--panels none` replaces `operator_panels` |
| 1.11 status LEDs, rotors, claw, magnet lights, props, payloads | done |
| 2.1 ROS interfaces | done; the viewer no longer publishes camera images, static camera TF or `water.*`/`depth_*` parameters |
| 5 composition/config | done (panels YAML unchanged; host YAML new) |
| 6 hardcoded names | moved to data (scenario, pack, `talos_uwrt_host.yaml`); camera-frame names from `bridge.frame_names` |

## Deviations

- Camera sensor look (water, noise, depth model) is not editable from the viewer; Water/Lighting tabs change only
  the observer render. Runtime editing of the camera products needs a bridge parameter path (open item).
- Detections resolve at the *truth* base pose at the marker stamp composed with the pack's fixed base-to-camera
  transform (the bridge publishes no per-camera truth TF); markers wait for TF instead of falling back to estimates.
- Loaded payload rounds are drawn from the mechanism slot poses (`payloads.loaded_namespaces`), released rounds and
  props from marker poses; magnet lights are emissive boxes at the marker pose/scale rather than bin LED meshes.
- In `--demo` the camera cards show this viewer's own render from each sensor pose (there is no stream).
- `--local-cameras` (default on; `--local-cameras false` restores topic-only cards): with a scenario document the
  RGB of each camera card is rendered by this viewer from the truth pose at preview size (10 Hz) and no image topic
  is subscribed; the card checkbox "What the stack sees (ROS)" switches that card to the bridge's images. Depth
  always comes from the topic. In ROS mode JPEGs are decoded on a worker thread (newest frame wins), never on the
  UI/spin thread. Pool, task visuals (cutouts, per-visual `texture`) and robot visuals are composed by
  `libraries/pack_scene` (`rp_pack_scene`), the same code the simulator cameras (`libraries/session_cameras`) use.
