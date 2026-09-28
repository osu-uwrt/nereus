# Source-linked replacement acceptance matrix

Reviewed against original simulator revision
`07647eebe706f96ea7b76db3cc9802735a146698` on 2026-09-28. Paths below are relative
to the release workspace. These are requirements and review findings, not passes.
Independent read-only reviews covered physics, rendering, and the whole mission.

## Mission and external stack

Entry: `src/riptide_autonomy/trees/RepairCompTree.xml`, `BehaviorTree` ->
`CompMissionTree`, with `repair_mission=1`. Reachable stages include kill wait,
preparation/dive, gate/flip, slalom, pinger sampling, torpedo/table in selected order,
bins/magnet, return home, and controller kill. The external launcher is
`src/riptide_launch/riptide_bringup/launch/simulation.launch.py`; its normal bringup
uses `hardware=none`, shared simulated time, and the hybrid controller.

All listed relative endpoints are under `/talos` unless explicitly absolute.
They belong to optional adapters, never platform core configuration fields.

| Workflow | Required contracts | Evidence/gate |
| --- | --- | --- |
| Plant/control | `/clock`, `thruster_forces` Float32MultiArray, `command/software_kill` KillSwitchReport, `state/kill` Bool, actual thrust and truth | Isolated closed-loop controller/EKF runs with fixed input traces and correct command/kill response |
| Navigation | `vectornav/imu` Imu; `gyro/twist`, `dvl_twist` TwistWithCovarianceStamped; `depth/pose` PoseWithCovarianceStamped; `odometry/filtered`, `set_pose` robot_localization/SetPose | Mounted signals, base-vs-COM transforms, calibration, covariance, clocks and reset behavior |
| Operator placement | `set_sim_pose`, `sync_sim_to_estimate`, `reset_sim_to_start`, real-time-factor | Distinct full reset/task reset/placement, correct state preservation and immediate paused result |
| Cameras | FFC/DFC ZED-shaped RGB/left/compressed/depth/CameraInfo/cloud endpoints and optical frame IDs | Same physical image/depth projection; optional UWRT conversion; actual detector consumes rendered images |
| Perception/mapping | detected_objects, camera/slalom/table-pair selection services, mapping_target/state, object poses/TF, classification/freeze/geometry-fit/reset services | Existing detector weights and mapping execute; ground-truth detections do not substitute for this gate |
| Mechanisms | Arm/claw SetBool services and Bool topics; torpedo/dropper/reload Trigger services and Empty topics; claw_move_s Float32; status/busy/result | Correct rejection, timing, ammunition, release/grasp physics, and reset |
| Run/tasks | Events/score/run commands, task/table/magnet resets, projectiles/props/joints/lights | Independent physical events and scoring; no duplicate ownership of objects |
| Autonomy | `autonomy/list_trees`, `autonomy/run_tree` ExecuteTree; motion/trust/stunt endpoints | Descendant execution traces plus physical outcomes, not action/root success alone |
| Passive pinger | Acoustics start/stop sample and buffer_0_closer; current amplitude input | Real acoustic node receives an explicitly modeled amplitude stream and makes the expected relative choice |

Source references: original `physics_simulator.cpp`, Talos
`robots/talos/behavior/mechanisms.py`, `tasks/2026/behavior/node.py`, viewer `main.cpp`,
`riptide_autonomy/include/riptide_autonomy/autonomy_lib.hpp`, and
`riptide_perception/riptide_mapping/riptide_mapping2/mapping.py`.

Known pre-existing integration gaps:

1. Original simulator emits `acoustics/delta_t` Vector3Stamped, while the current
   `riptide_core/riptide_acoustics/src/Acoustics.cpp` consumes
   `/talos/ivc/pinger/selected_freq_amp_stream` Float32. RepairCompTree needs this
   node's two-buffer sampling. Add the missing passive-pinger behavior through a
   declared model/adapter; this does not expand scope to general sonar simulation.
2. `RepairCompTree.xml` publishes Int8 `controller/stunt_state`, while
   `TriggerControllerStunt.btaction.hpp` uses UInt16 on that endpoint. Verify the
   controller's actual contract before accepting the flip/stunt stage.
3. The mission uses ForceSuccess wrappers, and `DoTask.cpp` can report action
   transport success despite BT failure. Log descendant failures and independently
   verify task events/scores. Existing `riptide_jev/docs/BASELINE.md` does not establish
   a qualified complete successful mission baseline.
4. ZED description package was not found during review; native Talos camera TF
   must be complete without requiring the physical camera driver. NCNN model
   directories exist; detector runtime and weight identity still need validation.

## Physics and sensing fidelity

| Required behavior | Original reference | Current gap and acceptance |
| --- | --- | --- |
| Midpoint actuator/RK4 orchestration | physics_simulator.cpp stage loop | Both use half-step actuator split. Preserve it; do not evolve actuators independently at RK stages. |
| Surface propulsion | robot_class.cpp propulsionWrench | Re-evaluate each propeller disk's submerged fraction at every RK stage, using radius/axis/water level. New plant currently uses constant stage wrench. |
| Evolving current | robot_class.cpp dynamics current calculation | Add oscillating current and analytic acceleration evaluated at stage times; preserve existing kernel equations. |
| Thruster configuration/kill | robot_class.cpp parameters, physics_simulator.cpp stop | Expose existing kernel deadband/scales/efficiency and queue-clearing stop; zero command is not equivalent to kill. |
| Hull/course contacts | collisionBox_class.cpp; physics_simulator.cpp OBB collision passes | Current sphere proxy is insufficient. Preserve required pre/post integration order, OBB response, friction/restitution, pool transform and finite walls. |
| Props/contacts | task_contacts.cpp, claw_world.py | Resolve generic shapes/materials separately from scene names/YAML; record vehicle/prop exchange order before changing ownership. |
| Physical frames | robot_class.cpp COM/base/CAD conversion | Convert COM state into base pose and offset velocity exactly once; handle rotated mounts and COM overrides. |
| IMU | physics_simulator.cpp publishIMU; config/talos_sensors.yaml | Talos requires orientation and reported gravity 9.755455 to match EKF. Generic raw specific force remains a separate contract. |
| Depth | physics_simulator.cpp publishDepth | Legacy navigation consumes base_link map Z, not pressure Pa or positive-down mount depth. Adapter must convert sign, water level and mount/base offsets explicitly. |
| DVL lock | physics_simulator.cpp publishDVL | Original default has no floor/range lock loss. Make the required reference policy explicit instead of silently changing generic bottom-track behavior. |
| Placement | physics_simulator.cpp reset/placement | Legacy placement clears actuator/dynamics but preserves ROS clock/sensor phase. Add placement separately; retain complete seeded full reset. |
| Noise | old shared mt19937 vs new independent device streams | Start noiseless parity; compare declared noise statistics separately. Do not reintroduce accidental cross-device RNG coupling into core. |

Reference workloads: 2 ms ticks; submerged/tilted free motion; each thruster in
both directions and saturation; queued kill/timeout; surface crossings; oscillating
current; COM/mount offsets; glancing/corner/floor contacts; carried/released props;
placement versus full reset; acquisition count/phase. Compare every tick, not just
final poses. Initial same-build targets: 1e-12 absolute/relative pure forces and
derivatives; 1e-9 SI/geodesic-radian no-contact trajectories; exact timestamps and
command/event ordering. Contact and GPU tolerances require per-fixture evidence.

## Renderer, sensors, and UI fidelity

Preserve the original `camera_faker/shaders/pool` scene, shadow, water, reflection,
bloom/post, points and focus pass semantics. Most underwater absorption/haze/
caustics live in scene.frag. Preserve render order, HDR/depth formats, filtering,
shadow bias/resolution, blend/cutout behavior and bloom downsampling. Original
camera.hpp has a half-pixel projection correction that must survive extraction.

Renderer constructor ROS-parameter YAML and named scene objects move to an importer
and native content. Torpedo backing cutouts and table-prop material/UV repairs move
to explicit prepared assets or generic material data. Talos body, rotors, launcher,
claw, loaded rounds, lights and task geometry are content, not renderer dispatch.

Use a reference harness with explicit scene, camera, time and Look. Old GUI captures
advance water animation with wall time and are not deterministic pixel goldens.
Compare raw intermediate/final color and metric depth at matched settings on the
same backend, including above/below water, surface crossings, indoors/outdoors,
reflections, transparent/cutout assets, lights, and carried/released props. Keep
observer-only edits separate from authoritative environment/sensor settings.

UI reference: original full-window theme, resizing sidebars, observer follow/focus,
map/camera cards, scene controls, neutral panels and pose gizmos. Compose Talos
providers outside the host. Compare equal viewport/font sizes and workloads; old
1480x940/VSync-off versus new 1280x800/VSync-on is not a useful speed comparison.
Measure frame latency, physics/contact rate, native FFC/DFC acquisition, readback,
CPU/GPU work and memory separately. Retain actual detector/mapping tests.

Ported shaders, renderer code, CAD conversions and assets require updated provenance.
The original package's TODO license does not establish redistribution permission;
local implementation continues under the project's existing license-status policy.
