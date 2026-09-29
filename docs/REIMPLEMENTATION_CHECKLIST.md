# Full simulator reimplementation acceptance checklist

Status: the 2026-09-29 data-pack/real-stack plan below supersedes earlier
next-increment lists. Historical acceptance evidence remains below. Local commits
only; no pushes. No UI or visual-fidelity work until step 5 is complete.

## Current execution gates

- [x] 1. Schemas only: robot, pool, tasks, scenario and bridge; actual Talos,
  2026 pool, gate and torpedo data; UWRT bridge wiring; list every necessary task
  Python hook and custom-message converter with its reason. Joint source/schema
  review passed; production loaders and runtime checks remain in later gates.
- [x] 2. Generic YAML bridge, UWRT config and required converters: the existing
  controllers and EKF hold depth and heading about as well as the old simulator.
- [ ] 3. Robot-pack offscreen camera and stereo products: real perception runs on them.
- [ ] 4. Task runtime, launcher/dropper/claw mechanisms, all 2026 task packs and full
  reset: each task scores correctly in a scripted run.
- [ ] 5. RepairCompTree through the real stack with every descendant outcome checked;
  then a synthetic AUV on a different topic layout, a new pool and a new task,
  all without simulator code changes; one-page robot, pool and task guides.

Step 2 progress:

- [x] Standalone pack loader, lossless saves, source snapshots, generated schemas,
  registered-type output, validation CLI and explicit unresolved dependencies.
- [x] Connect resolved packs to the native runtime and preserve navigation behavior;
  native-profile parity and a distinct four-thruster pack passed.
- [x] Native placement preserves time and sensor schedules, clears stale readings,
  and supports clearing or preserving propulsion; invalid input is atomic.
- [x] Generic ROS bridge, UWRT navigation bindings and independently installable
  package; 110 installed ROS tests and 111 installed core Python tests pass.
- [x] Record alignment acknowledgement, supersession and failure; preserve the
  original simulator's coalescing behavior. Native placement passes ASan/UBSan.
- [x] Old/new real-stack hold comparison: repeated runs, equal controller config,
  actual truth/EKF errors and real-time factor, no rejected input or missing alignment.
  `python3 integrations/uwrt/acceptance/compare.py --output build/gate2-final`
  passed two trial pairs and independent Opus evidence review. New physical
  depth RMS 3.18–3.83 mm; heading RMS 0.27–0.38 degrees; real-time operation.

Step 3 progress:

- [x] Optional camera projection, owned image/depth processing, original depth noise
  and JPEG encoding; 12 CPU/frame tests pass in release and ASan/UBSan builds.
- [x] Renderer PNG textures, UV cutouts and camera readback; eight GPU contracts
  pass on the current machine, with an installed camera/renderer consumer checked.
- [x] Import declared Talos and gate/torpedo sensor-scene assets with verified hashes;
  use the original rectified calibration and calibrated stereo baseline.
- [x] Finish independent Opus review of camera processing and stereo calibration.
- [x] Declare robot visual placements, right-eye frames, water tint and lighting in
  packs; 79 existing and 10 new pack tests pass, including lossless saves.
  Water and lighting match Talos bringup startup, not the UI Pool preset.
- [x] Optional EGL capture host and standalone Python camera extension; eight host,
  nine renderer and five installed Python cases pass, including real stereo capture,
  context cleanup and image ownership. Camera-only wheel built from the source
  archive passes the five Python cases without DISPLAY, simulation or ROS.
  Opus reviewed and approved the host/binding corrections.
- [ ] Compose sensor scenes from pack data, including robot visual placement.
- [ ] Execute the existing robot-pack camera/stereo definitions offscreen.
- [ ] Publish calibrated, synchronized image/CameraInfo products through the bridge.
- [ ] Run actual UWRT perception on those products and review its output.

The user delegated approval to Codex and Claude Code, using Opus 5.5/high. Agree
ownership, have the other partner review each gate's evidence, and continue without
human approval. Stop and report genuine blockers; do not work around a wrong gate.
Claude may delegate bounded mechanical work to Sonnet 5.5, with separate file
ownership; Codex and Opus retain difficult work and validation. Keep readable meeting
notes in the Git-ignored `collaboration_logs/` directory; raw output stays in `build/`.

Before every change: does it move the current step toward its gate? If not, defer
it below. Add types/fields only for concrete Talos/2026 needs. Preserve existing
dependency boundaries; no unrelated restructuring, plugin frameworks or generic
event buses. Packs own folder-relative YAML/assets; schemas and type introspection
come from loader definitions; load/save is lossless; runs record resolved config.
User Python is only a task-pack hook taking read-only state/events and returning
score changes/new events, with no runtime or stepping access. Bridge wiring is data;
only required custom-message converters live in `integrations/uwrt`.

## Deferred follow-ups

- Resume UI, LEDs, radiance and visual-fidelity work only after step 5 passes.
- Revisit broad RViz and industry expansion after the simulator/data-only gates.
- Keep future training use possible; do not implement RL or general sonar now.
- Revisit timestamp-matched TF lookup when task mapping makes external frames move;
  gate 2 uses the old simulator's latest-TF behavior and fixed external transforms.

## Reference and fidelity policy

- Original simulator baseline: `riptide_simulator` commit
  `07647eebe706f96ea7b76db3cc9802735a146698` (clean worktree at inventory).
- Preserve intended Talos physical behavior and the old renderer's appearance.
  Reuse validated algorithms, shader passes, and assets behind neutral contracts;
  do not substitute simplified physics or water effects and label them equivalent.
- UI should look and operate very similarly to the old simulator in the Talos
  workspace. Empty/live-data workspaces still require no simulation or pool.
- Compare identical inputs, physical frames, acquisition times, seeds/settings,
  and supported graphics backend. Record tolerances and any platform-dependent
  floating-point/image differences. Existing approximation limits remain explicit.
- Legacy incidental package names/APIs are not core contracts. Required existing
  robot-stack interfaces belong in separately built ROS/UWRT adapters.
- Robot, pool/environment, task instances and scoring rules remain independent
  compositions as specified in ARCHITECTURE_PLAN.md. Talos mission completion is
  the final integration gate, not a substitute for the generic extension gates.
- Architectural corrections (independent acquisition, explicit clocks, coordinated
  ownership) must retain physical/rendering behavior at equivalent inputs. No
  unreviewed fidelity change should be hidden as cleanup.
- A mission action/root success is insufficient when descendants mask failure.
  Require descendant execution evidence and physical task outcomes/score events.

## Already established foundation

- [x] Separate local repository, CMake installs, dependency checks, contribution docs.
- [x] Headless deterministic C++ plant and explicit command/step/observe/full reset.
- [x] IMU/FOG/DVL/pressure models with independent schedules, noise, and typed samples.
- [x] Native robot/world/sensor profiles and installed Python runtime access.
- [x] Independent ImGui/GLFW/OpenGL viewer; source/frame/display contracts, local
  playback, workspace save/reopen, source-generation handling, installed extensions.
- [x] Viewer-only install and neutral libraries without graphics discovery verified.

These are foundation checks, not Talos fidelity or full platform acceptance.
Current physics offers disabled, sphere-pool and static compound-box contacts;
the baseline viewer draws lines, while its optional scene build now draws original
body/pool content from source frames.

## A. Capture and lock the acceptance baseline

- [ ] Finish capability/interface matrix for old simulator, viewer, and robot stack.
- [ ] Record pinned revisions/configuration/resource hashes for Talos and 2026 pack.
- [ ] Capture deterministic numerical reference fixtures and physical-frame mapping.
- [x] Capture pinned original free-motion equations/kernels and compare 753 synthetic
  submerged/surface/entry states; see reference/STAGE_DYNAMICS.md for limits.
- [ ] Capture fixed-camera, fixed-time renderer reference images (above/below water,
  indoor/outdoor, reflections, shadows, tags/holes, props, payloads, sensor cameras).
- [ ] Record representative physics/contact/camera/UI rates and startup/resource use.
- [x] Record a repeatable current-machine synthetic headless baseline with machine,
  build and content identity (PERFORMANCE.md); full Talos/graphics baseline remains open.
- [ ] Inventory all reachable RepairCompTree descendants and their success evidence.

## B. Simulation and viewer connection

- [x] Separate read-only simulation source adapter with bounded delivery/history.
- [x] Prove attached/detached/slow pose transport leaves trajectories and noisy IMU
  samples equal through reset/reconnect (see LIVE_VISUALIZATION.md). Camera/other
  future payload transport requires the corresponding acquisition comparisons.
- [x] Application composition connects runtime and viewer without inward dependencies
  (finite scenario worker, pose/trajectory, sensor draining independent of display).
- [ ] Optional simulation control provider: pause/resume, full reset, task reset,
  placement, acknowledged state; separate from playback and live robot commands.
- [ ] Rewinds, resets, reconnects and source changes invalidate incompatible visuals.

## C. Talos physics and native content fidelity

- [ ] Standalone Talos physical parameters, frames/mounts, thrusters and device models.
- [x] Pinned native Talos dynamics/mount pack, eight thrusters and original hull/pool
  proxies load independently (reference/TALOS_PHYSICS_PACK.md). Devices remain open.
- [x] Independent original Talos dynamics capture: startup contact, partial immersion
  and floor approach, every tick of three 1500-step runs, including realized forces
  and accelerations (reference/TALOS_DYNAMICS.md). Full mission parity remains open.
- [ ] Preserve COM/base_link/CAD frame conversions and offset velocity/acceleration.
- [x] Shared rigid transforms, named native sensor mounts and immutable live viewer
  mounts; rotated mount equivalence and Talos COM/CAD/base offset algebra verified.
  Actual native Talos assembly and sensor reporting remain open.
- [x] Scenario-owned upright pool/static-world placement shared by collision and
  finite-floor queries, without transforming robot mounts or world current vectors.
- [x] Stage-dependent thruster submersion and water current evolution match the
  pinned synthetic free-motion fixture through native C++/profile/Python interfaces.
- [x] Expose actuator response/scales/deadbands/efficiency and immediate queue-clearing
  stop with coast-down, preserving clock and sensor acquisition phase.
- [ ] Verify partial buoyancy and hydrodynamic behavior under native Talos
  configuration and longer reference rollouts, including stop/watchdog response.
- [ ] Hull/course collisions with required friction/restitution and prop coupling.
- [x] Extract private static compound-box response and match 11 pinned original
  contact cases (reference/BOX_CONTACTS.md).
- [x] Select contacts through C++/native profiles/Python, with robot/world-owned
  proxies and 550 original whole-step states covering pre/post-RK4 ordering.
  Dynamic prop contacts and full Talos/course acceptance remain open.
- [ ] Distinct placement, task reset and full reset semantics, plus ROS clock mapping.
- [ ] Match required Talos sensor products/calibration through declared models/adapters.
- [x] Raw IMU supports explicit measurement-only gravity calibration and independently
  reported sensor-axis variances through C++/profiles/Python.
- [x] Separate simulated attitude and composed AHRS outputs with shared acquisition,
  independent noise, copied C++/profile/Python contracts and CSV telemetry. Native
  Talos full device composition, original output captures and ROS products remain open.
- [x] Pinned native Talos 50 Hz AHRS and 500 Hz FOG assembly, source-independent
  execution, complete three-second acquisition and unchanged plant trajectory.
  Navigation assembly below adds DVL/depth; cameras remain open.
- [x] Independent extracted original AHRS/FOG measurement formulas match 24 prescribed
  kinematic states, now also including original DVL velocity/variance and retained
  uncertainty with sampling noise disabled
  (reference/SENSOR_KINEMATICS.md). Whole-stack timing/products remain open.
- [x] Separate reference-velocity model and original default Talos DVL policy,
  optional inclination validity, native 8 Hz acquisition and captured formula parity.
- [x] Explicit reference-altitude observation with same-acquisition mount-to-target
  correction, original Talos 20 Hz depth configuration and all 28 captured sensor
  fields verified. Physical pressure remains independently selectable.
- [ ] Native assets and reproducible resolved configuration load outside old workspace.

## D. Renderer and UI fidelity

- [x] Independent CPU mesh/material loader and exact original Talos body/eight-rotor
  resource pack, pinned hashes, original submesh comparison and relocated consumption.
- [x] Neutral mesh instances, material/emission/lighting inputs and optional water,
  with a context-owned renderer independent of viewer/simulation/ROS.
- [x] Optional interactive mesh/water viewport with neutral scene documents, frozen
  source-frame resolution, independent overlays and relocated standalone consumption.
- [ ] Complete Talos visual assembly with no robot/year dispatch.
- [x] Original scene/shadow/water/reflection/bloom/post pipeline over exact body/pool
  inputs; all 30 captured buffers match original release/debug builds on this backend.
- [ ] Texture/cutout/overlay inputs and complete scene acceptance through that pipeline.
- [ ] Same meshes/textures/material corrections and task cutouts through content data.
- [x] Neutral source-clock rotor and indicator state models; original captured phases,
  pivot matrices and LED colors verified (ANIMATION.md).
- [x] Source-owned rotor motion through atomic body/moving-frame batches, named
  plant channels and original imported Talos rig; drops/reconnect do not change phase.
  Animated rotor pixel comparisons remain open.
- [x] Original three LED bars/material/radiance through generic source color bindings;
  fixed original off/red/individual-RGB buffers match (INDICATORS.md).
  Simulation command ownership and ROS target routing remain open.
- [ ] Same Talos 3D mesh/materials with thruster/rotor motion, direction and speed
  driven by explicit state; LEDs and all other status/mechanism visuals reproduce
  the original observable state transitions without robot-name logic in the renderer.
- [ ] Same RoboSub 2026 pool and task/prop models, placements, dimensions, textures
  and visual interactions through independently composed world/task/robot content.
- [ ] Preserve observer lighting/water controls without modifying sensor appearance.
- [ ] Matching camera projection/optical conventions and image/depth registration.
- [ ] Talos-style observer, follow/focus, sidebars, camera views, map and inspection UI.
- [ ] Visual comparisons and representative full-scene performance checks.
- [ ] Reference old simulator workflows and animated/stateful visuals as well as
  fixed screenshots: LEDs, thrusters, claw, payloads, task feedback and operator panels.

## E. Mechanisms, props, tasks and scoring

- [ ] Node-free example mechanism/task extensions and capability validation.
- [ ] Robot mechanisms work without competition tasks; tasks declare required
  capabilities and reject incompatible bindings without robot-name dispatch.
- [ ] Pool/environment packs load independently of robots and task selections;
  task instances and scoring rules can be selected/replaced independently.
- [ ] Talos arm/disarm/kill/cooldown/ammunition, torpedo/dropper launch state.
- [ ] Payload trajectories, swept contacts, loaded/released visual continuity.
- [ ] Claw actuation, grasp/release, attachments, table props and magnet-light behavior.
- [ ] Runtime owns synchronized vehicle/prop/contact updates and physical state.
- [ ] 2026 task interactions/events/scoring; selected instances and independent rules.
- [ ] Full reset/replay clears every model, queue, contact, attachment and score state.

## F. Cameras and independent visualization

- [ ] Scheduled camera and synchronized stereo acquisition with typed products.
- [ ] Required RGB/depth/calibration/cloud processing and declared noise models.
- [ ] Real offscreen rendering without a desktop; viewer is never the sensor owner.
- [ ] Acquisition and required outputs independent of viewer/subscriber activity.
- [ ] Neutral robot/joint, marker, image, cloud and path displays for live sources.
- [ ] Bounded asynchronous source delivery, staleness/gap diagnostics, explicit
  multi-source frame/time alignment and saved workspace composition.

## G. Optional ROS/UWRT and whole mission

- [ ] Generic simulator bridge for commands, observations, TF, clock and lifecycle.
- [ ] Independent live ROS viewer adapter with no simulation/UWRT dependency.
- [ ] UWRT sensor/frame conventions, kill/telemetry and estimator alignment.
- [ ] Motion/mapping/autonomy/actuator/run panels with explicit optional providers.
- [ ] Isolated full-stack launch using new platform and declared Talos content.
- [ ] Required pinger sampling/amplitude behavior (not a general sonar simulator).
- [ ] RepairCompTree stages execute with descendant traces and physical outcomes:
  prep/dive, gate, slalom, pinger selection, torpedoes, table, bins/magnet, return/kill.
- [ ] Repeat mission under documented seeds/configurations; verify resets and failure
  reporting, not just action transport or ForceSuccess-wrapped root completion.

## H. External reuse and distribution readiness

- [ ] External robot/content/task/source/display examples using installed public APIs.
- [ ] Two distinct robot definitions exercise the example task capability contracts;
  empty-pool, selected-task, and alternative-scoring compositions run standalone.
- [ ] Standalone, viewer-only, camera-only, generic ROS and Talos build/run matrix.
- [ ] Original behavior/visual/performance comparisons accepted with recorded limits.
- [ ] Record current-machine provisional performance baseline; repeat headless and
  full-scene/stack measurements as Talos, contacts, cameras and original rendering
  arrive. Demonstrate real-time operation at the declared physics/sensor settings.
- [ ] Resolve source/asset licenses, dependency inventory and maintenance contacts
  before public distribution; documentation and packaging reflect actual support.

RL implementation and general sonar remain deferred. Explicit stepping, seeded
reset, observation ownership, headless operation and instance isolation are retained
for future training integrations. Full RViz feature parity is broader than the
selected live-robot workflows required here.

## Review loop

Only the ordered current execution gates at the top determine the next increment.
State the gate, make the smallest relevant change, validate, obtain peer review,
commit locally, and update STATUS.md and this checklist. Report each gate in no
more than 15 lines: done, proof command, remaining failures and open questions.
No intermediate milestone constitutes full simulator acceptance.
