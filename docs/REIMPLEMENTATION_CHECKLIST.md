# Full simulator reimplementation acceptance checklist

Status: active implementation. This is the working ledger for the user's request
on 2026-09-28 to continue until Talos, the robot stack, original water appearance,
props/tasks, operator panels, and the complete RepairCompTree work on the new
platform. Check a box only after its acceptance evidence is recorded. Local Git
commits only; the original workspace remains the reference, not a runtime dependency.

Delivery priority: original simulator replacement first, RViz replacement expansion
second, broader team/industry reuse third. Generalized ownership/dependency contracts
remain mandatory through every increment. Prefer work that closes concrete original
simulator gaps over unrelated viewer/platform expansion.

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
Current physics uses a spherical pool contact proxy; the viewer draws line geometry.

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
- [ ] Preserve COM/base_link/CAD frame conversions and offset velocity/acceleration.
- [x] Stage-dependent thruster submersion and water current evolution match the
  pinned synthetic free-motion fixture through native C++/profile/Python interfaces.
- [x] Expose actuator response/scales/deadbands/efficiency and immediate queue-clearing
  stop with coast-down, preserving clock and sensor acquisition phase.
- [ ] Verify partial buoyancy and hydrodynamic behavior under native Talos
  configuration and longer reference rollouts, including stop/watchdog response.
- [ ] Hull/course collisions with required friction/restitution and prop coupling.
- [x] Extract private static compound-box response and match 11 pinned original
  contact cases (reference/BOX_CONTACTS.md); Plant/profile integration remains open.
- [ ] Distinct placement, task reset and full reset semantics, plus ROS clock mapping.
- [ ] Match required Talos sensor products/calibration through declared models/adapters.
- [ ] Native assets and reproducible resolved configuration load outside old workspace.

## D. Renderer and UI fidelity

- [ ] Neutral scene/asset/material/light descriptors with no robot/year dispatch.
- [ ] Original scene/shadow/water/reflection/bloom/post shaders and pass semantics.
- [ ] Same meshes/textures/material corrections and task cutouts through content data.
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

## Current next increment and review loop

1. Source-linked review matrix is recorded; full reference capture remains open.
2. Read-only pose transport and application composition are validated. Explicit
   simulation controls remain open and separate from recording playback.
3. Stage forcing and actuator calibration/stop interfaces are verified in their
   bounded reference/contracts. Next capture and port compound box contact behavior,
   named body/world frame support and native Talos content; neutral scene/assets follow.
4. Re-evaluate this order after each validated commit against concrete dependencies.

For every increment: state its acceptance target, implement, test appropriate
contracts and installed boundaries, review, commit locally, update this ledger and
STATUS.md, and check the next step against delivered code. No intermediate milestone
is the full completion of this checklist.
