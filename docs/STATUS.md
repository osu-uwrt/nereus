# Implementation status

## Current priority: real-stack mission through data packs (2026-09-29)

This section supersedes the historical next-step notes below. Complete
RepairCompTree through the real UWRT controllers, EKF and perception, checking
all descendant tasks against physical outcomes. Robot, pool, tasks and bridge
inputs are YAML-and-asset folders; a scenario selects them plus initial state and
seed. User Python is restricted to task/scoring hooks. No user code is required
for robots, pools, sensors or bridge wiring.

Visual work stopped at local commit `9081760`. UI, LEDs, radiance and visual
fidelity stay deferred until step 5 passes. Offscreen camera work is explicitly
part of step 3. Do not restructure unrelated components.

Current step: **2 — generic bridge and the real controller/EKF hold gate.**
Step 1 passed joint review. `proposals/step1/` contains nine proposed JSON Schemas
and eight YAML documents: Talos robot, 2026 pool, gate/torpedo task definitions,
task manifest, UWRT bridge, scenario and the exception ledger. One task-pack
Python scoring hook is declared for step 4; no custom ROS converter is currently
necessary because the required bindings use typed declarative field mappings.

Local proposal checks: `python3 build/check_step1.py` validates all eight documents;
`check_step1_sources.py` confirms native Talos body/thruster/navigation-sensor data;
`check_step1_references.py` checks selected packs, frames, assets and endpoint ids;
`check_step1_provenance.py` checks 37 source records, including 11 against declared
Git revisions; `check_step1_rejections.py` rejects 11 invalid mutations. Eight
provenance-free documents and a sensor-only bridge also passed schema validation.
These are local review helpers under ignored `build/`, not shipped loader tools.
Opus reviewed the contract and approved after the independent provenance check.

No loader, generated production schema, bridge or runtime behavior is delivered
by this schema gate. Declared camera/task assets and the scoring-hook body remain
explicitly absent until their steps. The random-pinger mission branch needs a
sensor model and source positions in step 4; it is recorded as blocker B1 in the
exception ledger. Step 2 must prove depth/heading holds through the real stack,
with an old-simulator baseline; static schema checks do not satisfy that gate.

The first step-2 data increment implements `robotics_platform.packs`: YAML 1.2
loading, task includes and cross-reference checks, lossless untouched saves,
comment-preserving edits, a single installed definition corpus, generated schema
and type exports, and reproducible resolved manifests with source-change detection.
Task hooks are never imported during validation. Config tooling imports without
ROS or the native simulation extension. Missing assets/hooks remain explicit;
`validate --strict` rejects them for every pack kind. Planning milestone numbers
are diagnostic metadata and do not select runtime behavior.

Commands: `PYTHONPATH=python/src python3 -m robotics_platform.packs validate
proposals/step1/packs/scenarios/talos_uwrt --dump build/step2-resolved.json`,
`types --json`, and `schema robot` on the same module. The installed-component
Python suite passed all 110 cases (the initially skipped C++ comparison was run
separately with `RP_REFERENCE_RUNNER=build/release/robotics-sim`). Ruff and strict
mypy passed for the changed Python code. This remains below the real ROS hold gate.

The user replaced human approval with joint Codex/Claude Code execution. Claude
uses `claude-opus-5-5` with high effort. It may delegate bounded mechanical work
to `claude-sonnet-5-5`; the exact model was checked successfully. Codex and Opus
retain architecture, difficult integration, review and gate decisions. Delegate
assignments have separate file ownership and short evidence-based handoffs.
Codex owns integration, runtime evidence and commits. Claude drafts concrete
packs, later task-scoring hooks and guides, and independently reviews gate evidence.
Use short file/diff references in the same Claude session. Review each gate's
concrete evidence and proceed without human approval; a review does not replace
runtime tests. Report each gate in at most 15 lines. If a blocker cannot be resolved
within the agreed scope, report it rather than inventing a passing gate.

Readable meeting notes, decisions and gate reviews live in the locally ignored
`collaboration_logs/` folder, never in Git. Raw diagnostic output stays under
`build/`. Keep changes small, validate against the current gate, and commit locally
with a one-line message. The checklist owns the ordered gates and deferred
follow-ups. Only this file, that checklist and the three requested final guides
receive new prose documentation.

This file distinguishes delivered code from the architecture's future work.
The active full-refactor ledger is [REIMPLEMENTATION_CHECKLIST.md](REIMPLEMENTATION_CHECKLIST.md).

## Initial standalone slice

Implemented:

- A separate local repository with no remote, no legacy runtime/build dependency,
  and its own CMake builds and installed-library API.
- Composed C++ vehicle plant: marine dynamics, delayed/saturating thrusters,
  fixed integer ticks, observation by value, full plant reset, instance isolation.
- Pool walls/floor with a conservative spherical contact proxy.
- Synthetic four-thruster AUV, native scenario validation, fixed command schedule,
  CSV runner, and external CMake consumer.
- Numerical reference and new public-contract tests; quality/installation checks.
- Architecture plan, design decision, contribution guidance, and source provenance.

This begins phases 0 and 1. It does not claim completion of the full phase-0
Talos/RViz workflow inventory, contact baselines, or all phase-1 reference comparisons.
Numerical fixtures come from the independent models; no comparison against a live
Talos ROS pipeline has been performed for this milestone.

## Nonvisual sensor increment

Implemented:

- Installed `RoboticsPlatform::sensors` library composing the standalone plant,
  typed model extensions, integer acquisition schedules, and delayed delivery.
- Data-only sample/payload headers, bounded queues with explicit overflow policy,
  stream lifetime/fault handling, reset generations, and independent seeded noise.
- Mounted IMU specific force/angular rate, one-to-three-axis FOG, and ideal
  bottom-track DVL with finite pool-floor queries and explicit unavailable readings.
- Read-only endpoint plant kinematics, including contact-acceleration validity.
- Standalone installed sensor example, public-contract tests, and documented
  timing, noise, coordinate-frame, provider, and model-fidelity limits.

This is part of phase 2, not completion of the full runtime/configuration phase.
[SENSOR_RUNTIME.md](SENSOR_RUNTIME.md) describes the delivered API; the broader
[SENSORS.md](SENSORS.md) requirements still include future camera/stereo work.

## Native profile composition increment

Implemented:

- Versioned scenario/robot/world/sensor documents, declaring-file-relative paths,
  strict validation, full physical matrices, and resolved native model factories.
- Robot-owned sensor mounts/schedules, world-owned pressure conditions, and
  independent pressure-to-depth calibration. Empty sensor lists remain valid.
- Checked typed stream lookup after configuration-based runtime construction.
- Runner execution through the composed runtime, optional named-field sensor CSV,
  and installed reusable robot/world/sensor examples.

[PROFILES.md](PROFILES.md) defines the delivered schema and extension boundaries.
This remains single-vehicle/pool composition; serialized resolved manifests,
content hashes, search roots, visual assets, task packs, and public dynamic plugin
loading are not delivered. The Python increment below now exposes this runtime.

## Python API increment

Implemented:

- Optional pybind11 module and PEP 517 wheel/sdist packaging with typed public API,
  NumPy conversions, bundled profiles, and an installed standalone example.
- Profile-based and programmatic construction; explicit command/advance/reset;
  typed IMU, FOG, DVL, and pressure streams using the existing native scheduler.
- Detached observations, exact integer nanosecond times, independent instances,
  exception translation, and safe stream/data lifetime after runtime destruction.
- Python binding tests, C++ runner comparisons, type/lint checks, source-distribution
  installation checks, and native ASan/UBSan checks under CPython.

[PYTHON.md](PYTHON.md) documents ownership, build dependencies, and current limits.
This is direct simulation access, not a training environment or complete phase 2.
Python callbacks/plugins, concurrent shared-instance calls, checkpoints, and batched
training environments are not implemented. Calls retain the GIL.

## Standalone viewer foundation increment

Implemented:

- Optional native ImGui/GLFW/OpenGL viewer, independently built and installed with
  no simulation, Python, ROS, robot-profile, or course-content dependency.
- Neutral geometry, exact source-clock time, immutable frame histories, checked
  frame trees, interpolation, and explicit missing/out-of-range diagnostics.
- Composed source factories and display functions, separate optional playback,
  generation-aware local rewind/disconnect/reconnect, and independent source state.
- GPU line rendering for grid/frame axes/pose glyphs/trajectories, orbit/pan/zoom,
  fixed-frame selection, local playback controls, display status, and workspace saves.
- Versioned bounded local recordings/workspaces and an installed external source
  and display example. The viewer can also start with an empty workspace.

[VIEWER.md](VIEWER.md) defines delivered contracts, limits, controls, and checks.
This begins phase 5A. The live ROS gate, meshes/joints, images/clouds/markers,
async queues, source overlay alignment, dynamic plugin loading, and arbitrary
workspace layouts are not delivered. Simulation attachment remains a separate
adapter; the viewer has no simulation or robot control capabilities.

## Next increments

1. Complete the behavior/workflow inventory and establish Talos/reference timing
   fixtures, keeping intentional model differences explicit.
2. Connect simulation observations through a separate viewer adapter using the
   delivered neutral contracts. Continue the live-viewer workflow inventory and
   add displays/live transport against explicit requirements.
3. Extend native profile composition with assets/tasks when their implementations
   exist; refine executable camera/stereo contracts when rendering begins.
4. Add optional ROS/UWRT integration, task/mechanism ownership, coordinated general
   contacts, and camera/stereo rendering in their planned phases.

ROS adapters, mesh/sensor rendering, full hull/prop contacts, and competition
scoring are not implemented. RL implementation remains out of scope. The plant is deterministic;
the composed sensor runtime owns seeded measurement noise. Cameras/stereo are planned;
sonar implementation is deferred.

## Release prerequisites

Resolve the numerical-port license metadata in PROVENANCE.md and assign the new
project's license; name maintainers/reporting contacts before public distribution.
Local implementation and verification can continue. Do not treat planned CI jobs
or architecture goals as verified functionality.

## Verification of the initial implementation

Locally verified on Ubuntu 22.04 / aarch64 with GCC 11.4:

- Release: all 34 CTest entries passed, including the CLI contract suite's malformed
  input cases and repeatability checks.
- Address/undefined-behavior sanitizers: all 34 entries passed; the relocated
  installed consumer and runner also passed.
- clang-format 23.1.1 and configured clang-tidy 14 analysis passed for project code.
- A copied source tree outside the release workspace built with CMake 3.22.1,
  `RP_BUILD_CLI=OFF`, and tests disabled. No YAML, Python, GoogleTest, ROS, or old
  simulator package was discovered by that plant-only build. Its installed
  downstream consumer ran successfully.
- Full builds use CMake 3.28.3; installed prefixes were moved before building the
  external consumer, and exported CMake files were checked for source-path leaks.
- The example emits 1,501 snapshots for 3 simulated seconds, with the vehicle
  moving from x=2 m to approximately x=4.61648 m while remaining at z=-2 m.

CI configuration is present but has not run on a remote service. These are local
checks, not a claim of complete phase-0 workflow validation or cross-platform support.

## Verification of the sensor increment

Locally verified on 2026-09-28 in the same Ubuntu 22.04 / aarch64 environment:

- All 56 CTest entries passed in both Release and Address/UndefinedBehaviorSanitizer
  builds, including existing plant/CLI regressions and the new sensor contracts.
- Formatting, dependency checks, and configured clang-tidy analysis passed.
- Relocated installed plant and sensor consumers, plus the CLI, passed in both
  build configurations with the ROS overlay environment removed.
- A copied headless source build with CMake 3.22.1, CLI/tests disabled, built and
  installed both libraries. The sensor example then built and ran against the moved
  prefix after the source copy was removed; it required no ROS, YAML, graphics,
  Python binding, or legacy simulator package.
- Data-only sensor headers compiled with Eigen and the sensor include directory,
  without adding the simulation include directory.

These checks establish the documented synthetic-model behavior and package
boundaries, not hardware fidelity. At that milestone camera/stereo rendering,
Python access, native sensor-profile loading, and the viewer were not implemented.

## Pressure/depth sensor addition

The sensor library now includes a mounted pressure sensor, a hydrostatic environment
provider, separate pressure-to-depth calibration, scalar noise/drift, operating limits,
and pressure/depth uncertainty. The installed sensor example includes it. All 60
Release tests, formatting, configured static analysis, and relocated install checks
passed locally for this addition. The model's limits are in SENSOR_RUNTIME.md.

## Verification of native profile composition

Locally verified on 2026-09-28 on Ubuntu 22.04 / aarch64:

- All 68 CTest entries passed in Release and Address/UndefinedBehaviorSanitizer
  builds, including pressure tests, profile construction/replay, malformed input,
  and the full CLI trajectory/sensor CSV contracts.
- Formatting, dependency checks, and configured clang-tidy analysis passed.
- Relocated installed plant/sensor consumers and both runner scenarios passed
  with the ROS overlay removed. The new external C++ profile consumer also built
  and ran against relocated Release and sanitizer installations.
- The composed three-second example delivered 300 IMU, 150 FOG, 29 DVL, and 60
  pressure readings. Repeated recording produced identical measurements; disabling
  recording preserved the trajectory. Pending final DVL data was not mislabeled
  as delivered after the configured run ended.

Remote CI has not run. These remain synthetic-model and local packaging checks,
not hardware calibration or full phase-2 completion.

## Verification of the Python increment

Locally verified on Ubuntu 22.04 / aarch64, GCC 11.4, CPython 3.10.12, NumPy 2.2.6,
pybind11 3.1.0, and scikit-build-core 1.0.3:

- All 68 existing CTest entries and relocated C++ installation checks passed.
- Eleven installed Python tests passed in Release and native ASan/UBSan builds.
  They include all trajectory states and sensor numeric fields from the C++ runner;
  cross-build floats use 1e-12 relative/absolute tolerance, with exact timestamps
  and within-build seeded replay. No sanitizer diagnostics were reported.
- The release and sanitizer wheels built from sdists outside the checkout, then ran
  in fresh environments after removal of copied source/build trees. The bundled
  example delivered its configured samples and reset successfully.
- Ruff, strict mypy, C++ formatting/dependency checks, and configured clang-tidy on
  native bindings passed. The headless C++ preset still built with Python/CLI off.
- LeakSanitizer is disabled specifically for the CPython embedding check; address
  and undefined-behavior checks remain enabled. This does not claim process leak
  checking or cross-platform/free-threaded Python support.

CI jobs are configured but have not run remotely. Python wheels remain local,
platform-specific artifacts with system yaml-cpp linkage; nothing was published.

## Verification of the viewer foundation

Locally verified on 2026-09-28 on Ubuntu 22.04 / aarch64, GCC 11.4, GLFW 3.3.6,
GLEW 2.2.0, Dear ImGui 1.91.9b, and the Apple M1 Pro (G13S C0) graphics driver:

- All 15 viewer contract tests passed in Release and ASan/UBSan builds. Coverage
  includes transform composition/interpolation, exact large integer times, tree
  validation, history limits, missing/stale data, source isolation, rewind/removal,
  extension errors/budgets, relative paths, atomic save failure, and camera math.
- Release and sanitizer installations built from copies physically omitting all
  simulation/Python sources and robot/world content. After removal of copied
  source/build trees, relocated libraries compiled and ran an external source and
  display consumer; installed GUI runs captured empty, initial, and advanced views.
- Configured clang-tidy, formatting/dependency scans, and the viewer checker's Ruff
  checks passed. All 68 existing simulation tests and relocated simulation/sensor/
  profile consumers passed in Release after the optional build-graph changes.
- Neutral visualization/IO libraries built with OpenGL, GLFW, and GLEW discovery
  explicitly disabled. The normal simulation build still requires no graphics.
- A desktop interaction check exercised exact nanosecond entry, play/pause, camera
  keyboard controls, display visibility, and Save as. Inspection of the saved file
  confirmed camera/display changes and correctly rebased source paths.
- A local Release CPU-only scene-construction measurement (100 iterations, one
  root frame and one trajectory) averaged about 0.011 ms for 61 samples and 1.61 ms
  for 10,000 samples. This excludes graphics/UI work and is not a frame-rate or
  general workload guarantee; the UI currently constructs the scene twice per frame.
- All 16 vendored ImGui files were byte-compared to the upstream v1.91.9b archive;
  its archive checksum is recorded in third_party/imgui/README.md.

No sanitizer diagnostics were reported in the final runs. The debug graphics run
also exercises ImGui assertions. GUI checks disable LeakSanitizer for external
OpenGL driver process globals; CPU tests and installed extensions retain it.
Remote CI/Xvfb/Mesa jobs are configured but have not run remotely. This is local
hardware and synthetic-data validation, not full RViz workflow parity or a
cross-platform graphics-support claim. Nothing has been published.

## Read-only live visualization transport

Implemented a neutral bounded live-pose source and an optional simulation adapter
that consumes snapshots without owning or advancing a runtime. Producer/reset
identity, stale rejection, queue/history loss, immutable retained observations,
disconnect, and reconnect are explicit. See [LIVE_VISUALIZATION.md](LIVE_VISUALIZATION.md).

Locally validated: all 90 combined native tests passed in Release and ASan/UBSan;
formatting/dependency scans, configured clang-tidy, and relocated installed
consumers (including the new adapter) passed. All 20 viewer contracts and an
isolated relocated viewer install also passed without simulation source files.
The new comparisons require exact matching trajectory states and noisy IMU samples
across viewer polling, overflow, disconnect, reset and reconnect. A concurrent
producer/consumer functional test and locking review passed; this is not a
ThreadSanitizer or hard-real-time guarantee. No sanitizer diagnostics were reported.

This increment is pose transport, not a connected simulation application or Talos
fidelity. Application composition is next; physics, scene rendering, mechanisms,
independent task/scoring composition and full-stack gates remain open in the ledger.

## Composed simulation viewer application

`robotics-sim-view` now composes the standalone desktop with an independent,
wall-paced scenario worker and read-only pose source. Sensors drain on the worker
regardless of display state; normal close cancels/joins; worker failures propagate.
The shared desktop accepts explicit source/display/workspace composition and still
builds without simulation. Reopening a retained endpoint preserves its connection;
removing it disconnects presentation. Recordings retain their own playback clock.

Validation: all 93 combined Release tests passed; eight changed live-source/worker
contracts passed under ASan/UBSan. The synthetic configured scenario's final state
matched synchronous execution exactly despite unpolled bounded delivery. Cancellation
and error propagation passed. Configured static analysis and formatting/dependency
checks passed; static analysis caught and corrected a C++17 lambda portability issue.
The isolated viewer install and its graphics captures passed. The composed application
ran and captured its body/trajectory through a relocated installation with the ROS
environment removed. Its sanitizer graphics run reported no diagnostics (driver leak
checking disabled, as for the standalone viewer). Screenshots were inspected; an
initial missing display-source binding was corrected before final verification.

This remains a finite scenario pose viewer. Simulation controls, Talos physics,
water/assets, independent mechanisms/tasks/scoring, camera acquisition, and ROS
integrations are still open. Next is fixed-input reference capture and stage-dependent
thruster submersion/current fidelity, not a claim of original simulator equivalence.

## Stage-dependent propulsion and current fidelity

Optional robot propeller disk immersion and world sinusoidal current now feed every
RK4 stage; endpoint kinematics use the same forcing. The numerical integrator is
shared with fixed-input model tests. Native profiles and Python expose the model
choices; absence of immersion keeps generic thrust unmodulated. Full reset restarts
current phase and batch advancement matches individual steps exactly.

The offline capture tool compiled pinned original kernels and extracted original
propulsion/current/RK4 expressions. All 753 fixture states (surface, submerged, and
above-water entry with rotating body and force reversal) match 20 numerical fields
within 2e-12 absolute tolerance. Original source, driver and fixture hashes are
recorded. See [reference/STAGE_DYNAMICS.md](reference/STAGE_DYNAMICS.md).

Validation: all 72 Release tests, formatting/dependency checks, configured static
analysis and relocated installed consumers passed. ASan/UBSan passed the 71-test
suite and relocated consumers, then all three stage tests after the added batching/
reset regression. The isolated installed Python wheel passed all 12 tests plus Ruff
and strict mypy, including the new optional-immersion/current behavior. No sanitizer
diagnostics were reported. Independent read-only review approved the reference
capture and port for this bounded scope.

This does not establish full Talos fidelity. The original's accumulated floating
clock may diverge slightly from tick-based flow phase on long runs; preserving that
incidental drift is not the new clock contract. Actuator calibration/stop interfaces,
COM/base/CAD conversions, contacts and native Talos configuration are next.

## Actuator calibration and explicit stop

Native thruster configuration now exposes command deadband, forward/reverse scale
and efficiency through C++, profiles and Python. The existing numerical actuator
kernel retains calibration order and validation. Plant/runtime `stopThrusters()`
(and Python `stop_thrusters()`) clears delayed commands and targets, preserves
realized force until coast-down, and leaves simulation/sensor clocks unchanged.
It deliberately does not own robot kill/arming policy or reject later commands.

Validation: all 76 Release and ASan/UBSan tests passed. The stop/sensor-phase test
was strengthened after review to stop at62ms, between acquisitions; its updated
Release and sanitizer checks also passed. The installed Python wheel passed13
checks including calibration and delayed-stop behavior, plus Ruff/strict mypy.
Configured native static analysis, formatting/dependency scans and relocated C++
consumers passed. No sanitizer diagnostics were reported.

The reviewed [Talos content conversion map](reference/TALOS_CONTENT_CONVERSION.md)
records source revisions, physical parameters, effective collision proxies, and
startup-versus-placement frame conventions. Compound box contacts and world/body
frames remain prerequisites for a faithful competition profile. The user's added
performance constraint uses the current machine as a provisional baseline; collect
headless costs now and repeat for contacts, Talos and the full renderer as delivered.

## Replacement priority and visual acceptance clarification

The user explicitly prioritizes replacing the old simulator, then expanding RViz
replacement use, then broader team/industry reuse. Existing general dependency and
ownership constraints remain mandatory. The acceptance target includes the exact
Talos mesh/materials, moving thrusters/rotors, LEDs and other status/mechanism visuals,
and the same RoboSub 2026 pool/task/prop models and water/UI behavior. These are
recorded as open checks, not implemented features. Subsequent work should close
those concrete replacement gaps rather than add unrelated generic viewer features.

## Provisional headless performance baseline

An optional Release benchmark now measures plant and plant-plus-scheduled-sensor
cost independently of rendering, output and ROS. The recording tool retains
machine/build/content/executable identity and rejects mismatched final trajectory
checksums. See [PERFORMANCE.md](PERFORMANCE.md) and its checked JSON record.

On this Apple/aarch64 machine, 1.5 million measured ticks per mode gave plant p99
1.334 microseconds and plant+IMU/FOG/DVL/pressure p99 2.917 microseconds. The latter
completed 3,000 simulated seconds in 2.851 measured seconds. These are repeated
synthetic scenarios with construction excluded, not Talos/contact/task/camera or
full-stack results. Maximum observed tick latency was also recorded; no hard-real-time
or other-hardware guarantee is implied. Release compilation, configured static
analysis, formatting and the recording tool's lint/checksum checks passed.

## Static compound-box numerical extraction

A private static-box contact component now preserves the original SAT, contact-point,
depenetration, coupled-mass normal impulse and friction sequence. Robot/world proxy
values and ordering are explicit; static geometry is cached and per-call geometry
uses fixed-size Eigen temporaries. No ROS/task names or dynamic prop ownership enter
this solver. It is not yet selected by Plant or profile configuration.

Eleven cases captured from actual pinned original equations compare all state fields
within 2e-12, including separating contacts, disjoint geometry, compound/local/world
rotation and nonunit stage quaternions. All 78 Release tests passed; the two new
contact tests passed under ASan/UBSan and after final geometry validation changes.
Formatting, configured static analysis and independent numerical review passed.
See [reference/BOX_CONTACTS.md](reference/BOX_CONTACTS.md) for hashes and limits.

Next: expose explicit disabled/sphere-pool/compound-box contact selection, keep body
and world geometry in their owning profiles, and validate original pre/post-RK4
ordering through whole-step comparisons. Dynamic task/prop contacts remain later
runtime-owned work, and full-scene performance still requires measurement.

## Selectable contacts through the public runtime

Plant, native profiles and Python now select disabled, sphere-pool or static
compound-box contacts. Robot/world content owns ordered proxies. Box scenes allow
initial depenetration, skip unrelated sphere containment constraints, and preserve
original pre/post-RK4 resolution and quaternion-normalization timing. The new
`contact_pool.yaml` exercises a hull against finite pool boxes with four sensors.

Validation: all 82 native tests passed in Release and ASan/UBSan, plus relocated
installed consumers in both builds. Release configured static analysis and
formatting/dependency checks passed; the installed Python wheel passed all14 tests,
Ruff and strict mypy. After adding the unoptimized reference variant, the four
changed Release contracts passed again. No sanitizer diagnostics were reported.

Eleven single-contact responses match within2e-12 and550 whole-step states match
one complete original trajectory within1e-9. Independent review reproduced the
original's own optimization-dependent divergence in one late repeated-contact case;
the new debug trajectory equals the original unoptimized trajectory exactly.
Both original captures are retained with flags/hashes; tolerance was not widened.
See [BOX_CONTACTS.md](reference/BOX_CONTACTS.md) for scope and acceleration semantics.

The same-machine contact benchmark recorded plant p99 3.25 microseconds and
plant-plus-sensors p99 4.75 microseconds over1.5 million ticks each. This synthetic
one-hull/five-box scene is not full Talos/task/graphics performance acceptance.
Dynamic prop contacts, transformed sensor-query geometry, named body frames,
native Talos content and original rendering remain open and drive the next steps.

## Shared rigid frames and resolved sensor mounts

A small Eigen-only spatial library now supplies poses and immutable named rigid
frames to configuration and visualization independently. Robot profiles can resolve
sensor `mount_frame` references from a COM-rooted tree, including CAD/base/device
chains. Python exposes detached values. Live pose sources carry immutable static
mounts with the same body history; the simulation viewer supplies resolved robot
frames. No simulator or robot-specific dependency enters spatial/viewer libraries.

Validation: all114 combined Release tests and111 simulation-view ASan/UBSan tests
passed. Installed Python checks passed all15 tests plus Ruff/strict mypy. Configured
native static analysis, formatting/dependency checks and relocated installed
consumers passed. The24-test isolated viewer build/install and graphics captures
passed with simulation sources absent; the composed viewer also ran with an
explicit frame tree and its screenshot was inspected. No sanitizer diagnostics
were reported.

Review corrected accepted quaternion rounding before sensor attachment. Tests cover
rotated chained mount equivalence, source deletion, invalid trees, detached ownership,
reset/reconnect, and Talos CAD/base/COM offset algebra. Actual Talos content/rendering
is still open. See [FRAMES.md](FRAMES.md) for the current graph limits. Next: scenario
placement of reusable pool geometry and its sensor queries, then native Talos
physical content and the audited original visual resources.

## Reusable world placement and consistent pool queries

Schema2 scenarios now place world geometry through an upright position/yaw,
independently of robot content and reusable pool files. Resolution transforms static
boxes once and moves water level once; world current vectors retain their declared
world axes. The same resolved footprint drives sphere containment/contacts and DVL
floor queries. C++/Python expose pool corner and heading directly.

All93 Release tests passed with configured static analysis, formatting/dependency
checks and relocated installed consumers. The91-test ASan/UBSan suite passed,
followed by allfive placement tests after exact-boundary regressions were added.
The final installed Python wheel passed16 tests, Ruff and strict mypy. No sanitizer
diagnostics were reported.

Independent review caught inverse-transform roundoff at legal wall/floor edges;
scale-aware rounding tolerance now preserves those boundaries while tests reject
meaningfully exterior origins and projected ray hits. Additional checks compare
rotated corner dynamics, finite ray availability/range, pressure under Z translation,
static-box composition and independent loading without robot/current mutation.
This enables the original competition pool-to-map placement without embedding2026
layout into reusable pool data. Native Talos content and its original sensor/reporting
policies remain the next fidelity work; full-stack/rendering gates remain open.

## Native Talos dynamics and pool content

A pinned offline importer now emits native Talos mass/hydrodynamics, eight ordered
calibrated thrusters, effective hull/poker proxies, and COM/CAD/base/device mounts.
The original finite competition pool remains a separate world; the example owns
its2026 map placement and a new scripted force schedule. Inputs and generated files
have recorded revisions/hashes; the runtime has no importer or original-workspace
dependency. See [TALOS_PHYSICS_PACK.md](reference/TALOS_PHYSICS_PACK.md).

All94 Release tests and relocated installed consumers passed. The new Talos loading,
source-removal and1500-step independent replay test passed under ASan/UBSan. The
installed Python wheel passed17 tests, Ruff and strict mypy, including loading and
running Talos from installed package data. Importer byte reproduction and read-only
conversion review passed. No sanitizer diagnostics were reported.

The original physical content has no instantiated devices yet: this pack explicitly
uses sensors=[] and mechanism/camera mount frames only. It is not the complete
robot/mission/renderer. The finite rollout proves execution, not original-trajectory
parity; that independent numerical reference is the next check.

A same-machine1.5-million-tick benchmark measured plant p99 6.791 microseconds;
the runtime-wrapper mode p99 was5.5 microseconds with zero scheduled devices.
The wrapper completed3000 simulated seconds in6.872 measured seconds. This excludes
camera/renderer/task/ROS costs and does not establish full-stack performance.

## Independent original Talos dynamics reference

The native Talos pack now matches an independently built pinned-original reference
at every tick of three 1500-step cases: startup pool contact, moving partial
immersion and floor approach. All 4503 states compare 27 values, including the
eight realized thruster forces and COM/angular accelerations, with a maximum
absolute error tolerance of1e-9 against one complete original compiler candidate.
The capture reads original Git objects independently of the native importer. See
[TALOS_DYNAMICS.md](reference/TALOS_DYNAMICS.md) for reproduction and scope.

All95 Release tests, formatting and dependency checks passed. The expanded
Talos comparison also passed under ASan/UBSan without diagnostics. Independent
read-only review verified the every-tick coverage, complete-candidate comparison,
finite values and recorded source/driver/capture/fixture hashes. No production
or binding code changed in this increment.

This closes the bounded assembled-physics comparison, not full simulator parity.
Sensor reporting, original rendered assets/animation (including LEDs), mechanisms,
tasks and full-stack operation remain open, in that simulator-first order.

## Explicit raw IMU reporting calibration

`ImuReporting` now provides optional measurement-only gravity magnitude and
independent sensor-axis acceleration/angular variances through C++, native sensor
profiles and Python. This expresses the original Talos gravity calibration and
noise/covariance distinction without changing physical gravity or inventing an
attitude estimate. Omitted settings preserve existing raw IMU behavior.

All 100 Release tests, installed C++ consumers and all 100 ASan/UBSan tests passed
without sanitizer diagnostics. The installed Python wheel passed 18 tests, Ruff
and strict mypy after deleting its build/source copies. Tests cover rotated mounts,
lever-arm acceleration, calibrated freefall residual, non-Z gravity, independent
reported covariance, unchanged noise/reset sequences, validation and copied
configuration ownership. Independent read-only implementation review found no
blocking issues.

The [Talos sensor conversion map](reference/TALOS_SENSOR_CONVERSION.md) records
remaining attitude composition, DVL validity, FOG uncertainty and pressure/base-link
depth policies. Native Talos still has no instantiated devices; full sensor products,
original visuals/LEDs/mechanisms and mission/stack acceptance remain open.

## Separate attitude and composed AHRS acquisition

The explicit `Attitude` model now reports sensor-to-world orientation using the
original random-axis/scalar-angle noise order and world-axis heading drift. `Ahrs`
composes it with raw IMU using one mount, one acquisition state and one header.
Independent random streams preserve the inertial sequence; both components advance
even when the composite is unavailable. C++, native profiles, typed CSV telemetry,
installed Python and downstream C++ consumers expose the owned typed contracts.

All 106 Release tests and installed C++ consumers passed. The full 106-test
ASan/UBSan suite passed without diagnostics. The installed Python wheel passed
19 tests, Ruff and strict mypy. Targeted clang-tidy reported no user-code warnings.
Independent review found no blocker and prompted a radial fourth-moment noise test
that distinguishes the original scalar-angle distribution from Gaussian-vector
noise. The strengthened distribution check and all six attitude/composition
checks passed again under ASan/UBSan after the test was added.

Raw IMU remains orientation-free, and no estimator or ROS dependency was introduced.
The next increment assembles the pinned native Talos inertial devices. Remaining
DVL/pressure reporting, cameras, original visuals/LEDs, mechanisms and full-stack
mission acceptance remain open.

## Native Talos inertial device assembly

A separately generated Talos inertial robot/example now instantiates the pinned
50 Hz AHRS and 500 Hz FOG, including original CAD mounts, SI noise conversions,
IMU gravity calibration and reported uncertainty. The reusable AHRS profile and
all new input/output hashes are captured by the offline importer. The existing
sensor-free physics profiles are unchanged. Neither assembly requires the original
workspace or ROS. See [TALOS_PHYSICS_PACK.md](reference/TALOS_PHYSICS_PACK.md).

All 107 Release tests and relocated installed C++ consumers passed. The new
1,500-tick assembly test passed under ASan/UBSan without diagnostics, proving
source removal, unchanged plant trajectory, 150 AHRS/1,500 FOG acquisitions and
reset replay. The installed Python wheel passed 20 tests, Ruff and strict mypy.
The CLI also completed the full run with those sample counts and no invalid
readings. Importer byte verification and independent read-only source/data review
passed. The preceding model increment passed the complete 106-test sanitizer suite.

On the provisional baseline machine, 1.5 million measured ticks per mode gave
plant p99 5.417 microseconds and plant-plus-inertial-acquisition p99 6.459 microseconds.
The latter completed 3,000 simulated seconds in 8.075 measured seconds; trajectory
checksums matched. See [PERFORMANCE.md](PERFORMANCE.md) for the record and limits.

This is native inertial assembly, not independent original sensor-output parity.
Next: capture original inertial reporting, then resolve DVL/pressure policies and
complete native devices before original scene/asset integration. Cameras, water
rendering, LEDs/mechanisms, task/scoring and full robot-stack acceptance remain open.

## Independent original inertial reporting reference

A pinned-source capture now records 24 prescribed COM states/velocity derivatives
and original noise-disabled IMU, FOG, DVL and depth reporting expressions. The
AHRS/FOG model comparison verifies all 21 corresponding output fields within
1e-12, including freefall residual, rotating lever arms, mounted orientation and
reported uncertainty. Source/input/output provenance is recorded independently
of native profiles/importers. [SENSOR_KINEMATICS.md](reference/SENSOR_KINEMATICS.md)
identifies scope; DVL/depth outputs are captured acceptance inputs, not yet gates.

FOG now supports explicit sensor-axis reported variance independently of sampling
noise, before configured axis projection. C++, profiles and Python preserve
nonorthogonal-axis correlations and noise sequences. Native Talos explicitly
selects the original reported FOG variance, including for noise-disabled sampling.

All 110 Release tests and relocated C++ consumers passed. Six focused FOG/Talos
checks passed under ASan/UBSan without diagnostics, and the installed Python wheel
passed 20 tests, Ruff and strict mypy. Independent read-only source/implementation
review verified provenance, formula extraction and kinematic conventions and
prompted an explicit finite-output assertion in the reference test.

The fixture comparison configures original reporting values explicitly; native
profile/settings assembly has separate tests. It does not establish acquisition,
whole-trajectory, stochastic-sequence or ROS message parity. Remaining DVL/depth
reporting models now have source-derived comparison inputs for the next increment.

## Declared reference velocity and Talos DVL assembly

`ReferenceVelocity` now observes mounted velocity relative to a declared constant
world velocity, with independent noise/reported uncertainty and optional inclination
validity. It has no range or bottom query. The existing finite-floor DVL remains
unchanged. The pinned `talos_navigation` robot/example selects the original default
8 Hz DVL policy alongside AHRS/FOG. Its CLI run produces 150 AHRS, 1500 FOG and
24 velocity acquisitions with declared timestamps and no invented range fields.

The independent original measurement fixture now compares 25 AHRS/FOG/DVL fields
across all 24 prescribed states within 1e-12. Moving-reference, lever-arm, covariance,
invalid-input, noise-history, source-removal and reset contracts are tested. Review
found a zero-inclination roundoff rejection; normalized atan2 and a documented
16-epsilon angular allowance now preserve exact alignment and reject 1e-8-radian
exterior cases. The importer reproduces all ten pinned generated files.

All 117 final Release tests and installed C++ consumers passed. The preceding
116-test full sanitizer run passed, followed by all eight focused velocity/navigation
and reference tests under ASan/UBSan after the boundary fix. No sanitizer diagnostics
were reported. The final installed Python wheel passed 21 tests, Ruff and strict mypy.
Independent source/data/implementation review confirmed the conversion and fix.

Next is original depth reporting: an explicitly selected ideal-altitude observation
with same-acquisition mounted-to-base correction. Physical pressure remains a separate
truthful pressure-device model; unbounded original world-Z cannot be preserved by
extrapolating absolute pressure through negative values. Depth reference columns
remain unverified until that model lands. Cameras, scene/water/LED/mechanism/task
implementation and full-stack acceptance remain open.

## Original depth reporting and complete prescribed sensor comparison

The selectable `ReferenceAltitude` model observes mounted world Z and applies a
configured COM-local target correction using the same acquisition pose. Both
outputs share one scalar noise observation and variance; physical pressure remains
separately selectable. C++, profiles, CSV and copied Python products are delivered.
The pinned navigation assembly now includes original 20 Hz depth reporting with
.010 m noise and .0001 m² reported variance. Its three-second run acquires 150 AHRS,
1500 FOG, 24 DVL and 60 depth samples without changing the physical assembly.

All 28 captured original noiseless sensor outputs match across 24 prescribed
states, including mounted/base height above and below water. This closes the
captured formula comparison, not full trajectory, stochastic-sequence, ROS or
mission acceptance. Independent read-only review found no actionable defects.

Next: original Talos body/rotor assets and a neutral CPU loader, then extract the
full original scene/shadow/water/reflection/bloom/post renderer for fixed-input
comparisons. Camera acquisition follows that renderer. LEDs, mechanisms, course
assets/tasks, original operator panels and full RepairCompTree remain required.

Validation: all 122 Release and ASan/UBSan tests passed, relocated installed C++
consumers passed, and the installed Python wheel passed all 22 tests plus Ruff and
strict mypy. The pinned importer reproduced all ten files. No ROS environment or
original runtime files were required by the installed consumers.

## Original Talos body/rotor assets and independent CPU loading

The optional `mesh_assets` library loads owned geometry/material data through
private Assimp, with no GL, simulation, YAML, ROS or viewer dependency. It preserves
node/submesh order, authored units/axes, inverse-transpose normals and opacity,
checks input/output limits, and computes indexed bounds. External texture paths
are references only; texture resolution/decoding remains future renderer work.

The exact pinned original Talos body/eight-rotor GLBs are now an optional native
resource pack with per-file hashes, original conversion metadata and a transport-free
inventory of precise pivots/axes/order/RPM settings. It contains 1,087,230 triangles.
Launcher, payloads, claw, magnet, LEDs, animation and complete assembly remain open.

An independent capture of the unchanged original loading method verifies all 37
submeshes' order, counts, index hashes, RGBA and geometry statistics. Synthetic
nested transforms verify nonuniform scale, normal transformation, alpha and UV
conventions. Both Release and ASan/UBSan asset suites passed (seven tests each),
as did relocated installed consumers loading all nine meshes after removal of the
copied source/build trees. Hash/import checks passed for all 15 visual files.

Default simulation checks still pass all 122 tests and installed consumers. The
installed Python wheel passes all 22 tests, Ruff and strict mypy; its source archive
and wheel exclude the optional visual pack. Standalone viewer contracts and its
isolated installed extension pass with mesh loading disabled; no new graphics
appearance test is claimed. The asset CI matrix is configured but has not run remotely.
Independent read-only review found no blocking defects in the body/rotor scope.

Next: expose neutral scene instances/appearance and extract the entire original
scene/shadow/water/reflection/bloom/post pass graph for fixed Talos-body/pool RGB
and depth comparisons. That renderer must own no window or simulation and later
serve camera acquisition as well as the viewer. Mechanisms/LEDs and course assets
follow; original UI/ROS panels and full physical RepairCompTree remain required.

## Original water rendering behind an independent scene API

The optional `scene_rendering` library now accepts explicit immutable mesh instances,
view/projection matrices, appearance, water and time. It owns GPU resources in a
caller-owned context, with no window, simulation, viewer, clock or ROS dependency.
The original shadow, reflection, HDR scene, water, bloom and post-processing passes
are retained. Water is optional; ordinary live scenes require no pool or water
shading. Original GLM is private and licensed; public types remain Eigen/std.

Six fixed body/rotor/pool views exercise above/below water, close transparent hull,
near-surface, indoor/outdoor, reflection and shadow settings. All 30 final RGBA,
opaque HDR/depth and composite HDR/depth buffers match the independently compiled
pinned original byte for byte on this machine, in Release and ASan/UBSan builds
against their corresponding original optimization modes. Visual inspection also
confirmed nonempty Talos/pool/water imagery. Original float normalization order is
retained because changing it affected sparse shadow decisions in debug builds.

Both renderer configurations pass seven CPU tests and isolated relocated install
checks after their copied source/build trees are removed. GPU checks cover hostile
caller state, resize/replay, explicit time, absent water, invalid-plane rejection
and recovery. Default headless Release still passes 122 tests and installed
consumers. Rendering source clang-tidy checks passed. A Mesa CI matrix is configured
but has not run remotely; no cross-driver pixel equality is claimed.

The current-machine 1280×800 benchmark averages 8.246 ms per GPU-complete draw versus
8.364 ms for the original (p99 14.639 vs 14.501 ms). These are provisional body/pool
measurements without UI, cameras/readback, textured tasks or ROS. See
[RENDERING.md](RENDERING.md) and the recorded source/output hashes and timings in
[rendering_baseline.json](reference/rendering_baseline.json).

Next: connect scene rendering to the interactive viewer using neutral scene/source
data; the existing viewport still draws lines. Complete Talos rotor animation,
LEDs, mechanisms and original course content afterward. Per-view targets and shared
scene preparation must precede camera integration. Original simulator replacement
remains the first acceptance priority, followed by RViz expansion and wider reuse;
this fixed scene does not close full assembly, task, UI or robot-stack acceptance.


## Interactive scene rendering from neutral source frames

The optional scene-viewer build now presents the original mesh/pool/water renderer
inside the existing native viewer. Versioned scene documents contain independently
bound groups with relative mesh resources, materials and an optional pool. Dynamic
groups require an explicit source identity and resolve at one frozen snapshot shared
with line displays; static groups require their declared fixed frame. Missing frames,
source changes and disconnects omit groups rather than retaining an old pose.

The desktop copies scene color and composite depth into its own framebuffer before
overlays. Graphics checks verify unchanged background color, floor occlusion,
visible foreground lines, resizing and unchanged renderer captures. The pool's
lighting center remains independent of orbit/follow target. Camera view/projection
and eye are available separately through the neutral visualization API.

The supplied original Talos body/eight-rotor/pool document works with a local recording
and with the existing scripted simulation source. Workspace scene paths survive
Save As/relocation. Explicit Reload scene refreshes assets; loading errors appear in
the UI and never substitute stale meshes. The default line-only build reports an
unavailable scene feature without acquiring mesh/scene dependencies.

Validation: both Release and ASan/UBSan scene configurations pass 31 CPU tests,
relocated standalone installs and external installed consumers after removal of
copied source/build trees, plus hidden-window graphics contracts/captures. The
combined simulator/viewer passes 155 tests and a 200-frame Talos navigation run;
its final screenshot shows the model and original water at source time 3 seconds.
Recorded and live screenshots were inspected. The baseline line viewer's contracts,
relocated consumer and graphics captures still pass. Changed C++ implementation
files pass clang-tidy. CI coverage is configured but has not run remotely.
The renderer regression still matches all 30 original reference buffers exactly
after exposing its borrowed depth texture for the viewer.

This closes the initial interactive renderer connection, not original UI/assembly
or whole-stack acceptance. Next are original rotor motion, LED state and complete
mechanism/course assets, then the remaining camera/control/transport/task gates.
Per-view render targets and shared scene preparation still precede camera providers.
See [SCENE_VIEWER.md](SCENE_VIEWER.md) for commands, schema and current limits.


## Original rotor and indicator state models

`RotorAnimator` now preserves the original signed RPM fit, deadband, handedness,
replacement-time integration, sample timeout, phase wrapping and clock rewind.
`Indicator` preserves source-clock flash/breath phases, clamped RGB and temporary
pulse override/replacement with exclusive expiry. Both own their state and accept
explicit times; neither reads a clock, renderer, ROS message or robot identifier.
Invalid input is rejected before mutation. Explicit generation reset clears state.

Independent pinned-original captures compare all eight rotor phases/matrices over
169 timed events and all 402 indicator output colors. Phase tolerance is 1e-10 rad;
local matrices compare at 1e-6 because native Eigen/double spatial transforms differ
from original float GLM arithmetic. This does not claim byte-identical animated
pixels. Color channels match at float precision. Additional tests cover timeout
preservation after invalid packets, reset, independent instances and shaft geometry.

Release and ASan/UBSan viewer suites pass 29 tests and relocated installed consumers,
including the new models without a graphics context or simulation. Independent
review found no blocking correctness issue. The current interactive example still
has stationary rotors and no LED assembly: next is source-owned animation delivery
and generic scene bindings, followed by original LED geometry/radiance and animated
render comparisons. Delivery must integrate every force sample independently of
lossy display queues. See [ANIMATION.md](ANIMATION.md) for the ownership contract.

## Source-owned moving rotors

The live source now accepts complete body/moving-frame batches at one source time.
All poses are validated before delivery; queue overflow, producer reset and
reconnect preserve complete batches. Pending/drain/latest storage is preallocated,
and immutable frame histories are rebuilt outside the producer lock.

A neutral named-input rotor rig contains the original force/RPM curves, pivots,
axes and directions. The offline Talos importer now reproduces this rig alongside
the unchanged meshes and records its hash. The simulation adapter maps plant
channels by ID and processes every observation before lossy delivery. Consecutive
ticks are required while animating; generation changes clear phase. Preallocated
staging preserves accepted state after invalid forces, geometry or resets.

`robotics-sim-view --rotors` composes this provider with the new animated Talos/pool
scene. All eight original meshes follow their individual moving frames. The
standalone recording and body-only examples remain available. Shader/render-pass
behavior is unchanged. No robot identity, force integration, ROS interface or plant
configuration was added to the renderer or standalone viewer.

Validation includes named-channel reorder, queue loss/reconnect/reset equivalence,
malformed-batch atomicity, retained snapshot isolation, topology/history bounds,
error recovery, and exact unchanged dynamics/noisy sensors with animation attached.
The actual Talos scenario verifies every rotor moves about its original shaft and
resolves through world/COM/CAD frames. A 200-frame live scene run reaches three
seconds; its captured original body/pool/water image was inspected. This is not a
pixel-equality claim for animated rotor poses. Original phase/matrix references
remain the numerical fidelity evidence.

Release and ASan/UBSan combined builds pass 168 tests and relocated installed
consumers, including animated-frame delivery through the installed API. The
standalone viewer passes 32 tests and its isolated relocated consumer. Changed
implementation files pass clang-tidy; the importer verifies all 16 pinned outputs.
Read-only review found no remaining blocker in this increment.

Next: generic source color bindings and the original LED bars/material/radiance,
including independent original image comparisons, then remaining mechanisms and
course content. LED ROS routing, full operator workflows, camera acquisition and
physical RepairCompTree acceptance remain open. The replacement priority and
independent robot/world/task/scoring boundaries are unchanged.
