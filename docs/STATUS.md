# Implementation status

This file distinguishes delivered code from the architecture's future work.

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
loading are not delivered. Python access is the next runtime increment.

## Next increments

1. Complete the behavior/workflow inventory and establish Talos/reference timing
   fixtures, keeping intentional model differences explicit.
2. Add the Python binding over the explicit runtime lifecycle (remaining phase 2).
   Extend profile composition with assets/tasks when their implementations exist;
   refine executable camera/stereo contracts when rendering integration begins.
3. Build the independent viewer/source/display contracts and a minimal local-data
   viewer (phase 5A), without waiting for ROS or competition tasks.
4. Add optional ROS/UWRT integration, task/mechanism ownership, coordinated general
   contacts, and camera/stereo rendering in their planned phases.

The viewer, rendering, ROS adapters, full hull/prop contacts, competition scoring,
and Python simulation API are not implemented. RL
implementation remains out of scope. The plant remains deterministic without a
seed API; the composed sensor runtime owns seeded measurement noise. Cameras and
stereo cameras are planned; sonar implementation is deferred.

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
