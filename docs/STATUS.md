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

## Next increments

1. Complete the behavior/workflow inventory and establish Talos/reference timing
   fixtures, keeping intentional model differences explicit.
2. Extend native robot/world/sensor profile composition and add the Python binding
   over the explicit runtime lifecycle (remaining phase 2). Refine executable
   camera/stereo contracts when rendering integration begins.
3. Build the independent viewer/source/display contracts and a minimal local-data
   viewer (phase 5A), without waiting for ROS or competition tasks.
4. Add optional ROS/UWRT integration, task/mechanism ownership, coordinated general
   contacts, and camera/stereo rendering in their planned phases.

The viewer, rendering, ROS adapters, full hull/prop contacts, competition scoring,
native sensor YAML loading, and Python simulation API are not implemented. RL
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
boundaries. They do not establish hardware fidelity or implement camera/stereo
rendering, Python access, native sensor-profile loading, or the independent viewer.

## Pressure/depth sensor addition

The sensor library now includes a mounted pressure sensor, a hydrostatic environment
provider, separate pressure-to-depth calibration, scalar noise/drift, operating limits,
and pressure/depth uncertainty. The installed sensor example includes it. All 60
Release tests, formatting, configured static analysis, and relocated install checks
passed locally for this addition. The model's limits are in SENSOR_RUNTIME.md.
