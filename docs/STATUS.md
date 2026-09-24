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

## Next increments

1. Complete the behavior/workflow inventory and establish Talos/reference timing
   fixtures, keeping intentional model differences explicit.
2. Extend native profile composition and the runtime/Python binding, sensor
   scheduling, seeded noise, and complete lifecycle contracts (phase 2).
3. Build the independent viewer/source/display contracts and a minimal local-data
   viewer (phase 5A), without waiting for ROS or competition tasks.
4. Add optional ROS/UWRT integration, task/mechanism ownership, coordinated general
   contacts, and simulation camera rendering in their planned phases.

The viewer, rendering, ROS adapters, full hull/prop contacts, sensors, competition
scoring, and Python simulation API are not implemented. RL implementation remains
out of scope. The current plant has no stochastic model and therefore no seed API.

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
