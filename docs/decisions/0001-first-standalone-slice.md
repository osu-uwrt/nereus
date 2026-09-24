# 0001: A small standalone plant before the runtime and viewer

Status: accepted for the initial implementation; revisit with sensor/task work.

Use a C++17 plant composed from concrete numerical components, with immutable
validated parameters and value snapshots. Hide adapted kernel types behind the
implementation boundary. Keep YAML and filesystem access in a separate target;
applications own IO. Introduce abstract provider interfaces when the viewer/task
substitution boundary is implemented, rather than manufacturing them for every
current component. The plant itself has no inheritance.

Port the existing independent marine/thruster kernels and numerical reference tests
with source attribution. These are useful measured behavior, not a requirement to
retain the old ROS node, Robot class, schemas, or binary interfaces. Source license
metadata is unresolved and explicitly blocks redistribution, not local development.

Use midpoint-thrust/RK4 operator splitting and a spherical pool contact proxy for
this first slice. General hull contacts and prop ownership require later work and
should not be hidden behind a misleading generic collision API now. The sphere
limitation and differences from the old node are documented in PLANT.md and tested
against analytical translation, torque, nonpenetration, and dissipative impulses.

The kernel currently uses floating-point event time internally; the public clock
uses integer nanosecond ticks. Repeated runs and advance partitioning are checked.
Changing the actuator clock representation is a future measured decision, not a
claim of arbitrary-duration exact event arithmetic today.

A resettable fault state avoids copying actuator histories every numerical tick
solely for rollback. Advancement commits each successful tick; numerical failure
requires reset and never returns an apparently successful partial advance. Public
input errors are checked before state mutation. Add fault-path regression coverage
alongside changes that introduce new numerical or external failure modes.

Use system-installed dependencies, target-scoped build options, relocatable CMake
exports, and a real downstream consumer check. Keep the new project out of colcon
recursive discovery with COLCON_IGNORE. No remote is configured and no code is
published by this milestone.
