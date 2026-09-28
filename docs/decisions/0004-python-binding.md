# ADR 0004: Optional Python binding over the native runtime

Status: accepted for the initial direct Python API.

## Context

Python callers need profile construction, explicit commands/stepping/reset, and typed
sensor observations without ROS. Reimplementing the scheduler in Python would create
two owners of acquisition time and noise state. Future training and behavior code
needs instance isolation without requiring a training framework now.

## Decision

Bind the existing runtime and concrete model/configuration types with pybind11.
Keep integration, sensor scheduling, noise, and queue ownership in C++; Python
orchestrates commands and observation consumption. Return copied snapshots/readings,
integer nanosecond timestamps, and checked typed stream handles. Binding Eigen values
uses explicit value returns rather than memory views into live native state.

Use a separate optional CMake target and a scikit-build-core wheel with a Python
source layout, explicit exports, public stubs, and packaged example profiles. The
wheel statically includes project libraries, while using the platform yaml-cpp
shared library. Pure C++ builds do not discover Python unless this target is enabled.
This is a simulation-facing wheel: the future independent viewer must use its own
neutral contracts and must not require this package for live data sources.

Retain the GIL for the initial synchronous API. Do not introduce Python callbacks,
shared-instance concurrency, or a second scheduler in this increment. There are no
external handles/background workers needing an artificial close protocol. Runtime
ownership is explicit; stream handles do not keep runtimes alive, and retained
observations remain valid independently.

## Consequences

Installed Python callers can configure and replay independent runtimes, with the
same numerical/error contracts as C++. The architecture's proposed Python behavior
coordinator remains possible without moving the validated physics/sensor tick loop.
Further task/contact coordination requires explicit interfaces and profiling.

The wheel is currently a local ABI/platform artifact, not a portable binary release.
Python model/provider plugins, GIL release with instance synchronization, checkpoints,
batched environments, rendering/viewer bindings, and training are not implemented.
Cross-build floating-point comparisons use tight tolerances; within-build seeded
replay and all timestamp/identity checks remain exact.

See [Python API and verification](../PYTHON.md). Binding/build behavior follows
[pybind11's Eigen conversion documentation](https://pybind11.readthedocs.io/en/stable/advanced/cast/eigen.html)
and [scikit-build-core's packaging configuration](https://scikit-build-core.readthedocs.io/en/stable/schema.html).
