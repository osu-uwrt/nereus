# ADR 0003: Resolve native profiles before runtime construction

Status: accepted for the single-vehicle native profile increment.

## Context

Teams need to reuse robot descriptions across worlds and configure sensors without
editing a ROS launch file or recompiling the simulator. Source files and YAML nodes
must not become hidden dependencies of numerical stepping or sensor sampling.

## Decision

Use explicit versioned scenario, robot, world, and sensor documents with references
relative to the declaring file. Validate fields and model/physical invariants before
returning a resolved C++ scenario. Preserve the existing inline schema for its small
plant example; new composed scenarios select schema 2. Do not add deep configuration
merges, inheritance, environment-based package discovery, or recursive include rules.

Resolve sensors into typed attachment factories. The runtime accepts arbitrary model
value types and exposes checked typed stream lookup by stable ID. Type erasure stores
stream handles, not measurement dictionaries. Configuration decoders and diagnostic
CSV exporters register supported built-ins at their respective edges. YAML never
enters the sensor or simulation libraries.

Keep pressure environment values in the world and pressure-to-depth calibration in
the device. Keep all physical matrix entries available through full matrix inputs;
diagonal inputs are a convenience, not a robot-model restriction.

## Consequences

A resolved scenario can create isolated replayable runtimes after its files are
removed. Installed content can move with its directory structure. Invalid native
configuration fails before output; failures during stepping retain the documented
runtime fault semantics.

The current resolved factories are in-process executable values, not a portable
serialized manifest. Source paths are recorded but content hashes, generic installed
resource roots, a public decoder/plugin registration API, and multi-body/task/asset
packs remain future work. CSV exports make all delivered built-in measurements
reviewable but do not implement a viewer or general recording transport.

See [native profile schema and usage](../PROFILES.md).
