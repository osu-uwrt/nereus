# Changelog

## 0.1.0 — unreleased

- Versioned reusable robot/world/sensor profiles and resolved runtime construction,
  checked typed stream lookup, full physical matrix configuration, and sensor CSV.

- Pressure/depth sensor with mounted hydrostatic sampling, separate calibration,
  seeded noise, operating limits, and propagated depth uncertainty.

- New standalone C++ plant with explicit ticks, value snapshots, reset, thruster
  delays/limits, marine dynamics, and simple sphere-to-pool contacts.
- Standalone typed sensor scheduling, bounded delivery queues, deterministic reset
  and per-device noise; mounted IMU, FOG, and ideal bottom-track DVL models.
- Read-only plant kinematics, explicit contact-acceleration validity, data-only
  measurement headers, and a relocated installed sensor example.
- Native YAML runner and deterministic CSV output for a synthetic example AUV.
- Optional config/CLI build, relocatable CMake exports, downstream consumer example.
- Numerical, lifecycle, command, contact, schema, and CLI regression tests.
- Local check tooling, CI configuration, architecture/decision/contribution docs.
- License/provenance audit identifies unresolved upstream kernel licensing before
  public distribution. No remote publication has occurred.
