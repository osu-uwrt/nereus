# Changelog

## 0.1.0 — unreleased

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
