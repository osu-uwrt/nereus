# Contributing

## Before committing

```sh
cmake --build --preset ros-viewer -j4
ctest --test-dir build/ros-viewer -LE live -R <area>      # the tests for what you touched
PYTHONPATH=python/src python3 -m robotics_platform.packs validate content/packs/scenarios/talos_uwrt
```

Run the full suites (`ctest ... -LE live -j2`, `pytest` in `tests/python`) for changes to the simulation,
session, rules or pack formats. Format C++ with the repo's `.clang-format`; the build treats warnings as errors.

## Where things go

- **Robot, pool, course, ROS wiring**: pack YAML in `content/packs/` (see the guides in `docs/guides/`). If a
  change needs code for one robot or one competition, it belongs in a pack or in `extensions/`, not in the
  libraries.
- **Scoring**: a rules class in `extensions/rules/`, registered by name.
- **Viewer layout and topics**: `content/viewer/*.yaml`.
- **Libraries** (`libraries/`) stay free of ROS; ROS lives in `integrations/ros2/`.

Tests check behaviour, not structure. Equivalence fixtures compare the C++ runtime with the Python reference;
re-record them with the capture scripts in `libraries/session/tests/fixtures/` and
`extensions/rules/robosub_2026/tests/` only when the reference itself changes.

## Commits

One coherent change per commit, with its tests and docs. The message says what changed and why, and how it was
checked. Content copied from other projects gets a line in `docs/PROVENANCE.md`.
