# Contributing

## Before committing

```sh
./build.sh
ctest --test-dir build/ros-viewer -LE live -R <area>      # the tests for what you touched
uv run nereus-packs validate content/packs/scenarios/talos_uwrt
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

Tests check behavior, not structure. The reference fixtures in `libraries/session/tests/fixtures/`,
`extensions/rules/robosub_2026/tests/` and `tests/fixtures/` are recordings of known-good runs. When you change
behavior on purpose, update the affected expected values and say why in the commit.

The C++ tests run on frozen resolved scenarios (`libraries/session/tests/fixtures/*_resolved.json.in`), not on
the live packs, so retuning the robot (mass, inertia, buoyancy, frames) or moving the course never breaks them.
Python tests that check Talos-specific values (e.g. `test_camera_pack_fields.py`) read the same frozen documents.
The live packs are only checked for being valid and saving byte-exact (`nereus.packs validate` and the Python
pack tests), never for their values. Re-freeze them with
`tools/freeze_session_scenarios.py` only when you re-record the references that depend on them.

## Commits

One coherent change per commit, with its tests and docs. The message says what changed and why, and how it was
checked.
