# Robotics Platform

A standalone underwater simulation library and, eventually, an independent
robotics viewer. This is a new local project; it does not depend on the old
Riptide simulator, ROS, UWRT packages, a display, or GPU drivers.

The first working slice provides a C++ plant, a strict YAML scenario loader,
a command-line runner, and an installed-library example. The plant composes
marine dynamics, delayed thrusters, and simple pool contacts behind explicit
`command`, `advance`, `observe`, and `reset` operations.

A second library adds deterministic sensor scheduling, seeded noise, IMU, FOG,
pressure/depth, and ideal bottom-track DVL models. Native robot/world/sensor
[profiles](docs/PROFILES.md) compose these into standalone runs. See the [sensor API and example](docs/SENSOR_RUNTIME.md).
An optional [Python API](docs/PYTHON.md) exposes profile loading, programmatic
configuration, stepping/reset, and typed sensor streams. The viewer, ROS integrations,
cameras/stereo, task interactions, and training integrations are not implemented. See [status](docs/STATUS.md) and the
[architecture plan](docs/ARCHITECTURE_PLAN.md). [Sensor scope](docs/SENSORS.md)
covers cameras, stereo cameras, IMUs, DVLs, FOGs, and pressure/depth sensors; sonar is deferred.
The current example parameters
are synthetic, not a calibrated prediction for any team's robot.

## Build and run

Initial tested environment: Ubuntu 22.04, GCC 11, C++17, CMake 3.22+, Eigen 3.3+,
yaml-cpp, and GoogleTest for tests. On Ubuntu:

```sh
sudo apt-get install cmake g++ libeigen3-dev libyaml-cpp-dev libgtest-dev
cmake --preset release
cmake --build --preset release
ctest --preset release
./build/release/robotics-sim content/examples/profile_pool.yaml \
  --sensors build/sensors.csv > build/trajectory.csv
```

The example advances three simulated seconds and emits one CSV row per tick,
including the initial state, plus timestamped sensor observations in a separate CSV.
Commands are scheduled in the YAML. There is no
wall-time pacing, ROS clock, background thread, or GUI process.

For the numerical and sensor libraries without YAML, tests, or Python:

```sh
cmake --preset plant-only
cmake --build --preset plant-only
```

`COLCON_IGNORE` keeps this project out of automatic recursive workspace discovery.
Build it directly; all generated files stay in this project's `build/` directory.

## Python

```sh
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/python examples/python/run_profile.py
```

Building the wheel also requires Python development headers and the native Eigen/
yaml-cpp dependencies. See [Python installation, contracts, and checks](docs/PYTHON.md).
The C++ build remains independent; Python is enabled only for the wheel or with
`RP_BUILD_PYTHON=ON`.

## Install and consume

```sh
cmake --install build/release --prefix "$PWD/install"
cmake -S examples/cpp_consumer -B build/consumer -DCMAKE_PREFIX_PATH="$PWD/install"
cmake --build build/consumer
./build/consumer/consumer
./install/bin/robotics-sim install/share/robotics_platform/examples/empty_pool.yaml
```

CMake consumers link `RoboticsPlatform::simulation` or `RoboticsPlatform::sensors`.
The optional config library exports `RoboticsPlatform::config`. Public headers contain no ROS or YAML types.
See [API and numerical contracts](docs/PLANT.md), [contributing](CONTRIBUTING.md),
and [design decisions](docs/decisions/0001-first-standalone-slice.md).

## License and provenance

This is local development, not a public release. The reused numerical source has
unresolved license metadata; see [license status](LICENSE.md) and
[provenance](docs/PROVENANCE.md). No new redistribution license is being asserted
for the imported code. Resolve this before publishing the project.
