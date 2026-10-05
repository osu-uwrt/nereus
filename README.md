<p align="center"><img src="docs/images/logo.svg" alt="Nereus logo: a trident rising out of sonar pings" width="128"></p>

# Nereus

A config-driven underwater robot simulator and operator interface, built to test a real ROS 2 robot stack
unchanged. Describe your robot, pool, course and ROS topics in YAML; the simulator publishes what your robot's
sensors and cameras would, takes your controller's thruster commands, and scores the run. The same viewer drives
the real robot.

![Pool viewer on the RoboSub 2026 course](docs/images/viewer.jpg)

- **Simulator** (`nereus-sim`): 6-DOF marine dynamics, delayed thrusters, IMU/FOG/DVL/depth, stereo
  cameras with depth and point clouds, claw/launcher/dropper/magnet mechanisms, contact props (Bullet), task
  scoring. C++, runs at real time with cameras.
- **Pool viewer** (`nereus-viewer`): 3D scene, camera cards, motion control, autonomy, mapping,
  detections and point clouds, run scorecard. Works against the simulator or a real robot.
- **Packs**: all robot/pool/course/bridge data lives in `content/packs/`; nothing about your robot is in code.

## Setup

Tested on Ubuntu 22.04 with ROS 2 Humble.

```sh
# System libraries
sudo apt install cmake g++ libeigen3-dev libyaml-cpp-dev nlohmann-json3-dev libgtest-dev \
  libbullet-dev libassimp-dev libglfw3-dev libglew-dev libegl-dev libpng-dev libjpeg-dev libopencv-dev
# uv, once per machine (manages the Python environment)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Build (with ROS 2 and your robot workspace sourced; the UWRT integration needs riptide_msgs2)
source /opt/ros/humble/setup.bash && source ~/osu-uwrt/release/install/setup.bash
./build.sh
```

`./build.sh` syncs the Python environment (`.venv`: pack tools, dataset tools, tests and linters, installed
editable from `python/src`) and builds a CMake preset: `ros-viewer` when ROS is sourced, otherwise `datasets`
(the headless renderer and dataset generator); `./build.sh <preset>` picks one. Use `NEREUS_JOBS=2` on machines
with little memory. Python-only setup is just `uv sync`. Run the tools with `uv run <tool>` or after
`source .venv/bin/activate`. `COLCON_IGNORE` keeps colcon out of this folder; everything builds into `build/`.

## Run

```sh
# Simulator + UWRT stack + viewer
ros2 launch integrations/uwrt/launch/sim.launch.py
#   pool:=rpac  (robosub default; scenario:=<pack folder> for any other; switch later with View > Pool)
#   stack:=false  viewer:=false  cameras:=false  rmw:=rmw_zenoh_cpp
#   active_control_model:=mpc mpc_model:=sim     (MPC controller on the simulator plant)

# Viewer against the real robot (no simulator)
ros2 launch integrations/uwrt/launch/robot.launch.py
#   robot_only:=true  (hide the simulated pool)   rmw:=...   config:=<viewer host yaml>
```

The launch files use your shell's RMW; UWRT runs Zenoh (`ros2 run rmw_zenoh_cpp rmw_zenohd`). Run records
(resolved scenario, performance, task events) go to `/tmp/nereus_sim/<timestamp>/`. A run waits for **Start
run** in the viewer before scoring.

## How it fits together

![Nereus architecture: the scenario pack selects robot, pool, tasks, bridge and equipment packs, which
nereus.packs resolve validates into one resolved.json. In simulation, nereus-sim loads it and talks to your robot
stack and nereus-viewer over ROS 2. On the real robot there is no simulator: nereus-viewer loads resolved.json
itself and talks to the robot. Synthetic datasets are planned, rendered and exported offline](docs/images/architecture.svg)

Everything starts from the same resolved scenario. In simulation, `nereus-sim` runs it and the viewer shows
simulator truth. On the real robot, the viewer loads it directly and draws the robot at its EKF estimate. The
simulator, your robot stack and the viewer share no code, only ROS 2 topics. `←` marks the config each part
reads.

| Folder | Contents |
| --- | --- |
| `content/packs/` | Robot, pool, task, bridge, equipment and scenario packs (the config you edit) |
| `content/viewer/` | Viewer layout, panels and topics |
| `libraries/` | C++ simulation, sensors, session (tasks, props), rendering, cameras |
| `integrations/ros2/` | Simulator bridge and pool viewer |
| `integrations/uwrt/` | UWRT launch files and acceptance scripts |
| `extensions/rules/` | Competition scoring rules (C++) |
| `python/` | Pack tools: validate and resolve packs |

## Guides

- [Using the viewer](docs/viewer.md)
- [Adding a robot](docs/guides/robot.md)
- [Pool and course layout](docs/guides/course.md)
- [Tasks and scoring](docs/guides/tasks.md)
- [Wiring your ROS stack](docs/guides/bridge.md)
- [Synthetic datasets](docs/guides/datasets.md)

Check a pack after editing:

```sh
uv run nereus-packs validate content/packs/scenarios/talos_uwrt
```

## Known limitations

- Acoustics aren't simulated: there is no pinger, so the random-pinger tasks can't score.
- The coin flips and the manual score adjustment are set by the operator in the Run panel; there
  is no session countdown or automatic time bonus.
- The plant is a model of Talos. Controller gains tuned in the simulator don't carry over to the
  real robot; tune those in the pool.

## Tests

```sh
ctest --test-dir build/ros-viewer -LE live -j2          # C++ (run a subset with -R <name>)
uv run pytest tests/python -q
```

## License

Apache License 2.0; see [LICENSE](LICENSE) and [NOTICE](NOTICE). Bundled Dear ImGui and GLM keep their MIT
licenses.
