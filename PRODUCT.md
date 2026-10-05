# Product

<!-- impeccable:product-schema 1 -->

## Platform

desktop

Native Linux desktop application (Ubuntu 22.04, ROS 2 Humble): the viewer is C++ with Dear ImGui (docking branch),
GLFW and OpenGL, with its own title bar. Not web, iOS or Android; none of the schema's platform values fit, so this
records the truth instead of the nearest value.

## Users

Primary: **the pool-test operator.** A UWRT (Ohio State Underwater Robotics) member on a laptop at the pool deck,
driving and watching the real Talos AUV: Enable / KILL, motion commands, camera feeds, mapping, autonomy, recording
and telemetry. Often hurried, often in bright light, with the robot in the water.

Secondary: **the stack developer in simulation.** A UWRT software member testing controllers (including the MPC),
autonomy, localization and perception against `nereus-sim` on a desk machine, iterating for hours with the same
viewer.

The prior map editor (Dead Reckoning in 3D) serves both: laying out the riptide_mapping prior map before runs.

## Product Purpose

Nereus is a config-driven underwater robot simulator and operator interface built to test a real ROS 2 robot stack
unchanged. The simulator publishes what the robot's sensors and cameras would, takes the controller's thruster
commands, and scores the run; the same viewer drives the real robot. Success is the team trusting sim results enough
to change the stack in simulation and have it behave the same in the pool, and operating the real robot from the
viewer without reaching for RViz or terminals.

## Positioning

One operator interface for both the simulator and the real robot, driven entirely by data packs: robot, pool,
course and ROS topics live in YAML (`content/packs/`), nothing about the robot is in code, and the robot stack runs
unmodified against either.

## Operating Context

- Pool tests (RoboSub pool, OSU RPAC dive well): laptop poolside, daylight and glare, robot in the water, Zenoh RMW.
- Simulation sessions: `ros2 launch integrations/uwrt/launch/sim.launch.py` (simulator under a supervisor that can
  switch pools, the UWRT stack, the viewer); run records in `/tmp/nereus_sim/<timestamp>/`.
- Real robot: `robot.launch.py`, viewer against TF / estimate topics with no simulator.
- Prior map: riptide_mapping `config/config.yaml` in the robot workspace's source tree, edited in the viewer's map
  mode and saved with comments intact.

## Capabilities and Constraints

- Simulator: 6-DOF marine dynamics, delayed thrusters, IMU / FOG / DVL / depth, stereo cameras with depth and point
  clouds, claw / launcher / dropper / magnet mechanisms, Bullet contact props, task scoring (RoboSub 2026).
- Viewer: 3D pool scene, camera cards, motion control gizmo, autonomy, mapping, detections and point clouds, run
  scorecard, dockable panels and saved layouts, themes and interface scale, a prior map editor (2D / 3D), pool
  switching.
- Hard constraints for UI work:
  - **High-DPI laptops:** runs at 200 % desktop scaling; everything must scale cleanly (`ui()` sizes, desktop-scaled
    title bar, interface scale setting).
  - **Outdoor glare:** used poolside in daylight; contrast must hold, and a readable light theme matters.
  - **Low-end hardware:** must stay responsive on team laptops and weak GPUs alongside the robot stack (the build VM
    has 14 GB and no swap).
  - **Safety-critical controls:** Enable / KILL and motion must never be ambiguous, hidden, or one stray click away.
- Tests never read robot configs; they use frozen scenarios and fixtures.

## Brand Commitments

- Name: Nereus. Logo: a trident rising out of sonar pings (`docs/images/logo.svg`, window icons in
  `content/viewer/icons`).
- Robot: Talos (UWRT). Team: UWRT, Ohio State.
- No AI co-author attribution in commits.

## Evidence on Hand

- `docs/images/viewer.jpg` (viewer screenshot), `docs/viewer.md` (operator guide), `CHANGELOG.md`.
- Packs: Talos robot, RoboSub 2026 pool and course, RPAC dive well, UWRT bridge and equipment
  (`content/packs/`).
- Absent: no outside users, testimonials, benchmarks or adoption claims; do not invent them.

## Product Principles

1. The same interface for sim and real robot: nothing the operator learns in simulation should change at the pool.
2. Safety first: robot state and the means to stop it are always visible and unmistakable.
3. Data, not code: robots, pools, courses and topics are packs; UI must not hard-code one robot or pool.
4. Fast under pressure: the common pool-deck actions are one obvious step, readable at a glance in daylight.
5. Light on the machine: the viewer must not starve the robot stack or the simulator of CPU, GPU or memory.

## Accessibility & Inclusion

High contrast that survives outdoor glare; clean scaling at 200 % and above; safety controls distinguishable by
more than colour alone.

## Audience Scope

UWRT only for now. The repository is Apache-2.0 with contributing docs, but adoption by other teams is not a current
goal; first-run polish for strangers is secondary.
