# Wiring your ROS stack

A bridge pack (`content/packs/bridges/<id>/bridge.yaml`) connects the simulator to an existing ROS 2 stack
without code: which topics to publish (sensors, cameras, TF, clock, run state), which to subscribe to (thruster
commands, kill, mechanisms), which services to offer, and how message fields map. Any ROS message type in your
sourced workspace works; fields are set by name through ROS introspection.

`bridges/uwrt_talos/bridge.yaml` wires Talos to the unchanged UWRT stack and has an entry for every case below.

## Top level

```yaml
kind: bridge
id: my_stack
requires_robot: my_auv           # refuses to run with a different robot pack
namespace: /my_auv               # prefix for every relative topic and service
node_name: physics_simulator
frame_names:                     # robot-pack frame -> ROS frame id
  world: map
  imu_mount: my_auv/imu_link
  ffc_left_optical: my_auv/ffc_left_camera_optical_frame
clock:                           # /clock for use_sim_time stacks
  topic: /clock
  epoch: system_time_at_start
  rate_hz: 500
  reset_policy: preserve_ros_epoch_and_time
  real_time_factor: 1.0
  qos: {history: keep_last, depth: 10, reliability: reliable, durability: volatile}
thrusters:                       # your controller's force array -> the robot pack's thruster order
  order: [VUS, VUP, HUS, HUP, HLS, HLP, VLS, VLP]
  input_scales: [1, 1, 1, 1, 1, 1, 1, 1]
  reject: [wrong_length, nonfinite]
kill: {command_stream: software_kill, state_stream: firmware_kill}   # stream ids below
streams: [...]
services: [...]
```

## Streams

Each stream is one topic.

```yaml
- id: dvl
  direction: publish                       # or subscribe
  topic: dvl_twist                         # relative to namespace
  message_type: geometry_msgs/msg/TwistWithCovarianceStamped
  native: sensor:dvl                       # the simulator side, see below
  frame_id: my_auv/dvl_link                # header.frame_id ('' for none)
  rate_hz: 8                               # publish rate; 0 = on every event
  qos: {history: keep_last, depth: 10, reliability: reliable, durability: volatile}
  fields:                                  # ROS field <- source
    header.stamp: {from: sample.time}
    twist.twist.linear: {from: reading.reference_relative_velocity}
    twist.covariance[0]: {from: reading.covariance[0][0]}
```

- **`fields`** map a ROS field path (`a.b[2]`) to `{from: <source path>}`, `{constant: <value>}` or
  `{from: ..., enum_map: {...}}`. On `subscribe` streams the keys are the native argument names and `from` is the
  ROS field: `fields: {forces_n: {from: data}}`.
- **`accept_if`** keeps only matching messages on a subscribe stream:
  `accept_if: [{field: kill_switch_id, equals: 1}]`.
- **Camera streams** add `image:` (`{encoding: 32FC1}` for depth, or `{compression: jpeg, quality: 93,
  source_encoding: rgb8, compressed_encoding: bgr8}`) or `point_cloud: {stride: 8}`, and are only rendered while
  subscribed.
- **Marker streams** use `format: marker_array` with `options` instead of `fields` (props, payloads, lights).

### Native endpoints

| `native` | Direction | What it is |
| --- | --- | --- |
| `sensor:<id>` | publish | a robot-pack sensor reading (`reading.*`, `sample.time`) |
| `sensor:<cam>.rgb_left` / `.depth_left` / `.camera_info` / `.point_cloud` | publish | camera outputs |
| `state:robot` | publish | truth pose and twist (`reference_pose`, `sim.time`) |
| `state:thrusters`, `state:mechanisms`, `state:claws`, `state:payloads`, `state:props`, `state:indicators` | publish | simulator state for viewers and telemetry |
| `state:run`, `state:task_score` | publish | scorecard and task counters (JSON) |
| `event:robot.kill_changed`, `event:tasks.feed`, `event:mechanisms.command_result`, `event:scenario.description` | publish | fired on change |
| `command:thrusters.set_forces` | subscribe | thruster forces |
| `command:robot.set_killed` | subscribe | kill switch |
| `command:mechanisms.set_armed`, `.reload_all`, `.<id>.fire`, `.<id>.command`, `.<id>.timed_move` | subscribe | actuators |
| `command:runs.command`, `command:tasks.reset` | subscribe | run control from the viewer |
| `estimate:latest` | subscribe | the stack's state estimate (for "sync sim to estimate") |

Reading fields per sensor type are in `libraries/sensors/include/nereus/sensors/readings.hpp` (for example
`reading.inertial.angular_velocity`, `reading.attitude.orientation_wxyz`, `reading.target_world_z`).

## TF

```yaml
tf:
  publish:                                 # moving frames
  - {parent: map, child: simulator/my_auv/base_link, native: state:robot.reference_pose, rate_hz: 100}
  static:                                  # fixed frames from the robot pack
  - {parent: my_auv/ffc_camera_link, child: my_auv/ffc_left_camera_optical_frame, from_frame: ffc_mount, to_frame: ffc_left_optical}
  lookup:                                  # frames your stack owns; the bridge reads them
  - {parent: world, child: map}   # published by the mapping node
  never_publish: [my_auv/base_link, map]   # guard against publishing over your stack
```

Publish the simulator's truth under its own prefix (`simulator/...`) so it never collides with your state
estimate's `base_link`. The viewer draws the robot from the truth frame and your estimate as a ghost.

## Services

```yaml
services:
- id: reset_sim_to_start
  service: reset_sim_to_start
  service_type: std_srvs/srv/Trigger
  action: command:robot.reset_to_start
  request: {}
  response: {success: {from: accepted}, message: {from: message}}
```

Actions: `command:robot.place` (set the sim pose), `command:robot.reset_to_start`, `command:scenario.reset`,
`command:tasks.reset`, and the mechanism commands. `reset:` and `placement:` at the top level name the services
used for resets and for aligning your estimator after a placement (`estimator_alignment`).

## Checking a bridge

```sh
uv run nereus-packs validate content/packs/scenarios/<scenario>
ros2 launch integrations/uwrt/launch/sim.launch.py scenario:=$PWD/content/packs/scenarios/<scenario> stack:=false
ros2 topic list          # every stream topic should be there
```

The bridge rejects unknown message types, fields and native endpoints at startup, naming the stream. Run records
in `/tmp/nereus_sim/<run>/summary.json` count published, filtered and rejected messages per stream.
