"""Generic synthetic pack set shared by the pack tests (no Talos or year data)."""

from pathlib import Path

ROBOT = """\
# Synthetic four-thruster robot; comments and key order must survive load/save.
kind: robot
id: synth
metadata: {author: tests, note: free non-executable annotation}
assets: []
reference_frame: base_link
body:
  type: marine_6dof
  parameters:
    mass_kg: 20.0
    inertia_matrix: [[1.0, 0.0, 0.0], [0.0, 1.5, 0.0], [0.0, 0.0, 1.2]]
    added_mass_matrix: [[5, 0, 0, 0, 0, 0], [0, 5, 0, 0, 0, 0], [0, 0, 5, 0, 0, 0], [0, 0, 0, 0.1, 0, 0], [0, 0, 0, 0, 0.1, 0], [0, 0, 0, 0, 0, 0.1]]
    linear_damping_matrix: [[10, 0, 0, 0, 0, 0], [0, 10, 0, 0, 0, 0], [0, 0, 10, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]]
    quadratic_damping: [20, 20, 20, 1, 1, 1]
    displaced_volume_m3: 0.02
    buoyancy_center_m: [0, 0, 0.01]
    buoyancy_radii_m: [0.2, 0.3, 0.2]
    command_timeout_s: 0.5
frames:
  root: com
  transforms:
  - {parent: com, child: base_link, position_m: [0, 0, -0.05], orientation_wxyz: [1, 0, 0, 0]}
  - {parent: base_link, child: imu_mount, position_m: [0.1, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
collision_boxes:
- {id: hull, size_m: [0.6, 0.4, 0.3], center_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
thrusters:
- {id: t0, type: lagged_force, position_m: [-0.3, 0.2, 0], direction: [1, 0, 0], parameters: {delay_s: 0.05, rise_time_s: 0.05, fall_time_s: 0.05, slew_rate_n_s: 200, forward_limit_n: 20, reverse_limit_n: 20, deadband_n: 0, forward_scale: 1, reverse_scale: 1, efficiency: 1, propeller_radius_m: 0.04}}
- {id: t1, type: lagged_force, position_m: [-0.3, -0.2, 0], direction: [1, 0, 0], parameters: {delay_s: 0.05, rise_time_s: 0.05, fall_time_s: 0.05, slew_rate_n_s: 200, forward_limit_n: 20, reverse_limit_n: 20, deadband_n: 0, forward_scale: 1, reverse_scale: 1, efficiency: 1, propeller_radius_m: 0.04}}
- {id: t2, type: lagged_force, position_m: [0, 0.2, 0], direction: [0, 0, 1], parameters: {delay_s: 0.05, rise_time_s: 0.05, fall_time_s: 0.05, slew_rate_n_s: 200, forward_limit_n: 20, reverse_limit_n: 20, deadband_n: 0, forward_scale: 1, reverse_scale: 1, efficiency: 1, propeller_radius_m: 0.04}}
- {id: t3, type: lagged_force, position_m: [0, -0.2, 0], direction: [0, 0, 1], parameters: {delay_s: 0.05, rise_time_s: 0.05, fall_time_s: 0.05, slew_rate_n_s: 200, forward_limit_n: 20, reverse_limit_n: 20, deadband_n: 0, forward_scale: 1, reverse_scale: 1, efficiency: 1, propeller_radius_m: 0.04}}
safety:
  initially_killed: false
  kill_stops_thrusters: true
  commands_while_killed: rejected
  kill_disarms_mechanisms: true
  arming: {initially_armed: false, arm_rejected_while_killed: true, applies_to: [marker]}
sensors:
- id: imu
  type: ahrs
  frame: imu_mount
  mount_frame: imu_mount
  period_ns: 10000000
  parameters:
    inertial: {acceleration_noise: {white_stddev: [0.01, 0.01, 0.01]}}
    attitude: {angle_stddev_rad: 0.001}
- id: altitude
  type: reference_altitude
  frame: world
  mount_frame: base_link
  period_ns: 50000000
  parameters: {target_position_body_m: [0, 0, -0.05], noise: {white_stddev: 0.01}, reported_variance: 0.0001}
mechanisms:
- id: marker
  type: dropper
  frame: base_link
  parameters:
    slots:
    - {id: s0, position_m: [0, 0, -0.1], orientation_wxyz: [0.7071067811865476, 0, 0.7071067811865476, 0]}
    projectile: {model: fixed_axis_body, length_m: 0.08, radius_m: 0.01, mass_kg: 0.02, displaced_volume_m3: 1.0e-05, added_mass_kg: 0.002, drag_axial: 0.01, drag_lateral: 0.3, neutral_buoyancy: false, max_age_s: 20}
    launch: {spring_energy_j: 0.01}
    cooldown_s: 0.5
    cooldown_group: markers
    capacity: 1
scoring_envelope:
  collision_boxes: [hull]
  points: []
"""

POOL = """\
kind: pool
id: tank
type: rectangular_pool
parameters: {length_m: 10, width_m: 5, depth_m: 3, deck_height_m: 0.3, water_level_m: 0, water_density_kg_m3: 1000, surface_pressure_pa: 101325, current_m_s: [0, 0, 0], current_oscillation_amplitude_m_s: [0, 0, 0], current_oscillation_frequency_hz: 0}
collision_boxes:
- {id: floor, size_m: [10, 5, 1], center_m: [5, 2.5, -3.5], orientation_wxyz: [1, 0, 0, 0]}
"""

TASKS = """\
kind: tasks
id: practice
assets:
- {id: hoop_mesh, path: assets/hoop.dae}
tasks: [hoop.yaml]
requires:
- {task: hoop, mechanism_type: dropper, min_count: 1}
run_options:
- {key: timed, type: bool, default: false}
scoring_hooks: []
"""

HOOP = """\
kind: task
id: hoop
frames:
- {id: marker_point, position_m: [0, 0, 0.5], orientation_wxyz: [1, 0, 0, 0]}
props:
- id: ring
  type: static_body
  parameters:
    collision_boxes:
    - {id: top, size_m: [0.05, 1.0, 0.05], center_m: [0, 0, 0.5], orientation_wxyz: [1, 0, 0, 0]}
    visuals:
    - {asset: hoop_mesh, frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
regions:
- id: opening
  type: rectangular_portal
  parameters: {plane: {axis: x, offset_m: 0}, bounds_local: {abs_y_lt_m: 0.5, z_lt_m: 0.45}, world_floor_clearance: true, crossing_reference: robot_reference_origin, fit_checks: [envelope_at_completion], traversal: full_envelope, approach_radius_m: 2, max_pose_step_m: 1}
events:
- {id: through, type: pass_through, parameters: {region: opening, from_side: positive, to_side: negative, emits: [crossing_point_local]}}
scoring:
- {id: through_points, type: event_points, parameters: {event: through, points: 100, max_awards: 1}}
"""

BRIDGE = """\
# Sensor-only bridge: no thrusters, TF, kill, reset or placement blocks.
kind: bridge
id: synth_sensors
namespace: /synth
clock: {topic: /clock, epoch: zero, rate_hz: 100, reset_policy: restart_from_epoch, real_time_factor: 1.0, qos: {history: keep_last, depth: 10, reliability: reliable, durability: volatile}}
streams:
- id: altitude
  direction: publish
  topic: altitude
  message_type: std_msgs/msg/Float64
  native: 'sensor:altitude'
  frame_id: ''
  rate_hz: 20
  qos: {history: keep_last, depth: 10, reliability: reliable, durability: volatile}
  fields: {data: {from: reading.target_world_z}}
"""

SCENARIO = """\
kind: scenario
id: practice_run
robot: ../robot
pool: ../pool
tasks: ../tasks
bridge: ../bridge
seed: 3
timestep_ns: 2000000
sensor_noise: false
world_frame: map
pool_placement: {position_m: [0, 0, 0], yaw_deg: 0}
task_placements:
- {task: hoop, position_m: [5, 2.5, -1], yaw_deg: 90}
contacts: {model: box_scene, restitution: 0.1, friction: 0.4}
initial: {frame: com, position_m: [2, 2, -1], orientation_wxyz: [1, 0, 0, 0], linear_velocity_m_s: [0, 0, 0], angular_velocity_rad_s: [0, 0, 0]}
run: {auto_start: true, options: {}}
"""


def write_generic_packs(root: Path, bridge: bool = True) -> Path:
    """Write robot/pool/tasks/(bridge)/scenario folders under root; return the scenario folder."""
    files = {
        "robot/robot.yaml": ROBOT,
        "pool/pool.yaml": POOL,
        "tasks/tasks.yaml": TASKS,
        "tasks/hoop.yaml": HOOP,
        "scenario/scenario.yaml": SCENARIO if bridge else SCENARIO.replace("bridge: ../bridge\n", ""),
    }
    if bridge:
        files["bridge/bridge.yaml"] = BRIDGE
    for relative, text in files.items():
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding="utf-8")
    (root / "tasks" / "assets").mkdir(exist_ok=True)
    (root / "tasks" / "assets" / "hoop.dae").write_bytes(b"mesh")
    return root / "scenario"
