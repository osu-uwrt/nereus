# Native robot, world, and sensor profiles

Scenario schema 2 composes local robot/world files and a deterministic run. Robot
and world files use profile schema 1, distinguished by `kind`. Robot sensor entries
can contain model parameters or reference a reusable sensor profile. All formats
reject unknown and duplicate keys, malformed numbers, unsupported versions/kinds,
and invalid physical/model settings before the runner emits trajectory output.

This increment describes one rigid vehicle in a pool. It does not yet load meshes,
multiple bodies, mechanisms, task/competition packs, or viewer workspaces. There
are no inheritance chains, overlays, implicit environment expansion, ROS package
lookups, or network resources. Existing inline scenario schema 1 remains a small
plant-only run format. This is unrelated to compatibility with the old simulator.

## Run the composed example

```sh
./build/release/robotics-sim content/examples/profile_pool.yaml \
  --sensors build/observations.csv > build/trajectory.csv
```

The synthetic robot includes IMU, FOG, bottom-track DVL, and pressure/depth sensors.
Its robot and world files are reusable in another scenario. The example's noise
values are illustrative, not manufacturer calibration. The entire `content/` tree
is installed under `share/robotics_platform`; the same relative references work
when the installation is moved.

## File responsibilities and resolution

| Document | Required fields | Responsibility |
| --- | --- | --- |
| Scenario | `schema_version: 2`, `robot`, `world`, `seed`, `timestep_ns`, `ticks`, `initial`, `commands` | Choose profiles, initial conditions, seed, and run schedule. |
| Robot | `schema_version: 1`, `kind: robot`, `vehicle`, `sensors` | Physical body, thrusters, device identities, mounts, and schedules. Empty sensor/thruster lists are valid. |
| World | `schema_version: 1`, `kind: world`, `pool`, `surface_pressure_pa` | Pool geometry, fluid density/current/level, and atmospheric pressure. |
| Sensor | `schema_version: 1`, `kind: sensor`, `model`, `parameters` | Reusable measurement/noise/calibration settings; no robot-specific ID, frame, mount, or schedule. |

Each reference resolves relative to its declaring file, independent of working
directory. Absolute filesystem paths are accepted but reduce portability. URI
schemes such as `package://` are rejected. References are explicit and shallow:
scenario to robot/world, robot to sensor. Sensor profiles cannot recursively include
other profiles. Symlinks follow filesystem semantics; this is not a filesystem sandbox.

`loadScenario` records resolved source paths in first-use order. It returns native
parameters and model factories, with no retained YAML nodes, file handles, or source
references needed for execution. Tests remove the files after loading and then
construct/run multiple instances. Source content hashes, installed resource search
roots, and a portable serialized resolved manifest remain future work.

## Physical and run fields

`vehicle` uses the field names shown in `content/robots/synthetic_auv.yaml`:
`mass_kg`, inertia/added-mass/damping, `quadratic_damping`, `displaced_volume_m3`,
`buoyancy_center_m`, `buoyancy_radii_m`, `collision_radius_m`, `command_timeout_s`,
and `thrusters`. Every thruster declares its ID, position/direction, delay, rise/fall,
slew rate, and forward/reverse limits. Offsets are from COM in body coordinates.

For each matrix choose exactly one representation:

| Diagonal vector | Full matrix (sequence of rows) | Size |
| --- | --- | --- |
| `inertia_diagonal` | `inertia_matrix` | 3 × 3 |
| `added_mass_diagonal` | `added_mass_matrix` | 6 × 6 |
| `linear_damping_diagonal` | `linear_damping_matrix` | 6 × 6 |

Optional `damping_center_m` defaults to zero. Physical invariants, including matrix
symmetry/definiteness requirements, are validated by the plant constructors rather
than duplicated in the loader. `pool` declares `length_m`, `width_m`, `depth_m`,
`water_level_m`, `water_density_kg_m3`, and `current_m_s`. Surface pressure must be
positive; it affects the pressure environment, not sensor calibration.

`initial` contains position, body-to-world `orientation_wxyz`, and body-frame linear
and angular velocity. Commands use strictly increasing integer `tick` and `forces_n`
in robot thruster order; their ticks must precede `ticks`. `seed` is an unsigned
64-bit decimal integer. Periods and elapsed time use integer nanoseconds; the loader
rejects duration/elapsed-time overflow. The existing [plant contracts](PLANT.md)
and [sensor runtime contracts](SENSOR_RUNTIME.md) define numerical behavior.

## Device declaration

Every robot sensor entry requires `id`, `frame`, `period_ns`, and `mount`. Mounts
contain `position_m` and sensor-to-body `orientation_wxyz`. Device IDs are unique;
frames are nonempty labels, not resolved transform graph nodes.

Optional schedule fields are `latency_ns` (default 0), `capacity` (64 entries in each
queue), and `overflow` (`fail` by default, or explicit `drop_oldest`). Periods must
be at least one physics timestep. See the runtime document for quantization and
queue behavior.

Choose either `profile: ../sensors/imu.yaml` or inline `model` plus `parameters`.
Combining a reference with inline parameters is rejected; there are no hidden merge
rules or overrides. All models require a parameters mapping, even when it is `{}`.

| Model | Parameters (optional unless stated otherwise) |
| --- | --- |
| `imu` | `acceleration_noise`, `gyro_noise`, `reporting` |
| `attitude` | `angle_stddev_rad`, `heading_drift_rad_s`, `heading_axis_world` (unit +Z), `reported_variance` |
| `ahrs` | Required `inertial` mapping (raw IMU settings) and `attitude` mapping (attitude settings); each may be `{}` |
| `fog` | Required `axes`: one to three unit vectors in sensor coordinates; `gyro_noise`, optional sensor-axis diagonal `reported_variance` in (rad/s)² |
| `dvl` | `bottom_axis` (default sensor −Z), `minimum_range_m` (0.1), `maximum_range_m` (50), `velocity_noise` |
| `reference_velocity` | `reference_velocity_world_m_s` (zero), `velocity_noise`, `reported_variance`, optional `inclination_limit` |
| `pressure` | `noise`, `reference_pressure_pa` (101325), `reference_density_kg_m3` (1000), `reference_gravity_m_s2` (9.80665), `minimum_pressure_pa` (0), `maximum_pressure_pa` (10000000) |

Noise mappings allow `bias`, `white_stddev`, and `walk_stddev`, defaulting to zero.
Use three-element vectors for IMU/FOG/DVL and scalars for pressure. Units are the
measurement's units, per-acquisition standard deviation, and standard deviation per
sqrt(second), respectively. Pressure calibration intentionally remains independent
of the world's density and atmospheric pressure. DVL uses the resolved world's
finite pool-floor query; pressure uses its planar hydrostatic environment.

IMU `reporting` optionally contains `gravity_magnitude_m_s2` (positive finite scalar),
`force_variance` (three nonnegative finite variances in (m/s²)²), and
`angular_variance` (three nonnegative finite variances in (rad/s)²). These are
measurement settings: gravity calibration preserves the environment's gravity
direction and never modifies plant dynamics. Reported variances are diagonal in
sensor axes and independent of generated noise; omitted variances use the noise
model's evolving covariance. For example, the original Talos raw inertial reporting
can be expressed as:

```yaml
reporting:
  gravity_magnitude_m_s2: 9.755455
  force_variance: [0.01, 0.01, 0.01]
  angular_variance: [0.01, 0.01, 0.01]
```

Raw IMU does not include orientation. Select `attitude` for a separate simulated
orientation observation, or `ahrs` to compose both at one acquisition with a shared
mount. Example AHRS parameters:

```yaml
inertial:
  reporting:
    gravity_magnitude_m_s2: 9.755455
    force_variance: [0.01, 0.01, 0.01]
    angular_variance: [0.01, 0.01, 0.01]
attitude:
  angle_stddev_rad: 0.008726646259971648
  reported_variance: [0.00005, 0.00001, 0.01]
```

Attitude noise is an isotropic random-axis rotation with normally distributed angle.
Signed heading drift defaults to zero and uses elapsed simulation time. Default
covariance is the small-angle `angle_stddev_rad² / 3` diagonal; an explicit
`reported_variance` overrides it in sensor axes. See [SENSOR_RUNTIME.md](SENSOR_RUNTIME.md)
for composition, validity and reset semantics. This example alone does not configure
the complete Talos device suite.

`reference_velocity` is a velocity-only observation with no implied floor/range.
Optional `inclination_limit` requires `maximum_angle_rad` in [0, pi] and accepts
unit `sensor_axis` and `reference_axis_world` (both default −Z). Omit the mapping
for unconditional acquisition. It is independent of the `dvl` finite-bottom query.

Camera/stereo/sonar model names are currently rejected as unsupported. Their future
adapters must register concrete decoders and model factories rather than create
nonfunctional entries that appear to be supported.

## C++ construction and extensions

```cpp
#include <robotics/config/scenario.hpp>
#include <robotics/sensors/readings.hpp>

auto scenario = robotics::config::loadScenario("content/examples/profile_pool.yaml");
auto runtime = robotics::config::makeRuntime(scenario);
auto pressure = runtime->stream<robotics::sensors::PressureReading>("pressure");
runtime->advance(25);
auto samples = pressure->drain();
runtime->reset(scenario.initial, scenario.seed);
```

The complete installed consumer is in `examples/profiles`:

```sh
cmake -S examples/profiles -B build/profile-demo -DCMAKE_PREFIX_PATH="$PWD/install"
cmake --build build/profile-demo
./build/profile-demo/profile_demo install/share/robotics_platform/examples/profile_pool.yaml
```

Link `RoboticsPlatform::config`. The optional config library depends on sensors
and privately on yaml-cpp; numerical/sensor libraries still require neither YAML
nor ROS. `RP_BUILD_CLI=OFF` omits the loader and runner.

A resolved `SensorPlan` contains a device, model label, and a typed attachment
factory. `makeRuntime` supplies the authoritative device and resolved world values;
the factory owns typed model configuration and attaches its device to the runtime.
Factories must register their declared device once and must not retain references
to constructor arguments or reenter the runtime. Every runtime owns freshly reset
models. Applications can compose their own C++ factories/models without changing
a sensor-kind enum. Typed `stream<Reading>(id)` lookup checks the stored type and
rejects unknown IDs or mismatches, including after configuration-based construction.

Native YAML decoders and CSV exporters currently register the four built-in families
at the configuration/application edges. Extending YAML support requires a compiled
decoder in that edge layer; there is no public dynamic plugin loader or Python model
registration yet. The [Python API](PYTHON.md) can construct and use the built-in runtime. The model label selects decoding/export only, not acquisition or
physics. Public payloads remain typed. The internal `std::any` holds typed stream
handles for checked lookup, never measurement values or arbitrary parameter maps.

## Sensor CSV

`--sensors PATH` writes a long-form CSV alongside trajectory stdout. Each delivered
sample generates named field rows containing generation, device/frame, sequence,
physics tick, scheduled/acquired/delivered nanoseconds, validity/reason, field, unit,
and value. IMU/DVL vectors use `.x/.y/.z`; covariance fields use `.row.column`.
FOG rates use configured axis indices. Pressure exports absolute pressure, depth,
and both variances. An unavailable reading produces one row with its reason and
an empty field/unit/value, not valid zeros. Strings are CSV-escaped.

The runner drains every device each tick whether recording is enabled or not.
This keeps observer choice from affecting acquisition/noise and bounds ready queues.
Pending queues still obey their configured capacity and overflow policy. At run
end, samples awaiting latency remain pending; the runner does not advance extra
time to flush them. The shipped example ends with 300 IMU, 150 FOG, 29 delivered DVL,
and 60 pressure acquisitions. Output writes/flushes are checked. The output may
replace an existing CSV, but cannot overwrite a source profile (including aliases).

This CSV is a diagnostic export, not a general recording/playback protocol, frame
graph, or viewer transport. It does not replace direct typed API access.

### Optional flow and propeller immersion

The world `pool` may add `current_oscillation_amplitude_m_s: [x, y, z]` and
`current_oscillation_frequency_hz`. Defaults are zero. These augment `current_m_s`
with a spatially uniform sinusoidal flow and its physical acceleration.

A robot thruster may add a positive `propeller_radius_m`. It selects the immersed
disk-area thrust model against the world's water surface, recalculated at every
integration stage. Without it, actuator thrust is unmodulated. Geometry remains
robot-owned and water/flow remain world-owned. This is not a ventilation/cavitation
model. See [the numerical reference](reference/STAGE_DYNAMICS.md).

### Actuator calibration

Each thruster may declare `deadband_n` (default 0), `forward_scale` and
`reverse_scale` (default 1), and `efficiency` (default 1, in [0,1]). A command with
magnitude strictly below deadband becomes zero, then sign-specific scaling is
applied, then asymmetric saturation, then efficiency. Delay/response dynamics
operate on that calibrated target. These are robot-owned model parameters.

`Plant::stopThrusters`, `Runtime::stopThrusters`, and Python `stop_thrusters()` clear
queued commands and targets immediately. Realized force coasts down during later
steps; time, generation and sensor schedules remain unchanged. A later explicit
command is accepted normally. Stop is not a latched kill/arming policy: optional
robot mechanism/integration components own that policy and command authorization.

### Contact selection and geometry

Scenario `contacts` accepts `model: disabled`, `sphere_pool` (default), or `box_scene`,
plus `restitution` (default 0.1) and `friction` (default 0.4) for box response. Robot
`vehicle.collision_boxes` and world root `collision_boxes` are ordered lists of
`{id, size_m, center_m, orientation_wxyz}`. Centers/orientations are COM-local for
body proxies and world-frame for static geometry. IDs must be unique per list;
box dimensions must be positive and orientations valid when box contacts are selected.

The loader validates schema regardless of selection; the plant validates the
selected numerical model. `collision_radius_m` is optional (default 0.2) and only
applies to `sphere_pool`. Disabled/box scenes impose no sphere containment test.
Box intersections are allowed initially and depenetrated when stepped. Geometry
order affects sequential resolution and is preserved. Lists are limited to 4,096
proxies each; that storage limit is not a workload guarantee. See
[BOX_CONTACTS.md](reference/BOX_CONTACTS.md) and `content/examples/contact_pool.yaml`.
Collision geometry does not replace the separate DVL floor-query provider.

### Named rigid frames

Robot root `frames` optionally declares `{root, transforms}`. Each transform has
`parent`, `child`, `position_m`, and `orientation_wxyz`. The root denotes COM;
child poses map into parents. A sensor may use `mount_frame` instead of an explicit
`mount`. Resolution and validation finish during loading; factories retain model
values, not tree/file references. `Scenario::body_frames` exposes the immutable tree.
See [FRAMES.md](FRAMES.md) for units, limits, Python access and viewer composition.

### Scenario world placement

Schema2 scenarios may add an upright placement for reusable world geometry:

```yaml
world_placement:
  position_m: [0, 19.5136, 0]
  yaw_rad: -1.5707963267948966
```

This is the original competition pool-to-map placement, kept outside the pool
profile. Omission is identity. The loader moves/rotates world-owned collision
boxes, sets the pool corner/heading, and adds translation Z to water level once.
Pool dimensions, density and atmospheric pressure remain unchanged. Both sphere
pool contacts and DVL finite-floor queries use the same footprint placement.
Only yaw is supported: the fluid model requires a horizontal free surface.

Initial robot state is already simulation-world-relative; body proxies and mounts
remain COM-relative. None is transformed by world placement. Future task/course
placement is separately composed and must not receive this transform twice.
World profile box poses are local before resolution; resolved `BoxProxy` poses
are simulation-world-relative.

`current_m_s` and oscillation amplitude are explicitly simulation-world vectors,
as in the original model. Placement does not rotate them. Water-level Z translation
affects buoyancy, propeller immersion, pressure and floor queries consistently.
Programmatic C++/Python Pool values expose `origin_xy_world` and `yaw_world`;
`water_level` is already world Z and must not receive another origin offset.

Placed footprint boundaries allow only a scale-aware floating-point rounding
tolerance (16 double epsilons times the coordinate/extent scale). This keeps exact
rotated edge positions legal; it is not a physical collision margin. Identity
footprint queries retain their original strict bounds.
