# Original sensor kinematics reference

`tests/fixtures/legacy_sensor_kinematics.csv` records 24 prescribed COM states and
body-coordinate velocity derivatives, followed by original noise-disabled sensor
outputs. The offline capture reads pinned original Git objects and compiles extracted
sensor initialization, `Robot::setAccel`, IMU pre-noise publication, FOG rotation,
DVL velocity and mounted depth/base-link correction expressions. It does not use
native profiles, the native importer or platform model code to generate expected
values. The wrapper resolves original FOG parameter defaults without building ROS.

```sh
python3 tools/capture_sensor_reference.py /path/to/riptide_simulator /path/to/riptide_core
ctest --preset release -R OriginalNoiseDisabledInertial --output-on-failure
```

Capture requires a C++ compiler, Eigen and yaml-cpp. Normal tests use the committed
fixture and need no original workspace. Source, capture-script, driver and result
hashes plus compiler flags accompany the CSV. Inputs include rest, physical free
fall, above/below-surface positions, noncommuting rotations, nonparallel linear and
angular velocities and angular acceleration. Original COM velocity derivatives
become native inertial acceleration via `v_dot + omega × v`; omitting that term
would change mounted specific force.

## Verified and pending outputs

The current comparison checks all 25 AHRS/FOG/reference-velocity fields: specific force, angular
velocity, sign-equivalent sensor-to-world quaternion, three reported covariance
diagonals, FOG rate/variance and DVL velocity/variance. Maximum absolute error must remain below
`1e-12`. Native mounts come from the resolved Talos frame pack. Models explicitly
select zero sampling noise with the original gravity and reported uncertainty.
This gates native models and resolved mounts; profile loading/settings have separate
assembly tests and are not all exercised by this manually configured comparison.
Reported variance remains independent of sampling noise, including for FOG before
configured axis projection.

The remaining three fields capture mounted depth-point world Z, corrected base-link
world Z and depth variance. They are retained as
upcoming acceptance inputs and are **not yet verified against native models**.
Keeping both depth heights prevents the final lever-arm cancellation from hiding
an incorrect pressure-sensor mount.

This proves measurement formulas for prescribed states, not acquisition scheduling,
plant integration/contact impulses, transport, camera products or full stack
behavior. Noise-disabled original mode also disables heading drift. Independent
native random streams deliberately differ from the original shared generator;
noise-enabled distribution and replay contracts have separate tests.
