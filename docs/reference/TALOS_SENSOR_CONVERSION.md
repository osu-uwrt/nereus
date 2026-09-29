# Talos sensor reporting conversion

The reference revisions are the simulator and vehicle-description pins in
[TALOS_CONTENT_CONVERSION.md](TALOS_CONTENT_CONVERSION.md). Inputs are
`c_simulator/src/{robot_class,physics_simulator}.cpp`,
`c_simulator/robots/talos/config/sensors.yaml` and
`riptide_descriptions/config/{talos,simulator}.yaml`. Sensor overrides take precedence
as in the original profile composition. These values describe the original simulator,
not hardware identification. The separate native Talos inertial assembly now instantiates AHRS and FOG; the
sensor-free dynamics assembly remains available for physical comparisons.

## Raw IMU reporting implemented

The original `setAccel` stores world inertial COM acceleration plus physical gravity
on world +Z. Publication adds `(imuGravity - GRAVITY)` on that same axis, then
angular/tangential mount acceleration and transforms to sensor axes. Equivalently:

```text
a_mount_body = a_COM_body + alpha_body × r + omega_body × (omega_body × r)
f_sensor = R_sensor_to_body^-1 * (a_mount_body - R_body_to_world^-1 * g_report_world)
```

Native `ImuReporting` implements the measurement-only gravity magnitude and
independent reported diagonal variances. Defaults preserve physical specific force
and noise-derived covariance. Talos selects 9.755455 m/s² while dynamics retain 9.80665.
The difference rotates with body and sensor orientation; it is not a fixed sensor-axis
bias. Raw IMU remains orientation-free. See [SENSOR_RUNTIME.md](../SENSOR_RUNTIME.md).

The resolved raw IMU settings are 50 Hz, acceleration noise standard deviation 0.016 m/s²,
gyro noise standard deviation 0.01 degree/s (convert to radians), and reported variance
0.01 on each acceleration and angular-velocity axis. Reported covariance remains
present with sampling noise disabled. Sensor mount uses the original CAD-relative
pose minus COM, with original RPY values unchanged.

## Device and adapter conversion status

- Attitude and AHRS composition are implemented as separately declared models. Original orientation is
  body orientation times sensor mount orientation, with isotropic random-axis
  angle noise (standard deviation 0.5 degree), and optional world-Z heading drift.
  Resolved Talos yaw drift is 0 degree/minute. Reported orientation diagonal variance
  is [0.00005, 0.00001, 0.01]. Compose attitude and raw inertial acquisition at one
  scheduled state for the full IMU product; do not fabricate orientation
  in raw inertial observations or drive acquisitions from subscriber activity.
  The native model defaults to small-angle variance sigma²/3; conversion must select
  original reported variances (including sigma² when the old override is absent).
  Original noise-disabled mode disables heading drift too. The native Talos AHRS
  profile now selects the original noise-enabled values; independent original
  [prescribed-state output capture](SENSOR_KINEMATICS.md) now verifies noise-disabled
  AHRS and FOG formulas; full trajectory/acquisition and transport parity remain open.
- DVL runs at 8 Hz with default noise standard deviation 0.001 m/s and independently
  reported variance 0.000001. Original velocity includes `omega × r` and sensor
  rotation. Default `dvl_max_tilt=0` disables lock loss entirely: the original
  implementation does not query a floor or publish range. The current finite-floor
  bottom-track model therefore needs a separately declared reference/validity policy
  for fidelity. Do not invent a bottom hit/range to bypass its validation.
- FOG runs at 500 Hz and reports sensor Z angular rate, with independent noise
  standard deviation 0.01 degree/s. Default reported variance is
  `max(1e-9, sigma_rad_s²)`; configuration may override it independently of noise.
  Native Talos now selects the original default FOG noise and covariance (the
  noise variance exceeds the original floor). The explicit independent covariance
  override is now preserved even with sampling noise disabled.
- Depth runs at 20 Hz with 0.010 m standard deviation. The original samples the mounted
  pressure point's world Z, adds scalar noise, then corrects to base_link Z using
  the same acquisition orientation and mounted offsets. It reports world/map Z
  with variance 0.0001, not raw pressure or positive-down depth. Preserve the native
  pressure model and make the adapter conversion explicit. Above-water behavior
  needs a declared reference policy: original Z is unbounded while hydrostatic
  pressure clamps at atmospheric pressure. Do not silently treat those as equal.

Use independent device/component random streams, as required by the architecture;
restoring the original shared random generator would reintroduce coupling. Compare
noise-disabled original formulas and configured noise statistics, and explicitly
record that stochastic samples are not bit-identical to the old shared stream.
Retain timestamps/frames/validity and bounded delivery throughout. Cameras/stereo
follow through independently owned acquisition and the original render backend.
