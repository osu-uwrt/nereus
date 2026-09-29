# Native Talos dynamics and competition pool slice

The standalone `content/robots/talos_dynamics.yaml` imports the original physical
parameters, eight thrusters, effective hull/poker collision proxies and named rigid
mounts. `content/worlds/competition_pool.yaml` owns the original water properties
and five finite pool boxes. `content/examples/talos_pool.yaml` selects the original
2026 pool-to-map placement independently and starts COM at [0,0,-1], as the original
startup does. It applies a new illustrative three-second force schedule, not a
recorded mission or controller sequence.

```sh
build/release/robotics-sim content/examples/talos_pool.yaml > build/talos.csv
```

The files are native data. Loading/running/installing them requires neither ROS,
UWRT packages, URDF parsing, the original repositories, nor Python/YAML conversion
tools. The installed Python wheel also contains this example. Profiles are resolved
before execution and remain usable after their source files are removed.

## Separate inertial assembly

`content/robots/talos_inertial.yaml` selects the same physical robot and fixed
mounts with a 50 Hz AHRS and a 500 Hz sensor-Z FOG. Its reusable
`content/sensors/talos_ahrs.yaml` preserves original noise-enabled settings, gravity
reporting calibration, explicit orientation/acceleration/gyro covariance and zero
heading drift. It uses the same independently selected pool and placement:

```sh
build/release/robotics-sim content/examples/talos_inertial_pool.yaml --sensors build/talos-inertial.csv
```

The offline importer materializes both robot assemblies from one physical-data
conversion; there is no runtime inheritance or implicit override merge. The
sensor-free example remains the numerical dynamics reference. `imu_mount` and
`fog_mount` are native acquisition frames; future ROS adapters map them to the
required stack frame names. Sampling and noise never depend on that adapter.

This inertial assembly has two devices; the navigation assembly below adds DVL
and original depth reporting. Cameras/stereo remain open. The three-second inertial
example acquires 150 AHRS and 1500 FOG samples while preserving the sensor-free
plant trajectory. Tests verify original mount/calibration/covariance settings,
source-file removal and native reset replay. An independent [prescribed-state sensor reference](SENSOR_KINEMATICS.md) now
verifies AHRS/FOG formulas; complete trajectory/transport comparison remains open; sample-by-sample equality to the old shared RNG is not claimed.
See [TALOS_SENSOR_CONVERSION.md](TALOS_SENSOR_CONVERSION.md) for conversion limits.

## Navigation assembly with original DVL and depth policies

`talos_navigation.yaml` adds the 8 Hz device using `talos_dvl.yaml`, whose declared
model is `reference_velocity`: a stationary-world velocity observation with original
noise/variance, no range and no floor or inclination gate. The original default
publishes precisely this product. The physical finite-floor DVL is still available
through its separate model. Run the navigation assembly with:

```sh
build/release/robotics-sim content/examples/talos_navigation_pool.yaml --sensors build/talos-navigation.csv
```

This example produces 150 AHRS, 1500 FOG, 24 velocity and 60 depth samples in three seconds.
The 125 ms device period rounds acquisitions onto the 2 ms physics lattice without
phase drift (126 ms, 250 ms, 376 ms, ...).

The 20 Hz depth device selects `reference_altitude`, with the original .010 m
sampling standard deviation and .0001 m² reported variance. It measures the mounted
point's world Z and corrects to the configured base point using the same pose.
The target is the native COM-local base offset; the model has no robot/frame-name
conventions. Both outputs share one noisy observation. This preserves the original
unbounded world-Z product, including above-water positions. It is not raw pressure;
physical pressure sensing remains separately selectable. Cameras remain open.
The independent prescribed-state reference now verifies velocity and reported
variance plus both depth heights and variance, as well as AHRS/FOG. ROS names/messages remain outer adapter work.

## Bounded delivery and remaining work

This is a physics/mount slice, not the complete Talos simulator replacement:

- The dynamics-only assembly explicitly has `sensors: []`; inertial and navigation
  assemblies add the documented devices. Camera acquisition and stereo pairing
  remain to be connected.
- Camera frames describe mounting extrinsics only, not optical transforms or stereo
  baselines. Mechanism frames carry mount coordinates, not mechanism dynamics.
- Original body/rotor/LED/prop assets, water rendering, task geometry/scoring and ROS
  integrations are not instantiated here. Their conversion map remains separate.
- Hydrodynamic coefficients retain the original `unvalidated_prior` status. Matching
  these simulation parameters is not evidence of hardware identification.
- Atmospheric pressure101325Pa is the platform default, not an original pressure
  sensor measurement model. It does not affect this sensor-free plant trajectory.

Native tests verify selected source values, full thruster ordering, original source
angle conventions, COM/CAD/base/device offsets, raw effective collision origins,
pool geometry and independent deterministic execution after source removal. The
1500-step example produces finite states. An independent pinned-original capture now
compares every tick of three three-second trajectories (4503 states, including the
initial states). See [TALOS_DYNAMICS.md](TALOS_DYNAMICS.md) for reproduction,
optimization sensitivity and the bounded numerical acceptance scope. Sensors, course
contacts and mechanisms remain outside this comparison.

## Reproduce the offline conversion

The importer reads pinned Git objects, not uncommitted source files or ROS-expanded
profiles. It requires Python3 and PyYAML (capture used6.0.1). Revisions, source hashes,
output hashes and importer identity are written by the importer (not kept in the repo).

```sh
python3 tools/import_talos_physics.py /path/to/riptide_simulator /path/to/riptide_core
python3 tools/import_talos_physics.py /path/to/riptide_simulator /path/to/riptide_core --check
```

The first command regenerates the nine profiles and provenance record; the second
checks exact generated bytes without writes. Both repositories are development-time
reference inputs only. Different serializer versions may require an explicit
reviewed regeneration. Source license status is unchanged from [PROVENANCE.md](../PROVENANCE.md).

The reviewed [numerical map](TALOS_CONTENT_CONVERSION.md) explains which resolved
hydrodynamic settings override historical controller/simulator defaults. The
[visual map](VISUAL_CONTENT_CONVERSION.md) specifies the exact resources and transforms
that must follow this pack. Neither copied controller gains nor convenient substitute
meshes establish that acceptance.
