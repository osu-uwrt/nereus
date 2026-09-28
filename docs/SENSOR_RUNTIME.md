# Standalone sensor runtime

The `RoboticsPlatform::sensors` C++17 library composes the plant with IMU, FOG,
pressure/depth, and ideal bottom-track DVL models. It requires Eigen and the simulation library;
no ROS, rendering, viewer, YAML, wall clock, or worker thread is involved. These
are synthetic measurement models, not calibrated device emulators. Cameras,
stereo capture and Python bindings are future work. Native sensor-profile loading
is available in the optional [configuration library](PROFILES.md).

## Ownership and extension

Create `Runtime(parameters, initial_state, seed)`, register devices with
`add(Device, model)`, then use `command`, `advance`, `observe`, and
`reset(initial_state, seed)`. Each model is owned by the runtime and has its own
noise/history. Registration closes on the first nonzero advance and stays closed
across resets. Reconfigure by constructing another runtime. Zero devices is valid.

A model is an ordinary value type with a `Reading` type, `reset(seed, id)`, and
`sample(const MotionSample&, elapsed_seconds)` returning `Measurement<Reading>`.
A measurement contains either a reading or a nonempty unavailable reason, never
both. Readings must be copyable; large future payloads can own immutable shared
buffers. Models receive authoritative motion and can compose additional providers,
such as a bottom query. No central sensor enum or family-specific scheduler branch
is needed. The internal virtual interface erases only scheduling/lifecycle types;
measurement models do not inherit from it or from each other.

The data-only `types.hpp` and `readings.hpp` headers contain no simulation, model,
ROS, or viewer imports. Future live/recorded sources can reuse these payloads
without constructing a plant or linking the sensor model implementation. These
are version-0.1 in-process C++ contracts, not a serialized wire format or stable ABI.

`add` returns a typed stream handle. `stream<Reading>(id)` retrieves that same
handle with checked ID/type lookup, including for profile-created sensors. `latest()` and `stats()` return copies;
`drain()` consumes delivered samples. None of these methods advances physics,
acquires a sample, or consumes random numbers. Independent consumers should use
an adapter that fans out values: a stream has one shared consumption queue, not
an implicit subscription per handle. Handles may outlive the runtime and become
inactive on its destruction. All access is single-threaded; callers must provide
external synchronization if needed.

## Time, queues, and failure

- Durations are signed integer nanoseconds. A device period must be at least one
  physics tick. First acquisition is due at one period, not at time zero.
- Deadlines advance by the configured period. A nonintegral deadline acquires on
  the first physics tick at or after that deadline, using that tick's motion.
  This quantizes timing without accumulating drift; there is no interpolation.
- A header preserves scheduled, actual acquisition, and actual delivery times,
  plus device ID, frame label, reset generation, acquisition sequence, and tick.
  Sequences start at zero and count unavailable acquisitions too.
- Constant nonnegative latency is measured from actual acquisition. Delivery
  occurs on the first tick at or after acquisition plus latency. Latency changes
  delivery time, never the captured reading. Jitter/dropout models are not supplied.
- Each device has separate pending and delivered queues, each bounded by
  `capacity` (default 64). The default overflow policy faults the runtime.
  Explicit `DropOldest` discards the oldest entry in the overflowing queue and
  increments the appropriate counter. With capacity smaller than latency backlog,
  this can discard every acquisition before delivery. Size queues intentionally.
- Unavailable measurements are normal scheduled results; provider exceptions,
  invalid model outputs, queue failures, and numeric/time overflow are errors.
  Errors during advance/reset invalidate and clear every stream and fault further
  commands/advancement until successful reset. The plant snapshot remains available.
  Completed physics ticks are not rolled back if sensor acquisition then fails.
- Invalid registration, invalid plant reset input, and an overflowing advance
  request are rejected before mutation. A model's failing reset can occur after
  the plant generation has advanced; all streams remain inactive until recovery.
- Reset clears pending/ready data, latest values, counters, deadlines, sequence,
  drift, and RNG caches. The generation distinguishes readings from earlier runs.
  Reset does not change mounts, calibration, rates, or provider configuration.

Providers must be deterministic functions of the acquisition inputs and their
explicit configuration. The supplied pool provider is a copied immutable value.
Custom stateful providers must arrange their reset through their owning model;
the runtime cannot reset mutable state hidden in a captured external callback.

## Motion and measurement conventions

All quantities use SI units. World Z is up; body X is forward, Y left, Z up.
`Mount.position_body` is measured from the COM. Its unit quaternion maps sensor
coordinates to body coordinates. Configured quaternions and axes must be unit
length within 1e-9; accepted rounding error is normalized away. `Device.frame` labels the resulting sensor frame;
there is no frame graph or automatic frame-name resolution in this increment.

`Plant::motion()` reads the committed snapshot and evaluates derivatives using
its committed actuator force. COM inertial acceleration in body coordinates is
`dv_body/dt + omega_body × v_body`, not just the derivative of body-frame velocity.
It exposes angular acceleration and world gravity as well. This is an instantaneous
endpoint derivative, not an average over the preceding integration tick or an
exposure interval. Reading it cannot advance the plant. At a pool contact boundary
(with 1 nm positional tolerance), acceleration is marked unavailable because the
impulse model has no instantaneous contact-force history. A faulted plant rejects
motion sampling until reset.

**IMU:** reports sensor-frame angular velocity and specific force. For offset `r`,
point acceleration is `a_COM + alpha × r + omega × (omega × r)`; specific force
subtracts gravity before rotating to the sensor frame and applying sensor noise.
An aligned supported body at rest reads +9.80665 m/s² on Z; free fall reads zero.
This follows the raw measurement convention described in
[REP-145](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0145.rst),
without depending on ROS. No orientation estimate is fabricated. When acceleration
is unavailable, the whole IMU reading is unavailable; noise history still advances.

**FOG:** projects angular rate and its covariance onto one to three configured unit
axes in the sensor frame. Axes can be nonorthogonal, with corresponding covariance.
It reports rates, not integrated heading or an attitude estimate. No privileged yaw
channel, Earth-rate model, temperature dependence, or manufacturer protocol exists.

**DVL:** computes the mounted point's velocity relative to the returned bottom's
world velocity, then rotates it into the sensor frame. The narrow bottom query takes
world origin/unit direction and returns an optional distance and bottom velocity.
The included provider intersects a finite stationary pool floor, rejecting rays from
outside the pool/water column and rays that miss the floor. Minimum/maximum range
are inclusive. Missing or out-of-range bottom produces an unavailable reading;
there is no ground-truth or water-track fallback. Invalid provider results fault
acquisition. `bottom_distance` is ideal slant range along the configured axis, not
vertical altitude or a noisy acoustic measurement. This single-ray lock model does
not simulate beam geometry, partial beam lock, reflectivity, sound speed, or noise
in range. Replacing its provider does not require changing the scheduler.

**Pressure/depth:** reports absolute pressure (Pa), its variance, and depth (m,
positive down) derived from that same measured pressure. A narrow environment
query receives the mounted point in world coordinates. The supplied hydrostatic
provider uses constant density and gravity below a horizontal surface and constant
atmospheric pressure above it: `P = P_surface + rho*g*max(depth, 0)`. This is the
constant-density specialization of the [hydrostatic relation](https://www.gfdl.noaa.gov/wp-content/uploads/files/model_development/ocean/guide4p1.pdf).
It does not model waves, flow-induced pressure, compressibility, or temperature.
The planar provider does not resolve pool walls or finite fluid volumes.

Depth conversion has independent reference pressure, density, and gravity. Noise
and bias affect pressure before depth conversion; depth is never substituted from
world position or clamped to zero. This permits negative estimated depth near the
surface and calibration error. Pressure/depth uncertainty is fully correlated,
not two independent observations. Scalar pressure noise reuses one component of
the shared noise process. Missing environment and out-of-range pressure are
unavailable results; invalid provider values are errors. Both ideal and measured
pressure must fit the inclusive configured range. Mount rotation affects the
world position of the offset through body attitude; pressure itself is scalar.

## Noise and reproducibility

Each three-axis noise component combines configured fixed bias, independent Gaussian
white noise per acquisition, and Gaussian bias random walk scaled by the square root
of elapsed acquisition time. Parameters are in the output's units; white standard
deviation is per sample, not a noise density. Bias is a known configured offset and
is not included in covariance. Reported covariance is diagonal white variance plus
accumulated random-walk variance; FOG projects that matrix onto its configured axes.
It does not describe temporal cross-correlation or uncertainty in the physical model.
Noise parameters are specified in the sensor frame, before FOG axis projection.

Stable length-framed hashing derives a stream from root seed, device ID, and component
name. Per-instance `mt19937_64` engines and normal-distribution caches restart on reset.
Adding another device, changing registration order, changing another device's rate,
or polling a stream does not perturb an existing device. Unavailable acquisitions
still evolve noise. Direct model users should call `reset(seed, id)` before sampling
and supply the elapsed time between acquisitions. Replay is tested within a supported
build; normal distribution algorithms and floating-point behavior are not promised
bitwise identical across standard libraries, compilers, or architectures.

## Installed example

After building and installing the project:

```sh
cmake -S examples/sensors -B build/sensor-demo -DCMAKE_PREFIX_PATH="$PWD/install"
cmake --build build/sensor-demo
./build/sensor-demo/sensor_demo > build/sensors.csv
```

The example composes a passive moving body with a 100 Hz IMU, 50 Hz FOG, and 10 Hz
DVL with 20 ms latency, plus a 20 Hz pressure sensor. It drains queues during stepping and verifies delivery counts
and reset clearing. Public-contract tests cover frame/lever-arm equations, specific
force, bottom lock, noise replay, independent schedules, queue policies, and failure
recovery. The install check copies and builds this example against a relocated prefix.
