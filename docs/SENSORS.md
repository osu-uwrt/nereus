# Sensor scope and contracts

Status: the C++ runtime, IMU, FOG, pressure/depth, and ideal bottom-track DVL are implemented.
See [delivered runtime contracts and limitations](SENSOR_RUNTIME.md). This document
retains the broader requirements for subsequent phases, including cameras/stereo.
It refines the [architecture plan](ARCHITECTURE_PLAN.md); it does not add an RL interface.

## Supported families

| Family | Planned outputs and configuration | Boundary |
| --- | --- | --- |
| Camera | Images, calibration, optical frame, resolution, field of view, acquisition rate, mount, and configured image effects. | Optional render backend; usable without the viewer. |
| Stereo camera rig | Identified left/right images, individual calibration/frames, relative extrinsics/baseline, shared acquisition time and pair ID. | Compose camera models with a rig coordinator; no vendor-specific stereo layout in the runtime. |
| IMU | Sensor-frame angular velocity and specific force, with uncertainty, bias/noise, and configured mount. | Raw inertial observations; any attitude estimate is a separately declared model/output. |
| DVL | Sensor-frame velocity with a declared tracking reference, validity/lock state, uncertainty, and supported range/altitude information. | Start with a documented bottom-track model and explicit loss of lock; do not silently substitute ground truth or a water-track estimate. |
| Pressure/depth | Absolute pressure and calibrated pressure-derived depth, mounting position, noise/drift, operating range, and uncertainty. | Hydrostatic environment is separate from sensor calibration; depth is derived from measured pressure, not privileged position. |
| FOG | Angular rate for configured measurement axis/axes, with uncertainty and bias/drift settings. | Independent gyro model; not hardcoded to Talos yaw or a particular ROS message. |

Every family supports independently identified instances. Zero sensors is valid;
no robot is required to have a particular camera name, IMU, DVL, FOG, or pressure sensor. Device
calibration and noise settings belong to profiles; manufacturer protocols belong
to optional adapters. Synthetic defaults must be labeled as estimates and should
not be described as calibrated hardware fidelity.

Sonar is not in current implementation scope. Keep room for later imaging,
scanning, or range-return models through the same registration, scheduling, and
sample interfaces. DVL support is explicitly in scope and must not require a
general sonar simulator first. Defer sonar-specific payloads and acoustic models
until a concrete implementation needs them.

## Composition and shared lifecycle

Compose each instance from validated configuration, acquisition model, mount,
schedule, noise/calibration state, and delivery policy. Reuse small functions or
components where behavior actually matches; avoid a universal sensor superclass
with camera, gyro, and acoustic methods that most instances cannot implement.

The shared scheduler handles identity, acquisition tick/time, generation, frame,
sequence, and delivery state. Model-specific measurements use typed, versioned
contracts owned by their extensions. Adding another family must not require
editing the plant or a central runtime switch over device types. Do not encode
all sensor values as anonymous numeric arrays or unrestricted dictionaries.

Supply read-only kinematics and required derivatives through a sampling context.
Models must not access private plant state, infer acceleration from wall-time
callback intervals, or drive integration themselves. Keep mounting-frame and
lever-arm effects explicit. State whether gravity is included in an inertial
output; test the chosen specific-force convention at rest and during acceleration.
Keep raw gyro rates, integrated angles, and attitude estimates distinct.

Models declare the inputs they need. IMU/FOG need kinematics; a DVL may need
bottom geometry queries; camera models need rendering. Providers implement those
queries independently of ROS and the viewer. Share world-query interfaces with
future models only when semantics match; do not introduce an acoustic engine or
require every sensor to carry graphics dependencies.

Each sensor owns its history, bias, and random stream derived from the run seed
and stable instance ID. Adding a camera, changing another sensor's rate, or
subscribing from a viewer must not alter an IMU's noise sequence. Configure and
document any deliberate correlations, such as a shared disturbance within a rig.
Reset clears histories, pending samples, pair state, and delivery queues.

Acquisition and delivery times are separate. Model latency, jitter, and dropouts
in simulation time where enabled; do not replace acquisition time with publication
time. Polling does not generate another reading or consume random numbers.
Retain both validity and uncertainty; unavailable data is not a valid zero sample.
Enforce bounded queues and document delivery/drop behavior for each consumer.

## Cameras and stereo

Acquire a stereo pair against one authoritative scene state at the rig's scheduled
time. Give both images a shared pair ID and preserve their individual optical
frames/calibration. Render completion order must not pair images from different
ticks. Missing, failed, or dropped members must produce an explicit incomplete or
invalid pair result, never a silently mismatched stereo observation.

Start with a documented synchronized-exposure model; rolling shutter, detailed
exposure timing, and device-specific synchronization can be separate extensions.
Projection, distortion/rectification state, and frame conventions must be explicit
in calibration metadata. Stereo-derived disparity/depth, if provided, is a
separate processing component. Renderer-provided depth is ideal scene information
and must be labeled separately; it must not masquerade as a stereo reconstruction.

Camera output does not depend on an open viewer, active ROS subscriber, display
frame rate, selected overlays, or interactive camera pose. Live camera/stereo
viewer adapters consume images and calibration without constructing simulated
camera models or loading the physics runtime.

## Acceptance checks and sequence

Phase 2 implements scheduling, sample contracts, seeded state, IMU, DVL, FOG, and pressure/depth
models. Define camera/stereo contracts in that phase; actual camera acquisition
arrives with offscreen rendering in phase 5B. Basic DVL geometry queries must work
against the standalone world, including explicit out-of-range/no-bottom results.

| Check | Required evidence |
| --- | --- |
| IMU | At-rest gravity convention, known translation/rotation, mounting rotation and offset effects, uncertainty, and reproducible noise/bias reset. |
| DVL | Known motion relative to a stationary bottom, sensor mount/rotation, configured operating range and lock loss, invalid samples, and uncertainty. |
| Pressure/depth | Hydrostatic pressure and mounted depth, atmospheric/surface behavior, separate calibration, noise/reset, uncertainty propagation, operating limits, and unavailable environment. |
| FOG | Known positive/negative rotation projected onto configured axes, independent instance rates, bias/drift/reset, and no privileged attitude substitution. |
| Camera | Calibration/projection and optical-frame checks, acquisition timestamps, offscreen output independent of viewer appearance and subscribers. |
| Stereo | Baseline/extrinsic and disparity geometry, pair identity/timing under delayed completion, dropped-member handling, and clear ideal-versus-derived outputs. |
| All models | Multiple instances and empty lists, rate scheduling, advance partition invariance, reset-generation isolation, observation without mutation, deterministic random streams. |
| Packaging | Nonvisual models build without graphics or ROS; camera models work without the viewer; adapters do not import simulator dependencies for live visualization. |

Extend the example profiles and installed-API examples as each family is delivered.
Test behavior through public contracts. Do not claim hardware accuracy or complete
sensor support from the presence of a schema, interface, or mock alone.
