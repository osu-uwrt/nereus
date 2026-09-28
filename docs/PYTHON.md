# Python simulation API

The optional `robotics-platform` wheel exposes the existing C++ plant/sensor runtime
as `robotics_platform`. It supports profile loading and programmatic construction,
explicit commands/stepping/reset, typed IMU/FOG/DVL/pressure streams, and NumPy values.
It does not implement training, rewards, vectorized environments, Python model
callbacks, or a viewer. The C++ libraries still build without Python.

## Install locally

On the tested Ubuntu 22.04 platform, install the native build dependencies and use
a virtual environment:

```sh
sudo apt-get install cmake g++ python3-dev python3-venv libeigen3-dev libyaml-cpp-dev
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/python examples/python/run_profile.py
```

The PEP 517 build installs its own build-time pybind11/scikit-build-core dependencies;
NumPy is the Python runtime dependency. Python 3.10+ is declared; local verification
currently covers CPython 3.10.12, GCC 11.4, and aarch64 Linux. Wheels are specific to
Python ABI/platform and currently require the host's yaml-cpp shared library (the
Ubuntu build links `libyaml-cpp.so.0.7`). They are not repaired manylinux/universal
binaries. Eigen headers and a compiler are only needed when building from source.

The wheel includes example profiles, type hints (`py.typed` and `.pyi`), and the
existing license/provenance documents. This remains local development under the
repository's unresolved [license status](../LICENSE.md); no artifact was published.

## Profiles and explicit execution

```python
import robotics_platform as rp

scenario = rp.load_scenario(rp.example_scenario())  # pathlib.Path also accepted
runtime = scenario.create_runtime(seed=42)
pressure = runtime.pressure_stream("pressure")

runtime.advance(25)
sample = pressure.latest()
if sample is not None and sample.value is not None:
    print(sample.header.acquired_ns, sample.value.absolute_pressure, sample.value.depth)

pressure.drain()
runtime.reset(scenario.initial, seed=42)
```

`load_scenario` accepts your own [native profiles](PROFILES.md). Resolution finishes
before construction; deleting the files afterward does not affect a created or
subsequently created runtime. `Scenario` exposes copied `plant`, `initial`, `commands`,
`sensors` (device/model metadata), and `sources`, plus read-only seed/tick/surface
pressure values. `create_runtime(initial=None, seed=None)` uses the profile values
unless explicitly overridden and creates independent native state each time.

Loading a scenario does not automatically execute its command schedule. Call
`command` at its declared tick boundaries, then `advance`; the complete installed
example does this and drains each device once per tick. `scenario.ticks` is a run
length for the caller, not an implicit background loop. Commands are forces in
thruster profile order, newtons. Observations use the same SI/frame conventions as
[the C++ API](PLANT.md).

## Programmatic construction

```python
import robotics_platform as rp

parameters = rp.PlantParameters()
parameters.body.mass = 10.0
initial = rp.BodyState()
initial.position = [5.0, 5.0, -2.0]
initial.orientation_wxyz = [1.0, 0.0, 0.0, 0.0]

runtime = rp.Runtime(parameters, initial, seed=42)
imu = runtime.add(rp.Device("imu", "imu_link", period_ns=10_000_000), rp.Imu())
pressure = runtime.add(
    rp.Device("pressure", "pressure_link", period_ns=50_000_000),
    rp.Pressure(rp.PressureParameters(), rp.HydrostaticPressure(0.0)),
)
runtime.advance(25)
print(imu.stats.delivered, pressure.stats.delivered)
```

`PlantParameters`, `BodyParameters`, `Pool`, `Thruster`, `BodyState`, `Mount`, and
noise/model parameter classes expose the corresponding C++ fields. NumPy arrays or
compatible sequences are accepted. Body orientation and mount orientation use
`orientation_wxyz`: body-to-world and sensor-to-body, respectively. All time values
use integer `*_ns` fields; no float seconds or Python `timedelta` conversion loses
nanosecond precision. `seed`, ticks, and sequences retain 64-bit integer semantics.

`Runtime.add(device, model)` has overloads for `Imu`, `Fog`, `Dvl`, and `Pressure`.
FOG accepts one to three configured axes. Programmatic DVL construction requires
its `DvlParameters` and `Pool`; pressure requires its `PressureParameters` and a
`HydrostaticPressure` environment. These providers copy their inputs. The Python
layer does not currently accept arbitrary Python query callbacks or custom models.
Construct a new model for different immutable calibration/configuration.

Register before the first nonzero advance. Typed lookups are `imu_stream(id)`,
`fog_stream(id)`, `dvl_stream(id)`, and `pressure_stream(id)`; wrong type or unknown ID
raises `ValueError`. There is no central family switch in native acquisition.

## Ownership, observations, and failures

- A runtime copies construction parameters/model state. Later changes to input
  configuration cannot modify it. Each runtime owns its own plant and noise state.
- Mutable configuration objects expose scalar/nested object fields normally.
  Array getters and thruster-list getters return copies: assign a whole property
  to change it (`state.position = [...]`, `parameters.thrusters = [...]`). Editing
  `state.position[0]` changes a temporary array, not that configuration object.
- Observations, sample headers, readings, counters, and resolved scenario values
  are detached values. Arrays retain detached data; they remain valid after the
  source observation or runtime is destroyed. Editing them cannot affect live state.
- `latest()` returns the newest delivered sample or `None` before delivery/reset.
  A sample's `value` is the typed reading or `None` for an unavailable acquisition;
  `unavailable_reason` explains the latter. These are distinct states. `drain()`
  consumes queued samples, not random numbers; it does not clear `latest()`.
- Samples preserve generation/sequence/tick and scheduled/acquired/delivered
  timestamps. Pending/ready capacities and overflow policies remain the native
  [sensor contracts](SENSOR_RUNTIME.md). Long advances still need sufficient queue
  capacity, explicit dropping, or smaller batches with draining.
- Native invalid configuration/commands become `ValueError`; conversion/shape errors
  generally become `TypeError`; arithmetic/time overflow becomes `OverflowError`.
  A runtime/model failure becomes `RuntimeError`, invalidates all streams, and
  requires a successful reset. Inspect `faulted` and `observe()` for its last state.
  Completed ticks are not rolled back when a later operation fails.
- Streams can outlive the Python runtime wrapper. Once that wrapper is destroyed,
  their `active` property is false and queues/latest are cleared. Retained readings
  stay valid. There are no background workers or external handles to close; ownership
  follows Python object lifetime and C++ RAII, not a hidden singleton runtime.

Calls retain the CPython GIL. The synchronous runtime is not concurrently callable;
this binding does not advertise free-threaded Python or concurrent shared-instance
access. Use bounded advances for responsive orchestration. Parallel training workers,
GIL release with per-instance synchronization, and async rendering are future work,
not implicit behavior. Native model objects contain no Python callbacks or ownership
cycles. The standalone viewer does not import this simulation wheel merely
to use live/recorded visualization contracts.

## Verification and development

```sh
.venv/bin/python -m pip install '.[dev]'
python3 tools/check.py --install-check --tidy
.venv/bin/python tools/check_python.py --tidy --reference-runner build/release/robotics-sim
.venv/bin/python tools/check_python.py --sanitizers --reference-runner build/release/robotics-sim
```

The Python checker creates an sdist, builds its wheel outside the checkout, removes
the copied source/build trees, and installs into a fresh disposable environment.
It runs the copied tests and example using isolated Python mode, without a sourced
ROS/Python overlay. Omit `--reference-runner` only when deliberately skipping the
cross-language comparison. Build/lint/type-check dependencies come from the `dev`
extra. The C++ contributor checks additionally need the prerequisites in
[CONTRIBUTING.md](../CONTRIBUTING.md); `--tidy` additionally needs system clang-tidy.

The binding tests cover model construction, force response, quaternion conventions,
exact integer timing, seed/reset replay, unavailable data, fault recovery, array
ownership, stream lifetime, and profile independence. A separate C++ runner provides
trajectory and sensor-field references for the whole example. Cross-build comparisons
use 1e-12 relative/absolute numerical tolerance because debug and optimized Eigen
arithmetic can round differently; timestamps and same-build replay remain exact.
Ruff and strict mypy check the package, public stubs, tests, and example. Native checks
include warnings-as-errors and configured clang-tidy analysis on the bindings.

Sanitizer checks build native code with ASan/UBSan and preload the sanitizer/C++
runtimes before CPython imports the extension. LeakSanitizer is disabled for that
embedding check because CPython/process teardown is outside the native ownership
contract; it is not a leak-free-process claim. Address/undefined-behavior diagnostics
remain enabled. No sanitizer settings are applied to normal user installations.

A local release-build smoke measurement of a passive body took about 10 ms for
10,000 steps in one call and 30 ms for 10,000 single-step calls. This checks binding
granularity only: it is not a throughput guarantee, sensor workload benchmark, or
claim of training-scale performance. Physics substeps and sensor scheduling remain
in C++; Python controls when to command, advance, consume, and reset.

## Contact models

`parameters.contacts` exposes `ContactParameters`: `model`, `body_boxes`,
`world_boxes`, `restitution`, and `friction`. Choose `ContactModel.DISABLED`,
`SPHERE_POOL` (default), or `BOX_SCENE`. Each `BoxProxy` exposes `id`, `size`,
`center`, and `orientation_wxyz`; assign complete box lists because getters return
copies. Body proxy poses are COM-local; static proxy poses are world-frame.
Box scenes allow initial overlap and resolve it on advancement. Unselected sphere
radius/containment constraints do not apply. See the [contact contract](reference/BOX_CONTACTS.md)
for ordering, sensor-acceleration semantics and current limitations.
