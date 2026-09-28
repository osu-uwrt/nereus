# Free-motion stage reference

The fixture `tests/fixtures/legacy_stage_dynamics.csv` contains 753 noiseless states
captured from original simulator revision `07647eebe706f96ea7b76db3cc9802735a146698`.
The adjacent JSON records all input source hashes, driver/fixture hashes, and compiler.
It is an offline reference; normal builds/tests do not discover or link the old repo.

Reproduction, explicitly supplying a checkout containing the pinned commit:

```sh
python3 tools/capture_stage_reference.py /path/to/riptide_simulator
```

The capture tool reads committed bytes through `git show`, builds the original
standalone marine/actuator kernels in a temporary directory, and extracts the actual
`Robot::stateDerivative`, `Robot::propulsionWrench`, and RK4 stage expressions.
Only the enclosing class name changes. No ROS headers, node, or graphics are built.
The authored test driver supplies identical synthetic model inputs to those equations.

There are three starting conditions: partially immersed propeller, fully submerged,
and above-water entry. Each uses 250 steps at 2 ms, a rotating/translating body,
a mounted propeller with 50 mm disk radius, sinusoidal current, and force commands
at ticks 0, 100, and 200. Rows are observed before issuing each boundary command.
The driver source contains the complete numerical configuration and chronology.

Columns are case/tick, 13 COM state coordinates, actuator force, COM inertial
acceleration in body axes, and angular acceleration in body axes. The acceleration
columns are not published legacy IMU messages: gravity/sensor-mount/message-frame
conversion remains a separate comparison. Integer timestamps derive from tick×2 ms.

The native comparison checks every numerical field with absolute tolerance 2e-12
on the local GCC/aarch64 builds. The old actuator accumulates floating-point time;
the new environment evaluates the authoritative simulation tick and RK4 offset.
Their small clock accumulation differences are included in this tolerance for
these short runs. Longer runs can accumulate larger differences; the new platform
retains authoritative tick time rather than reproducing accumulated clock drift.

This fixture establishes free-motion stage forcing for these synthetic cases.
It does not establish Talos parameter fidelity, collision response, stop/timeout,
placement/reset semantics, sensor noise, ROS products, or full mission behavior.
Those acceptance checks remain open.

## Native model choices

`Thruster::propeller_radius` optionally selects disk immersion against the horizontal
water surface. The disk rotates with the body; each RK4 stage recomputes its center
height and projected vertical extent. The old model's 1 mm minimum extent is retained
for nearly horizontal disks. Air thrust is neglected when this model is selected.
Absent radius leaves the configured actuator force unmodulated.

`Pool::current_velocity` is the mean world velocity. The optional oscillation adds
`amplitude * sin(2*pi*frequency*time)` and supplies its derivative to hydrodynamics.
Frequency is in Hz, amplitude in m/s, with phase zero at full-reset time zero. Zero
frequency disables oscillation. This is a spatially uniform flow model, not turbulence.
The same stage-aware derivative is used for endpoint kinematics/sensor acquisition.
Actuator force remains held at its midpoint during each RK4 body step.
