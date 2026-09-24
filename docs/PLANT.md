# Standalone plant contract (experimental 0.1)

`Plant` is a synchronous, single-owner object. The application constructs it from
validated `PlantParameters` and `BodyState`; there is no global active robot.
The object composes a private marine model, actuator model, thruster allocation,
and pool contact resolver. No inheritance-based robot hierarchy is introduced.
The public API uses Eigen value types; YAML parsing is a separate optional library.

## Coordinates and inputs

All quantities are SI. World Z points up. Body axes are X forward, Y left, Z up.
Position is at the center of mass; orientation maps body vectors into world space.
Linear and angular velocity are body-frame vectors. Thruster offsets, damping and
buoyancy centers are relative to COM. Quaternions use `[w,x,y,z]` in serialized
input; valid nonzero input orientations are normalized by the plant.

Thruster directions must be unit vectors. The allocation maps each force to
`[direction, position cross direction]`. Commands are forces in newtons in profile
order. Limits are asymmetric; delay, first-order rise/fall lag, and slew rate are
simulated. Timeout uses simulation time; zero timeout holds a command indefinitely.
A passive body with no thrusters is valid.

Pool dimensions and water properties are independent of the body. The API accepts
full symmetric added-mass and linear-damping matrices; the starter YAML format
accepts diagonal matrices only. The schema is explicitly versioned and experimental.
Required YAML fields are demonstrated in `content/examples/empty_pool.yaml`.
Unknown/duplicate fields, nonfinite numbers, malformed shapes, invalid units'
physical constraints, and unordered/out-of-range command ticks fail before output.

## Time, commands, reset, and failure

- Tick zero is the initial state. Time is an integer nanosecond timestep times tick.
- `command(forces)` queues input at the current boundary; the latest command at
  that boundary wins. Delay is simulated after this boundary.
- `advance(n)` completes exactly `n` ticks. `advance(0)` observes without mutation.
  It has no sleep, callback, ROS, or display dependency.
- `observe()` returns a value snapshot. Mutating it cannot change the plant.
- `reset(initial)` validates first, clears all actuator histories and forces,
  restarts tick/time, and increments the snapshot generation. Configuration is
  immutable; construct another plant to use different parameters.
- Invalid commands, resets, and time-overflow requests fail before mutation.
  Numerical/contact failure during advancement leaves the last completed snapshot
  available and faults further commands/advancement until a successful reset.
  A multi-tick call does not roll back ticks completed before a numerical failure.
- Methods are not concurrently callable. Own separate instances or synchronize
  outside the library. Instances have independent state, queues, and time.

No randomness is used in this slice, so no artificial seed argument is exposed.
Sensor noise and seed-stream contracts belong to the subsequent runtime work.
There is no task-only reset until task models exist.

## Numerical model and limitations

The private reference model includes rigid/added mass, Coriolis terms, dissipative
linear/quadratic damping, gravity, and partial-ellipsoid buoyancy. The initial
plant uses a constant world current. Propulsion is a midpoint actuator wrench
held through each RK4 body step; actuators advance half a tick before and after it.
This is an intentional operator-splitting choice, not bitwise parity with the old
ROS node's stage-dependent thrust implementation. Abrupt actuator transitions
reduce the combined method's order; compare step refinements for sensitive uses.

The initial contacts are a COM-centered sphere against four vertical pool walls
and a floor. There is no water-surface collision ceiling. Wall contact uses position
projection and a zero-restitution, frictionless impulse using the full inverse mass.
Corner normals are resolved with bounded sequential iterations; failure to converge
faults the plant. This is a simple discrete contact approximation, not a general
mesh solver, continuous collision detector, or faithful hull/gripper contact model.
It does not create tangential friction or contact torques for the sphere. It is
appropriate to exercise lifecycle and simple boundaries, not validate manipulation.

Repeatability is within a supported numerical build/backend, not bitwise guarantees
across architectures. Mathematical reference tests cover conservation, dissipation,
current response, buoyancy, and actuator event timing. They do not prove calibration
of the synthetic AUV or parity with the complete Talos runtime.
