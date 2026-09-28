# Independent original Talos dynamics comparison

The native Talos physics pack is checked against an independently constructed
original-source executable. It does not use the native importer, its emitted files,
or platform numerical code to calculate expected values. This verifies the assembled
vehicle and pool, beyond the earlier synthetic kernel/contact comparisons.

## Reference construction

The capture reads simulator revision `07647eebe706f96ea7b76db3cc9802735a146698`
and vehicle-description revision `7f37bdd62ab90113844137a23806c89b23a8ab9b`
from Git objects. It compiles the original marine and thruster dynamics and collision
box code, with extracted original state derivative, propulsion, RK4 and collision
response expressions. Logging and the separate optional task-contact hook are
excluded. Original YAML and URDF numbers supply the full mass/hydrodynamics, eight
ordered thrusters, COM mounts, hull/poker boxes, pool boxes and 2026 map placement.

The wrapper preserves the original half actuator step, pre-step contacts, raw RK4,
post-step contacts, quaternion normalization and final half actuator step. Full
source/driver/script/result hashes and compiler flags accompany each CSV.

```sh
python3 tools/capture_talos_reference.py /path/to/riptide_simulator /path/to/riptide_core
cmake --build --preset release
ctest --preset release -R TalosReference --output-on-failure
```

Capture needs the system C++ compiler, Eigen and yaml-cpp; it builds no ROS node.
Normal platform tests consume checked-in CSV fixtures and require neither original
repository. Regeneration is an explicit reference-maintenance operation.

## Acceptance and limits

Three cases run 1500 steps of 2 ms each: original startup (including initial pool
wall overlap), moving/rotating partial immersion, and a floor approach/contact.
The force schedule is the illustrative native Talos example schedule, not a recorded
controller or mission. Every tick, including initial states, is compared: 4503 rows
with 27 values each (position, quaternion, body velocities, eight realized thruster
forces, inertial COM acceleration in body axes and angular acceleration).

The maximum absolute difference must be at most `1e-9` for one entire reference
candidate across all three cases. Values are not selected independently by field,
tick or case. Tick and elapsed-time identity and finite numeric values are checked.
Both original GCC optimized (`-O2`) and unoptimized (`-O0`) captures are retained:
original contact branches can amplify floating-point differences, especially the
startup overlap case. Matching one whole candidate preserves a strict tolerance
without claiming cross-build bit identity. See also [BOX_CONTACTS.md](BOX_CONTACTS.md).

This does not validate long missions, moving props, task contacts, hardware-fitted
coefficients, sensor outputs, cameras, rendering, ROS interfaces or behavior trees.
Native Talos devices are still explicitly absent from this dynamics profile.
These bounded comparisons do not close the full simulator replacement gate.
