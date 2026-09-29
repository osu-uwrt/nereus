# Source and dependency provenance

The numerical kernels in `libraries/simulation/src/marine_dynamics.cpp`,
`thruster_dynamics.cpp`, their private `detail/` headers, and
`libraries/simulation/tests/model_reference_test.cpp` are adapted from:

- Project: OSU UWRT `riptide_simulator`
- Repository: https://github.com/osu-uwrt/riptide_simulator
- Revision: `93dfdb42e9280943677631b9fbef5afa034ec1b6`
- Original files: `c_simulator/src/{marine_dynamics,thruster_dynamics}.cpp`,
  `c_simulator/include/c_simulator/{MarineDynamics,ThrusterDynamics}.h`,
  `c_simulator/test/test_marine_dynamics.cpp`.
- Changes: private namespace/includes, formatting, empty-thruster support,
  checked index conversion and consistent Eigen index types.
- License: not established in that repository; see ../LICENSE.md. Public release
  is blocked on resolving this metadata, not on local implementation work.

The architecture plan was developed with the project owner in the original
simulator's `docs/REUSABILITY_PLAN.md` and copied here as the working design.
Other project-owned implementation files were authored for this new project. The
synthetic example AUV is not a Talos calibration or copied mesh. Native Talos/pool
profile data is separately attributed below.
No meshes, textures, ROS messages, or other robot assets are included.

Except for Dear ImGui described below, dependencies are installed separately: Eigen (plant), yaml-cpp
(optional scenario loader), GoogleTest (tests), Python standard library (test and
development tools), and CMake/compiler tools. No runtime downloads occur. Review
licenses and exact packaged dependency versions before public distribution.

## Viewer dependencies and content

`third_party/imgui` contains the upstream Dear ImGui v1.91.9b core and GLFW/OpenGL3
backends, copied from the identified unmodified distribution in
`riptide_simulator/camera_faker/vendor/imgui`. Upstream:
https://github.com/ocornut/imgui/tree/v1.91.9b. The MIT license and the bundled stb
notices are retained; the ImGui license is also installed with the viewer.
The viewer UI, neutral contracts, line renderer, and local motion fixture were
authored for this project; no legacy host, shaders, meshes, or ROS code was copied.

The viewer links system GLFW, GLEW, and OpenGL in addition to Eigen/yaml-cpp.
It optionally loads a system DejaVu font, falling back to ImGui's default; no font
file is redistributed separately. Linux system library packaging supplies those
third-party notices. Inventory dependencies again before any public distribution.

## Stage-dependent physical reference

The immersed propeller/current expressions in `libraries/simulation/src/plant.cpp`
and RK4 sequencing in `detail/rk4.hpp` are adapted from original revision
`07647eebe706f96ea7b76db3cc9802735a146698`, specifically
`c_simulator/src/robot_class.cpp` and `c_simulator/src/physics_simulator.cpp`.
They retain the legacy disk-area fraction, 1 mm minimum projected extent, sinusoidal
current derivative, stage forcing and midpoint actuator split. New configuration
is neutral, immersion is optional, and environment time uses authoritative ticks.
The original unresolved license metadata described above applies to these ports too.

`tools/capture_stage_reference.py` reads that pinned revision only when explicitly
invoked. Its test driver extracts original expressions into a temporary build with
the original pure numerical kernels, then records checked fixture data. The driver
and all inputs/results are hashed in `tests/fixtures/legacy_stage_dynamics.json`.
See `reference/STAGE_DYNAMICS.md` for reproduction and the bounded comparison scope.

## Static box contact equations

`libraries/simulation/src/box_contacts.cpp` adapts SAT, contact-point selection and
impulse/friction response from original revision
`07647eebe706f96ea7b76db3cc9802735a146698`:
`c_simulator/src/{collisionBox_class,physics_simulator}.cpp` and
`c_simulator/include/c_simulator/collisionBox.h`. Geometry validation, neutral
value descriptors, fixed-size temporaries, ownership and caching were rewritten.
Original proxy ordering and response expressions remain reference-tested.
The original license metadata remains unresolved as described above.

The offline contact capture extracts original method bodies, disables logging,
and excludes the separate optional task-contact hook. It does not build a ROS node.
Source/driver/fixture hashes and compiler flags accompany the single-response and
whole-step optimized/unoptimized captures in `tests/fixtures/legacy_box_*.json`.
See `reference/BOX_CONTACTS.md` for scope, optimization sensitivity and reproduction.

## Native Talos and pool data

`content/robots/talos_dynamics.yaml`, `content/worlds/competition_pool.yaml`, and
`content/examples/talos_pool.yaml` were converted from pinned simulator and
riptide_core vehicle-description data. The offline importer strips transport and
controller configuration, resolves CAD/COM offsets, and emits native physical
parameters, rigid mounts and collision boxes. The example command schedule is new;
sensors/visuals/tasks are not instantiated by this slice. Exact input revisions and
source/output/importer hashes are in `reference/talos_physics_sources.json`.
See `reference/TALOS_PHYSICS_PACK.md` for reproduction and limits. These project
sources retain the unresolved source/asset license review requirement before public
redistribution; local implementation continues under the user's authorization.

## Independent Talos trajectory reference

`tools/capture_talos_reference.py` and `tools/reference/talos_driver.cpp` build the
pinned original pure numerical kernels with extracted stage forcing, RK4 and
static-box collision methods. Physical inputs come independently from original
vehicle/hydrodynamics/world/mapping/URDF Git objects; the native importer and its
outputs are not reference inputs. Source, driver, capture script and fixture hashes
are recorded in `tests/fixtures/legacy_talos{,_unoptimized}.json`, alongside compiler
identity and flags. The fixtures contain every tick of three scripted trajectories.
See [TALOS_DYNAMICS.md](reference/TALOS_DYNAMICS.md) for scope and reproduction.
Original source license status remains as described above.

The Talos importer also emits a separate inertial robot/example and reusable AHRS
profile from the same pinned physical data plus original vehicle/simulator sensor
settings. Original FOG rate/noise/covariance defaults live in node construction;
the importer verifies their exact source expressions before emitting neutral
configuration. All additional source/output hashes remain in
`reference/talos_physics_sources.json`. No original ROS node is copied into the
platform or invoked at runtime.
