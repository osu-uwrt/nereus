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
Other files in this implementation were authored for this new project. The
example AUV is a new synthetic model, not a Talos calibration or copied mesh.
No meshes, textures, ROS messages, or other robot assets are included.

Dependencies are installed separately, not vendored: Eigen (plant), yaml-cpp
(optional scenario loader), GoogleTest (tests), Python standard library (test and
development tools), and CMake/compiler tools. No runtime downloads occur. Review
licenses and exact packaged dependency versions before public distribution.
