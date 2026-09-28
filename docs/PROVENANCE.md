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
Other project-owned files in this implementation were authored for this new project. The
example AUV is a new synthetic model, not a Talos calibration or copied mesh.
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
