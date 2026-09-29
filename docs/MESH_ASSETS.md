# CPU mesh assets

`RoboticsPlatform::mesh_assets` loads owned CPU geometry and materials without
OpenGL, a window, simulation, ROS or a viewer. Assimp is a private implementation
dependency; the public C++17 API uses Eigen and standard value types. Geometry can
be loaded once and shared as `const MeshAsset` across future render instances.

```sh
# Additional optional system dependency: libassimp-dev (tested with Assimp 5.2.0).
cmake --preset assets
cmake --build --preset assets
python3 tools/check_assets.py
python3 tools/check_assets.py --preset assets-asan
```

`loadMesh(path, limits)` returns ordered submeshes, vertices, triangle indices,
base colors, optional external diffuse texture references, and indexed bounds.
Node transforms are baked into authored coordinates; normals use the inverse
transpose and normalization. Original depth-first submesh order, source units,
CAD origin, alpha and Assimp texture-coordinate conventions are preserved.
Collada up-axis conversion is disabled, as in the original loading path.
No robot-specific material repair, coordinate shift or animation occurs here.

The loader rejects missing/empty input, invalid geometry/materials, singular
transforms and configured size/count overflows. Limits bound input file size and
returned geometry; they do not bound Assimp's internal allocations or constitute
an untrusted-file sandbox. Loading does not retain importer-owned memory.

Textures are references only in this increment: embedded images are unsupported,
and external references must be relative. File existence, pack containment, image
decoding, color space and GPU lifetime belong to the upcoming resource/rendering
layer. Diffuse tint is retained even on textured materials; converted course assets
must explicitly select white where the original renderer forced white. The current
Talos body/rotors contain no textures and do not exercise that future path.

## Original Talos resource pack

`content/visuals/talos` contains the unchanged original `Talos3_body.glb` and eight
rotor GLBs, totaling 43,595,600 bytes. `inventory.json` records CAD coordinates,
mesh references, precise rotor pivots/axes/input order and force/RPM animation
settings without a transport topic. It is a resource inventory; runtime animation
and complete robot assembly are not yet instantiated. Original files and metadata
are retained under `provenance/` with hashes and source revision. That historical
metadata includes original workspace/topic references; it is not runtime config.

```sh
python3 tools/import_talos_visuals.py /path/to/riptide_simulator --check
```

`RP_BUILD_MESH_ASSETS` selects the library; `RP_INSTALL_REFERENCE_VISUALS` separately
selects the pack. Both default off. The `assets` preset enables them while disabling
simulation/CLI. The pack installs at `share/robotics_platform/visuals/talos`.
Default headless installs, Python wheels and Python source archives exclude these
optional visuals. No runtime downloads or original checkout lookup occurs.

The body excludes launcher, payloads, claw and robot magnet. LEDs also remain
separate scene elements. Those, moving rotors, the original pool/course and full
scene/shadow/water/reflection/bloom/post renderer are required next. This CPU
increment does not establish rendered appearance, original UI or camera parity.

## Validation

A separate offline capture compiles the unchanged original `Renderer::load` method
and matrix conversion with the original vendored GLM. A CPU `Mesh` wrapper records
results without OpenGL. It reads pinned original Git objects directly, independently
of the new importer, assets and loader. Source/driver/capture/result hashes and
compiler/Assimp versions accompany `tests/fixtures/legacy_mesh_assets.csv`.

```sh
python3 tools/capture_mesh_reference.py /path/to/riptide_simulator
```

Tests compare all 37 original submeshes: order, vertex/index counts, index hashes,
RGBA, vertex bounds and position/normal/UV means within 1e-6. The complete body has
1,053,293 triangles; the eight rotors add 33,937. A separate authored nested-transform
fixture verifies nonuniform scaling, inverse-transpose normals, hierarchy order,
transparency and UV conventions. File/count failure paths are checked too.
These comparisons are CPU geometry/material evidence, not rendered-image evidence.

`check_assets.py` verifies every imported resource hash, builds a source copy with
no simulation/viewer/GL sources, relocates its install, removes that source/build
copy, and runs an external C++ consumer against all nine meshes. It checks exported
targets for unwanted graphics/simulation dependencies. The original simulator is
not needed by tests, ordinary builds or installed consumers.
