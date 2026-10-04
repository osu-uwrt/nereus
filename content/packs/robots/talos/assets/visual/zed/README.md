# ZED X Mini mesh

`zedxm.glb` is Stereolabs' `meshes/zedxm.stl` from
[zed-ros2-description](https://github.com/stereolabs/zed-ros2-description) at commit
`449eef9566a49461cc37d6ac13230fe2886ff2be` (Apache-2.0, see `LICENSE`), converted to glTF with trimesh. The single STL is
split into four parts colored after Stereolabs' product photos: black anodized body, smoked front glass
(front-facing faces inside the bezel, x 0.0135-0.0153 m), lenses (r < 10.8 mm around y = +-25 mm, z = 15.9 mm)
and the silver GMSL2 connector (behind x = -0.0215 m). Vertices merged, normals split at 30 degree creases,
placeholder UVs (trimesh drops the material without them). The geometry is unchanged: meters, origin at the bottom
mounting screw (zed_wrapper's `<camera>_camera_link`), +X forward, +Z up.
