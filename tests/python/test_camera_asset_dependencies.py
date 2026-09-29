"""Importer sidecars are part of a pack camera's verified visual input."""

import hashlib
import json
import struct
import tempfile
import unittest
from pathlib import Path

from robotics_platform.packs import resolve_scenario
from test_pack_cameras import generic_packs, pc


@unittest.skipIf(pc is None, "optional camera extension is not installed")
class CameraAssetDependencyTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def material_pack(self, declared=True, reference="hull.mtl"):
        scenario = generic_packs(self.root)
        mesh = self.root / "robot/assets/hull.obj"
        old = mesh.read_bytes()
        new = f"mtllib {reference}\nusemtl custom\n".encode() + old
        mesh.write_bytes(new)
        material = mesh.parent / reference
        material.write_text("newmtl custom\nKd 1 0 0\n", encoding="utf-8")
        robot = self.root / "robot/robot.yaml"
        text = robot.read_text().replace(hashlib.sha256(old).hexdigest(),
                                        hashlib.sha256(new).hexdigest())
        if declared:
            entry = ("- {id: hull_material, path: assets/hull.mtl, source: modelled for tests, "
                     "required_from_step: 3, status: present, sha256: "
                     + hashlib.sha256(material.read_bytes()).hexdigest() + "}\n")
            text = text.replace("assets:\n", "assets:\n" + entry)
        robot.write_text(text, encoding="utf-8")
        return resolve_scenario(scenario), mesh, material

    def test_obj_sidecar_is_owned_readonly_metadata_and_recorded(self):
        resolved, mesh_path, material = self.material_pack()
        mesh = pc._camera.load_mesh(mesh_path)
        expected = sorted([mesh_path.resolve(), material.resolve()])
        self.assertEqual(mesh.dependencies, expected)
        detached = mesh.dependencies
        detached.clear()
        self.assertEqual(mesh.dependencies, expected)
        with self.assertRaises(AttributeError):
            mesh.dependencies = []
        record = pc.PackCameras(resolved).describe()["scene"]["files"]
        self.assertIn({"pack": "robot", "id": "hull_material",
                       "kind": "importer_dependency", "path": "assets/hull.mtl",
                       "sha256": hashlib.sha256(material.read_bytes()).hexdigest(),
                       "used_by": ["hull_mesh"]}, record)

    def test_bin_vinyl_texture_override_is_verified_and_recorded(self):
        content = Path(__file__).resolve().parents[2] / "content/packs/scenarios/talos_uwrt"
        record = pc.PackCameras(resolve_scenario(content)).describe()["scene"]["files"]
        used = {item["id"] for item in record if item["kind"] == "texture"
                and "bin_vinyl_mesh" in item["used_by"]}
        self.assertEqual(used, {"bin_vinyl_blood_texture", "bin_vinyl_fire_texture"})

    def test_undeclared_material_is_rejected(self):
        resolved, _, _ = self.material_pack(declared=False)
        with self.assertRaisesRegex(ValueError, "importer_dependency .* not a declared"):
            pc.PackCameras(resolved)

    def test_changed_material_is_rejected_after_resolution(self):
        resolved, _, material = self.material_pack()
        material.write_text("newmtl custom\nKd 0 0 1\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "importer_dependency .* sha256 mismatch"):
            pc.PackCameras(resolved)

    def test_escaping_material_is_rejected(self):
        resolved, _, _ = self.material_pack(declared=False, reference="../../outside.mtl")
        with self.assertRaisesRegex(ValueError, "importer_dependency .* escapes the pack"):
            pc.PackCameras(resolved)

    def test_deleted_declared_material_cannot_silently_change_the_scene(self):
        resolved, _, material = self.material_pack()
        material.unlink()
        with self.assertRaisesRegex(ValueError, "sources changed.*missing file"):
            pc.PackCameras(resolved)

    def test_gltf_external_buffer_is_recorded(self):
        buffer = self.root / "triangle.bin"
        buffer.write_bytes(struct.pack("<9f3H", 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 2))
        mesh = self.root / "triangle.gltf"
        mesh.write_text(json.dumps({
            "asset": {"version": "2.0"}, "scene": 0,
            "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
            "buffers": [{"uri": buffer.name, "byteLength": 42}],
            "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 36},
                            {"buffer": 0, "byteOffset": 36, "byteLength": 6}],
            "accessors": [{"bufferView": 0, "componentType": 5126, "count": 3,
                           "type": "VEC3", "min": [0, 0, 0], "max": [1, 1, 0]},
                          {"bufferView": 1, "componentType": 5123, "count": 3,
                           "type": "SCALAR"}],
        }), encoding="utf-8")
        loaded = pc._camera.load_mesh(mesh)
        self.assertEqual(loaded.dependencies, sorted([buffer.resolve(), mesh.resolve()]))
        # Perforation produces a new mesh without losing its provenance.
        import numpy as np

        perforated, _ = pc._camera.perforate_mesh(
            loaded, np.eye(4, dtype=np.float32), [0.0], 2.0, [(0.5, 0.5, 0.1)])
        self.assertEqual(perforated.dependencies, loaded.dependencies)


if __name__ == "__main__":
    unittest.main()
