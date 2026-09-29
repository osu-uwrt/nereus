"""Pack-driven offscreen camera capture: real Talos 2026 content and a generic pack set."""

import copy
import dataclasses
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
import zlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from robotics_platform.packs import resolve_scenario
from test_camera_pack_fields import APPEARANCE, ASSET, CAMERA, CAMERA_FRAMES, MESH, mutate
from test_packs_fixtures import write_generic_packs

try:
    from robotics_platform import pack_cameras as pc
except ImportError:
    if os.environ.get("RP_REQUIRE_CAMERA"):
        raise
    pc = None

CONTENT = Path(__file__).resolve().parents[2] / "content" / "packs"
SCENARIO = CONTENT / "scenarios" / "talos_uwrt" / "scenario.yaml"
HALF = 0.3048  # torpedo panel half size (tasks/robosub_2026/torpedo.yaml)
IDENTITY = np.array([1.0, 0.0, 0.0, 0.0])


def quaternion(rotation):
    """Unit wxyz from a rotation matrix (columns are the rotated axes)."""
    m = np.asarray(rotation, float)
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# Optical axes (x right, y down, z forward) for looking along -X / +X of a task frame.
FACING_MINUS_X = quaternion(np.column_stack([[0, 1, 0], [0, 0, -1], [-1, 0, 0]]))
FACING_PLUS_X = quaternion(np.column_stack([[0, -1, 0], [0, 0, -1], [1, 0, 0]]))


def with_changes(resolved, robot=None, scenario=None, task_definitions=None, tasks=None):
    return dataclasses.replace(
        resolved, robot=robot or resolved.robot, scenario=scenario or resolved.scenario,
        task_definitions=task_definitions or resolved.task_definitions,
        tasks=tasks or resolved.tasks)


def root_for_eye(cameras, frame, world_eye):
    """Robot root (position, wxyz) that puts the named eye frame at ``world_eye``."""
    root = world_eye.compose(cameras._frames.from_root(frame).inverse())
    return root.translation, root.orientation_wxyz


def centre_depth(frame, info):
    return float(frame.depth[round(info["k"][5]), round(info["k"][2])])


@unittest.skipIf(pc is None, "optional camera extension is not installed")
class TalosPackCameraTests(unittest.TestCase):
    """Production camera packs and isolated copies for configuration/error cases."""

    @classmethod
    def setUpClass(cls):
        cls.resolved = resolve_scenario(SCENARIO)

    def enabled(self, *, noise=False, outputs=None, visuals=True, torpedo=None, sensors=None):
        robot = copy.deepcopy(self.resolved.robot)
        for sensor in robot["sensors"]:
            if sensor["type"] == "stereo_camera":
                sensor["enabled"] = True
                if outputs is not None:
                    sensor["parameters"]["outputs"] = list(outputs)
        if not visuals:
            robot["visuals"] = []
        scenario = dict(self.resolved.scenario, sensor_noise=noise)
        tasks = copy.deepcopy(self.resolved.task_definitions)
        if torpedo is not None:
            torpedo(next(item for item in tasks if item["id"] == "torpedo")["props"][0])
        return pc.PackCameras(with_changes(self.resolved, robot, scenario, tasks), sensors)

    def test_concurrent_cameras_preserve_stereo_pixels_jpeg_and_noise_replay(self):
        robot = copy.deepcopy(self.resolved.robot)
        for sensor in robot["sensors"]:
            if sensor["type"] != "stereo_camera":
                continue
            sensor["enabled"] = True
            p = sensor["parameters"]
            # Small frames keep this synchronization/replay test independent of GPU throughput.
            sx, sy = 160 / p["resolution_px"][0], 100 / p["resolution_px"][1]
            p["resolution_px"] = [160, 100]
            p["outputs"] = ["rgb_left", "rgb_right", "depth_left"]
            for eye in ("left", "right"):
                for key, scale in (("fx", sx), ("cx", sx), ("fy", sy), ("cy", sy)):
                    p[f"intrinsics_{eye}"][key] *= scale
        cameras = pc.PackCameras(with_changes(
            self.resolved, robot, dict(self.resolved.scenario, sensor_noise=True)))

        def sequence(sensor):
            position = [10, 10, -1] if sensor == cameras.sensor_ids[0] else [11, 12, -1.2]
            return [cameras.capture(sensor, position, IDENTITY, i / 15,
                                    jpeg_quality=93) for i in range(3)]

        expected = {sensor: sequence(sensor) for sensor in cameras.sensor_ids}
        with ThreadPoolExecutor(2) as workers:
            for _ in range(2):
                cameras.reset(self.resolved.scenario["seed"])
                actual = dict(zip(cameras.sensor_ids, workers.map(sequence, cameras.sensor_ids)))
                for sensor in cameras.sensor_ids:
                    for a, b in zip(expected[sensor], actual[sensor]):
                        for eye in ("left", "right"):
                            np.testing.assert_array_equal(getattr(a, eye).rgb, getattr(b, eye).rgb)
                            self.assertEqual(getattr(a, eye).jpeg, getattr(b, eye).jpeg)
                        np.testing.assert_array_equal(a.left.depth, b.left.depth)

    def torpedo_view(self, cameras, uv, distance, back=False):
        placement = next(item for item in self.resolved.scenario["task_placements"]
                         if item["task"] == "torpedo")
        target = np.array([0.0, (uv[0] - 0.5) * 2 * HALF, (uv[1] - 0.5) * 2 * HALF])
        eye = pc.pose(target + [-distance if back else distance, 0, 0],
                      FACING_PLUS_X if back else FACING_MINUS_X)
        world_eye = pc._upright(placement).compose(eye)
        return root_for_eye(cameras, "ffc_left_optical", world_eye)

    def test_disabled_cameras_and_explicit_selection_are_validated(self):
        robot = copy.deepcopy(self.resolved.robot)
        for sensor in robot["sensors"]:
            if sensor["type"] == "stereo_camera":
                sensor["enabled"] = False
        disabled = with_changes(self.resolved, robot)
        with self.assertRaisesRegex(ValueError, "no enabled"):
            pc.PackCameras(disabled)
        with self.assertRaisesRegex(ValueError, "disabled"):
            pc.PackCameras(disabled, ["ffc"])
        for sensors, error in ((["dvl"], ValueError), (["ghost"], ValueError), ("ffc", TypeError)):
            with self.subTest(sensors=sensors), self.assertRaises(error):
                pc.PackCameras(self.resolved, sensors)

    def test_module_needs_only_the_camera_extension(self):
        subprocess.run([sys.executable, "-c", "\n".join([
            "import sys",
            "import robotics_platform.pack_cameras",
            "loaded = [k for k in sys.modules if k.startswith(('robotics_platform._native',"
            " 'robotics_platform.packs', 'rclpy'))]",
            "assert not loaded, loaded",
        ])], check=True)

    def test_default_bringup_look_pool_and_only_referenced_assets(self):
        cameras = self.enabled()
        record = cameras.describe()
        json.dumps(record, allow_nan=False)
        expected = {
            "water_tint_rgb": [0.025, 0.22, 0.29], "water_absorption_per_m_rgb": [0.648, 0.145, 0.025],
            "water_scattering": 0.458, "water_distance_scale": 1.88, "water_distance_power": 0.40,
            "water_clear_distance_m": 0.0, "profile": "outdoor", "direct_light": 1.0,
            "ambient_light": 0.8, "sun_azimuth": 225.0, "sun_elevation": 55.0, "glare": 0.5,
            "caustics": 1.0, "exposure": 1.0, "surface": True, "shadows": True, "reflections": True}
        self.assertEqual(set(record["appearance"]), set(expected))
        for key, value in expected.items():
            with self.subTest(key=key):
                np.testing.assert_allclose(record["appearance"][key], value, rtol=1e-6) \
                    if not isinstance(value, (bool, str)) else \
                    self.assertEqual(record["appearance"][key], value)
        self.assertEqual(record["scene"]["pool"]["dimensions_m"], [50.0, 22.86, 2.1336])
        self.assertEqual(record["scene"]["robot_visuals"], 14)
        files = {kind: {(item["pack"], item["id"]) for item in record["scene"]["files"]
                        if item["kind"] == kind} for kind in ("mesh", "texture")}
        visuals = {("robot", item["asset"]) for item in self.resolved.robot["visuals"]}
        self.assertEqual(files["mesh"], visuals | {("tasks", name) for name in (
            "gate_mesh", "gate_repair_mesh", "gate_rescue_mesh", "torpedo_mesh", "slalom_mesh",
            "bin_mesh", "bin_vinyl_mesh", "bin_magnet_mesh", "octagon_buoy_mesh",
            "octagon_compass_mesh", "octagon_hammer_and_wrench_mesh", "octagon_sos_mesh",
            "table_visual", "basket_helmet_visual", "basket_warning_visual")})
        self.assertEqual(files["texture"], {("tasks", name) for name in (
            "gate_repair_texture", "gate_rescue_texture", "torpedo_texture",
            "octagon_buoy_texture", "octagon_sos_texture", "octagon_hammer_and_wrench_texture",
            "octagon_compass_texture", "bin_vinyl_fire_texture", "bin_vinyl_blood_texture",
            "basket_helmet_visual_task5_redcross_fixed",
            "basket_warning_visual_task5_warning_fixed")})
        torpedo = next(item for item in record["scene"]["files"]
                       if item["id"] == "torpedo_texture")
        self.assertEqual(torpedo["used_by"], ["torpedo_mesh"])
        declared = {item["id"]: item["sha256"] for item in self.resolved.tasks["assets"]}
        self.assertEqual(torpedo["sha256"], declared["torpedo_texture"])
        # Detached: editing the returned record changes nothing inside the capture object.
        record["scene"]["cutouts"][0]["face_triangles"].append(99)
        record["scene"]["files"][0]["used_by"].append("x")
        record["appearance"]["water_tint_rgb"][0] = 5
        record["cameras"]["ffc"]["seeds"]["left"] = 0
        self.assertEqual(cameras.describe()["scene"]["cutouts"][0]["face_triangles"], [2, 2])
        self.assertNotIn("x", cameras.describe()["scene"]["files"][0]["used_by"])
        self.assertAlmostEqual(cameras.describe()["appearance"]["water_tint_rgb"][0], 0.025)
        self.assertNotEqual(cameras.describe()["cameras"]["ffc"]["seeds"]["left"], 0)
        self.assertEqual(cameras.describe()["scene"]["cutouts"], [{
            "task": "torpedo", "prop": "board", "region": "panel",
            "faces_local_x_m": [0.0, -0.004], "face_triangles": [2, 2]}])
        self.assertEqual(record["clipping_m"], {"near": 0.05, "far": 100.0})

    def test_torpedo_front_and_backing_openings_are_cut_from_both_sides(self):
        hole = (0.16142625, 0.58890478)  # fire_large
        solid = (0.5, 0.5)
        distance = 1.5
        configurations = {
            "both": None,
            "front_only": lambda prop: prop["parameters"]["cutouts"].update(
                faces_local_x_m=[0.0]),
            "none": lambda prop: prop["parameters"].pop("cutouts"),
        }
        depths = {}
        for name, change in configurations.items():
            cameras = self.enabled(torpedo=change, sensors=["ffc"])
            info = cameras.info("ffc")
            for side in ("front", "back"):
                for label, uv in (("hole", hole), ("solid", solid)):
                    root = self.torpedo_view(cameras, uv, distance, back=side == "back")
                    frame = cameras.capture("ffc", *root, 0.0).left
                    depths[name, side, label] = centre_depth(frame, info)
        for side, first in (("front", distance), ("back", distance - 0.004)):
            with self.subTest(side=side):
                self.assertAlmostEqual(depths["both", side, "solid"], first, delta=0.002)
                through = depths["both", side, "hole"]
                self.assertTrue(math.isnan(through) or through > distance + 0.05, through)
                self.assertAlmostEqual(depths["none", side, "hole"], first, delta=0.002)
        # Only the front face cut: the backing sheet 4 mm behind remains visible.
        self.assertAlmostEqual(depths["front_only", "front", "hole"], distance + 0.004,
                               delta=0.001)
        self.assertAlmostEqual(depths["front_only", "back", "hole"], distance - 0.004,
                               delta=0.002)

    def test_right_eye_is_rendered_at_the_rectified_baseline_and_info_matches(self):
        cameras = self.enabled(outputs=["rgb_left", "depth_left", "camera_info", "rgb_right"],
                               visuals=False, sensors=["ffc"])
        root = self.torpedo_view(cameras, (0.5, 0.5), 2.0)
        pair = cameras.capture("ffc", *root, 0.0)
        self.assertEqual(pair.right.depth.size, 0)
        world_right = pc.pose(*root).compose(cameras._frames.from_root("ffc_right_optical"))
        shifted = root_for_eye(cameras, "ffc_left_optical", world_right)
        moved = cameras.capture("ffc", *shifted, 0.0).left
        differing = np.any(pair.right.rgb != moved.rgb, axis=2).mean()
        self.assertLess(differing, 1e-3)
        self.assertGreater(np.any(pair.right.rgb != pair.left.rgb, axis=2).mean(), 0.01)
        left, right = cameras.info("ffc"), cameras.info("ffc", "right")
        baseline = next(item for item in self.resolved.robot["sensors"]
                        if item["id"] == "ffc")["parameters"]["baseline_m"]
        self.assertEqual(left["k"], right["k"])
        self.assertEqual(left["p"][3], 0.0)
        self.assertAlmostEqual(right["p"][3], -right["k"][0] * baseline)
        self.assertEqual((left["width"], left["height"]), (1920, 1200))

    def test_seeded_noise_replays_and_failed_calls_consume_nothing(self):
        cameras = self.enabled(noise=True, sensors=["ffc"])
        root = self.torpedo_view(cameras, (0.5, 0.5), 1.5)
        first = cameras.capture("ffc", *root, 0.0).left.depth.copy()
        second = cameras.capture("ffc", *root, 0.0).left.depth
        self.assertFalse(np.array_equal(first, second, equal_nan=True))
        cameras.reset(7)
        for arguments, options in (((root[0], [1, 0, 0, 0.1], 0.0), {}),
                                   ((root[0], root[1], math.nan), {}),
                                   ((root[0], root[1], 0.0), {"jpeg_quality": 101}),
                                   ((root[0], root[1], 0.0), {"jpeg_quality": True})):
            with self.subTest(options=options), self.assertRaises(ValueError):
                cameras.capture("ffc", *arguments, **options)
        with self.assertRaises(ValueError):
            cameras.capture("dfc", *root, 0.0)
        np.testing.assert_array_equal(cameras.capture("ffc", *root, 0.0).left.depth, first)
        fresh = self.enabled(noise=True, sensors=["ffc"])
        np.testing.assert_array_equal(fresh.capture("ffc", *root, 0.0).left.depth, first)
        for seed in (-1, 1 << 64, True, 1.0):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                cameras.reset(seed)

    def test_seed_derivation_is_stable_and_independent_of_selection_order(self):
        self.assertEqual(pc.derive_seed(7, "ffc", "left"), 3893539410)
        self.assertEqual(pc.derive_seed(7, "ffc", "right"), 1429836168)
        self.assertEqual(pc.derive_seed(7, "dfc", "left"), 3363439695)
        a = self.enabled(sensors=["ffc", "dfc"]).describe()
        b = self.enabled(sensors=["dfc", "ffc"]).describe()
        for sensor in ("ffc", "dfc"):
            self.assertEqual(a["cameras"][sensor]["seeds"], b["cameras"][sensor]["seeds"])
        self.assertEqual(a["seed"]["scenario_seed"], 7)
        self.assertIn("sha256", a["seed"]["policy"])

    def test_description_waits_for_a_complete_seed_reset(self):
        cameras = self.enabled(outputs=["camera_info"])
        entered, release, describing = threading.Event(), threading.Event(), threading.Event()
        processors = cameras._cameras["ffc"]["processors"]
        original = processors["left"]

        def reset(seed):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("test reset was not released")
            original.reset(seed)

        def describe():
            describing.set()
            return cameras.describe()

        processors["left"] = SimpleNamespace(reset=reset)
        with ThreadPoolExecutor(2) as workers:
            restart = workers.submit(cameras.reset, 99)
            try:
                self.assertTrue(entered.wait(2))
                description = workers.submit(describe)
                self.assertTrue(describing.wait(2))
                self.assertFalse(description.done())
            finally:
                release.set()
            restart.result(timeout=2)
            record = description.result(timeout=2)
        self.assertEqual(record["seed"]["scenario_seed"], 99)
        for sensor in cameras.sensor_ids:
            for eye in pc.EYES:
                self.assertEqual(record["cameras"][sensor]["seeds"][eye],
                                 pc.derive_seed(99, sensor, eye))

    def test_outputs_and_jpeg_follow_the_pack_request(self):
        cameras = self.enabled(outputs=["depth_left", "camera_info"], sensors=["dfc"])
        frame = cameras.capture("dfc", [10, 10, -1], IDENTITY, 0.0, jpeg_quality=90).left
        self.assertEqual((frame.rgb.size, frame.jpeg), (0, b""))
        self.assertEqual(frame.depth.shape, (1200, 1920))
        info_only = self.enabled(outputs=["camera_info"], sensors=["dfc"])
        self.assertEqual(info_only.capture("dfc", [10, 10, -1], IDENTITY, 0.0),
                         pc.CameraCapture("dfc", None, None))
        cameras = self.enabled(sensors=["ffc"])
        plain = cameras.capture("ffc", [10, 10, -1], IDENTITY, 0.0).left
        encoded = cameras.capture("ffc", [10, 10, -1], IDENTITY, 0.0, jpeg_quality=80).left
        self.assertEqual(plain.jpeg, b"")
        self.assertTrue(encoded.jpeg.startswith(b"\xff\xd8"))
        self.assertIsNone(cameras.capture("ffc", [10, 10, -1], IDENTITY, 0.0).right)

    def test_robot_visuals_follow_the_supplied_root_pose(self):
        # Neither Talos camera sees its own hull or claw (the claw is ~59 deg off the DFC axis);
        # the robot still shades the pool floor below the DFC at this pose.
        with_robot = self.enabled(sensors=["dfc"])
        without = self.enabled(sensors=["dfc"], visuals=False)
        shaded = with_robot.capture("dfc", [10, 10, -1], IDENTITY, 0.0).left.rgb
        plain = without.capture("dfc", [10, 10, -1], IDENTITY, 0.0).left.rgb
        self.assertGreater(np.any(shaded != plain, axis=2).mean(), 0.01)
        # com -> cad -> claw_pinch and com -> cad (robot.yaml frames), identity orientations.
        expected = {"body_mesh": [0.157, -0.04, 0.048],
                    "claw_static_mesh": [0.157 + 0.1845620038521386, -0.04 + 0.034362,
                                         0.048 - 0.4164364383477267]}
        visuals = [item["asset"] for item in self.resolved.robot["visuals"]]
        for name, position in expected.items():
            matrix = with_robot._robot_visuals[visuals.index(name)][1]
            np.testing.assert_allclose(matrix[:3, 3], position, atol=1e-12)
            np.testing.assert_allclose(matrix[:3, :3], np.eye(3), atol=1e-12)

    def test_texture_dependencies_are_declared_present_and_unchanged(self):
        cases = {
            "sha256 mismatch": lambda assets: next(
                item for item in assets if item["id"] == "torpedo_texture").update(sha256="0" * 64),
            "is declared missing": lambda assets: next(
                item for item in assets if item["id"] == "torpedo_texture").update(
                    status="missing"),
            "is not a declared pack asset": lambda assets: assets.remove(next(
                item for item in assets if item["id"] == "torpedo_texture")),
        }
        robot = copy.deepcopy(self.resolved.robot)
        for sensor in robot["sensors"]:
            sensor["enabled"] = sensor["type"] == "stereo_camera" or sensor.get("enabled", True)
        for message, change in cases.items():
            tasks = copy.deepcopy(self.resolved.tasks)
            change(tasks["assets"])
            with self.subTest(message=message), \
                    self.assertRaisesRegex(ValueError, f"texture 'Task4_ver1_Fixed.png' used by "
                                           f"asset 'torpedo_mesh' {message}"):
                pc.PackCameras(with_changes(self.resolved, robot, tasks=tasks))

    def test_poses_use_native_validation(self):
        cameras = self.enabled(sensors=["ffc"])
        with self.assertRaisesRegex(ValueError, "unit quaternion"):
            cameras.capture("ffc", [10, 10, -1], [1 + 1e-7, 0, 0, 0], 0.0)
        with self.assertRaisesRegex(ValueError, "unit quaternion"):
            cameras.capture("ffc", [10, math.inf, -1], IDENTITY, 0.0)
        self.assertIsNotNone(cameras.capture("ffc", [10, 10, -1], [1 + 1e-9, 0, 0, 0], 0.0).left)
        with self.assertRaises(TypeError):
            cameras.capture("ffc", [10, 10], IDENTITY, 0.0)

    def test_referenced_assets_are_verified_before_rendering(self):
        cases = {
            "declared missing": lambda asset: asset.update(status="missing"),
            "sha256 mismatch": lambda asset: asset.update(sha256="0" * 64),
        }
        for message, change in cases.items():
            robot = copy.deepcopy(self.resolved.robot)
            for sensor in robot["sensors"]:
                sensor["enabled"] = sensor["type"] == "stereo_camera" or sensor.get("enabled", True)
            change(next(item for item in robot["assets"] if item["id"] == "claw_static_mesh"))
            with self.subTest(message=message), self.assertRaisesRegex(ValueError, message):
                pc.PackCameras(with_changes(self.resolved, robot))
        robot = copy.deepcopy(robot)
        robot["assets"] = [item for item in robot["assets"] if item["id"] != "claw_static_mesh"]
        with self.assertRaisesRegex(ValueError, "unknown asset 'claw_static_mesh'"):
            pc.PackCameras(with_changes(self.resolved, robot))


PANEL_DAE = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="print"><init_from>{texture}</init_from></image></library_images>
<library_effects><effect id="print-effect"><profile_COMMON>
<newparam sid="surface"><surface type="2D"><init_from>print</init_from></surface></newparam>
<newparam sid="sampler"><sampler2D><source>surface</source></sampler2D></newparam>
<technique sid="common"><lambert><diffuse><texture texture="sampler" texcoord="UVMap"/></diffuse>
</lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="print-material"><instance_effect url="#print-effect"/></material>
</library_materials>
<library_geometries><geometry id="panel"><mesh>
<source id="p"><float_array id="pa" count="12">0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5</float_array>
<technique_common><accessor source="#pa" count="4" stride="3"><param name="X" type="float"/>
<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<source id="n"><float_array id="na" count="3">1 0 0</float_array><technique_common>
<accessor source="#na" count="1" stride="3"><param name="X" type="float"/><param name="Y" type="float"/>
<param name="Z" type="float"/></accessor></technique_common></source>
<source id="t"><float_array id="ta" count="8">0 0 1 0 1 1 0 1</float_array><technique_common>
<accessor source="#ta" count="4" stride="2"><param name="S" type="float"/><param name="T" type="float"/>
</accessor></technique_common></source>
<vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
<triangles material="print-material" count="2"><input semantic="VERTEX" source="#v" offset="0"/>
<input semantic="NORMAL" source="#n" offset="1"/><input semantic="TEXCOORD" source="#t" offset="2" set="0"/>
<p>0 0 0 1 0 1 2 0 2 0 0 0 2 0 2 3 0 3</p></triangles></mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="panel"><instance_geometry url="#panel">
<bind_material><technique_common><instance_material symbol="print-material" target="#print-material">
<bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/></instance_material>
</technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
PANEL_TASK = """\
schema_version: 1
kind: task
id: hoop
frames: []
props:
- id: ring
  type: static_body
  parameters:
    collision_boxes:
    - {id: sheet, size_m: [0.02, 1.0, 1.0], center_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
    visuals:
    - {asset: hoop_mesh, frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
    cutouts: {region: sheet, faces_local_x_m: [0.0]}
regions:
- id: sheet
  type: perforated_panel
  parameters:
    plane: {axis: x, offset_m: 0.0}
    half_size_m: 0.5
    holes:
    - {id: middle, class: target, size: large, uv: [0.5, 0.5], radius_uv: 0.1}
    projectile_clearance: {rule: radius_over_axis_cosine, min_cosine: 0.1}
events: []
scoring: []
"""


def png(width, height, rgba):
    """Minimal RGBA8 PNG with a uniform colour."""
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))
    rows = b"".join(b"\0" + bytes(rgba) * width for _ in range(height))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 6,
                                                            0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


PRINT = png(4, 4, (200, 60, 20, 255))


def generic_packs(root, texture="panel.png", declare_texture=True, image=PRINT):
    """Generic robot/sensor/pool/task set with a textured perforated panel; returns scenario."""
    scenario = write_generic_packs(root)
    (root / "robot" / "assets").mkdir()
    (root / "robot" / "assets" / "hull.obj").write_bytes(MESH)
    (root / "tasks" / "assets").mkdir()
    model = PANEL_DAE.format(texture=texture).encode()
    (root / "tasks" / "assets" / "panel.dae").write_bytes(model)
    (root / "tasks" / "assets" / "panel.png").write_bytes(image)
    (root / "outside.png").write_bytes(PRINT)

    def edit(relative, old, new):
        path = root / relative
        path.write_text(mutate(path.read_text("utf-8"), old, new), encoding="utf-8")

    edit("robot/robot.yaml", "assets: []\n", ASSET.format(digest=hashlib.sha256(MESH).hexdigest()))
    edit("robot/robot.yaml", "collision_boxes:\n", CAMERA_FRAMES)
    edit("robot/robot.yaml", "mechanisms:\n", CAMERA.replace("enabled: false", "enabled: true"))
    edit("pool/pool.yaml", "collision_boxes:\n", APPEARANCE.replace("outdoor", "indoor")
         .replace("ambient_light: 0.8", "ambient_light: 0.6") + "collision_boxes:\n")
    declared = (f"path: assets/panel.dae, source: modelled for tests, required_from_step: 3, "
                f"status: present, sha256: {hashlib.sha256(model).hexdigest()}}}")
    if declare_texture:
        declared += ("\n- {id: panel_print, path: assets/panel.png, source: modelled for tests, "
                     f"required_from_step: 3, status: present, "
                     f"sha256: {hashlib.sha256(image).hexdigest()}}}")
    edit("tasks/tasks.yaml", "path: assets/hoop.dae, source: modelled for tests, "
         "required_from_step: 3, status: missing}", declared)
    (root / "tasks" / "hoop.yaml").write_text(PANEL_TASK, encoding="utf-8")
    return scenario / "scenario.yaml"


@unittest.skipIf(pc is None, "optional camera extension is not installed")
class GenericPackCameraTests(unittest.TestCase):
    """A different robot, sensor id, pool, lighting and task: no Talos or year specifics."""

    def root(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        return Path(temporary.name)

    def test_generic_scene_sensor_ids_textured_cutouts(self):
        resolved = resolve_scenario(generic_packs(self.root()))
        cameras = pc.PackCameras(resolved)
        self.assertEqual(cameras.sensor_ids, ("cam",))
        record = cameras.describe()
        self.assertEqual(record["scene"]["pool"]["dimensions_m"], [10, 5, 3])
        self.assertEqual(record["appearance"]["profile"], "indoor")
        self.assertAlmostEqual(record["appearance"]["ambient_light"], 0.6, places=6)
        self.assertEqual(record["scene"]["cutouts"][0]["face_triangles"], [2])
        self.assertEqual(record["scene"]["robot_visuals"], 1)
        self.assertIn({"pack": "tasks", "id": "panel_print", "kind": "texture",
                       "path": "assets/panel.png", "sha256": hashlib.sha256(PRINT).hexdigest(),
                       "used_by": ["hoop_mesh"]}, record["scene"]["files"])
        self.assertEqual(record["cameras"]["cam"]["seeds"]["left"], pc.derive_seed(3, "cam", "left"))
        info = cameras.info("cam")
        placement = resolved.scenario["task_placements"][0]
        for uv, solid in (((0.5, 0.5), False), ((0.5, 0.8), True)):
            target = np.array([0.0, (uv[0] - 0.5), (uv[1] - 0.5)])
            world_eye = pc._upright(placement).compose(pc.pose(target + [1.0, 0, 0],
                                                               FACING_MINUS_X))
            root = root_for_eye(cameras, "cam_left", world_eye)
            pair = cameras.capture("cam", *root, 0.0)
            depth = centre_depth(pair.left, info)
            with self.subTest(uv=uv):
                if solid:
                    self.assertAlmostEqual(depth, 1.0, delta=0.002)
                    printed = pair.left.rgb[round(info["k"][5]), round(info["k"][2])]
                else:
                    self.assertTrue(math.isnan(depth) or depth > 1.05, depth)
                self.assertEqual(pair.right.rgb.shape, (48, 64, 3))
        # The panel pixel follows the declared print: a blue print renders differently.
        blue = pc.PackCameras(resolve_scenario(generic_packs(self.root(),
                                                             image=png(4, 4, (20, 60, 200, 255)))))
        other = blue.capture("cam", *root, 0.0).left.rgb[round(info["k"][5]), round(info["k"][2])]
        self.assertGreater(int(other[2]), int(printed[2]))

    def test_texture_dependencies_are_checked_before_rendering(self):
        cases = (
            ({"declare_texture": False}, None, "is not a declared pack asset"),
            ({"texture": "../../outside.png"}, None, "escapes the pack"),
            ({}, lambda root: (root / "tasks" / "assets" / "panel.png").write_bytes(
                png(4, 4, (1, 2, 3, 255))), "sha256 mismatch"),
            ({}, lambda root: (root / "tasks" / "assets" / "panel.png").unlink(),
             "is unreadable"),
        )
        for options, after_resolve, message in cases:
            with self.subTest(message=message):
                root = self.root()
                resolved = resolve_scenario(generic_packs(root, **options))
                if after_resolve is not None:
                    after_resolve(root)
                with self.assertRaisesRegex(ValueError, f"texture .* {message}"):
                    pc.PackCameras(resolved)


@unittest.skipIf(pc is None, "optional camera extension is not installed")
class NativeCoexistenceTests(unittest.TestCase):
    def test_camera_spatial_types_coexist_with_the_native_extension(self):
        script = "\n".join([
            "import numpy as np",
            "from robotics_platform import _camera, _native",
            "def edge(module, parent, child, position, wxyz):",
            "    item = module.FixedFrame(); item.parent, item.child = parent, child",
            "    pose = module.Pose(); pose.translation = position",
            "    pose.orientation_wxyz = wxyz; item.pose = pose; return item",
            "q = [np.cos(0.3), 0, np.sin(0.3), 0]",
            "results = []",
            "for module in (_native, _camera):",
            "    frames = module.FixedFrames('com', [edge(module, 'com', 'cad', [1, 2, 3], q),",
            "                                        edge(module, 'cad', 'eye', [0.1, 0, 0], q)])",
            "    results.append(frames.from_root('eye'))",
            "np.testing.assert_array_equal(results[0].translation, results[1].translation)",
            "np.testing.assert_array_equal(results[0].orientation_wxyz,",
            "                              results[1].orientation_wxyz)",
            "assert type(results[0]) is not type(results[1])",
            "# Same C++ type: each extension accepts the other's value and composes identically.",
            "a, b = results[0].compose(results[1]), results[1].compose(results[0])",
            "np.testing.assert_array_equal(a.translation, b.translation)",
            "np.testing.assert_array_equal(a.orientation_wxyz, b.orientation_wxyz)",
            "bad = _native.Pose(); bad.orientation_wxyz = [1 + 1e-7, 0, 0, 0]",
            "for pose in (results[0], results[1]):",
            "    try:",
            "        pose.compose(bad)",
            "    except ValueError:",
            "        continue",
            "    raise AssertionError('both extensions must apply native pose validation')",
        ])
        probe = subprocess.run([sys.executable, "-c", "import robotics_platform._native"],
                               capture_output=True)
        if probe.returncode:
            self.skipTest("native extension is not installed alongside the camera extension")
        subprocess.run([sys.executable, "-c", script], check=True)


@unittest.skipIf(pc is None, "optional camera extension is not installed")
class CaptureConcurrencyTests(unittest.TestCase):
    def cameras(self):
        """Native pose validation with controlled GL/CPU boundaries and owned fake frames."""
        cameras = object.__new__(pc.PackCameras)
        cameras._lock = threading.Lock()
        cameras._capture_locks = {key: threading.Lock() for key in ("a", "b")}
        cameras._scene = SimpleNamespace(instances=[])
        cameras._static, cameras._robot_visuals = [], []
        cameras._appearance = None
        captures = []

        def capture(scene, view, *args):
            captures.append(view)
            return view

        cameras._host = SimpleNamespace(capture=capture)
        cameras._view = lambda camera, eye, root: (camera["id"], eye)
        cameras._cameras = {}
        for key in ("a", "b"):
            processors = {}
            for eye in pc.EYES:
                state = {"count": 0}

                def process(intrinsics, noise, raw, *, jpeg, quality, state=state):
                    state["count"] += 1
                    return (raw, state["count"])

                def reset(seed, state=state):
                    state["count"] = 0

                processors[eye] = SimpleNamespace(process=process, reset=reset)
            cameras._cameras[key] = {
                "id": key, "outputs": {"rgb_left", "rgb_right", "depth_left"}, "noise": None,
                "intrinsics": {eye: SimpleNamespace(width=1, height=1) for eye in pc.EYES},
                "processors": processors, "seeds": {},
            }
        cameras.reset(7)
        return cameras, captures

    def test_other_camera_processing_overlaps_but_same_camera_stays_ordered(self):
        cameras, captures = self.cameras()
        entered, release = threading.Event(), threading.Event()
        processor = cameras._cameras["a"]["processors"]["left"]
        original = processor.process

        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("test processing was not released")
            return original(*args, **kwargs)

        processor.process = blocked
        with ThreadPoolExecutor(3) as workers:
            a = workers.submit(cameras.capture, "a", [0, 0, 0], IDENTITY, 0)
            try:
                self.assertTrue(entered.wait(2))
                again = workers.submit(cameras.capture, "a", [0, 0, 0], IDENTITY, 0)
                b = workers.submit(cameras.capture, "b", [0, 0, 0], IDENTITY, 0)
                self.assertEqual(b.result(timeout=2).left, (("b", "left"), 1))
                self.assertFalse(again.done())
                self.assertEqual(captures, [("a", "left"), ("a", "right"),
                                            ("b", "left"), ("b", "right")])
            finally:
                release.set()
            self.assertEqual(a.result(timeout=2).left, (("a", "left"), 1))
            self.assertEqual(again.result(timeout=2).left, (("a", "left"), 2))

    def test_reset_waits_for_processing_before_resetting_either_stereo_eye(self):
        cameras, _ = self.cameras()
        entered, release, resetting = threading.Event(), threading.Event(), threading.Event()
        processor = cameras._cameras["b"]["processors"]["left"]
        original = processor.process

        def blocked(*args, **kwargs):
            entered.set()
            if not release.wait(5):
                raise TimeoutError("test processing was not released")
            return original(*args, **kwargs)

        processor.process = blocked

        def reset():
            resetting.set()
            cameras.reset(99)

        with ThreadPoolExecutor(2) as workers:
            capture = workers.submit(cameras.capture, "b", [0, 0, 0], IDENTITY, 0)
            try:
                self.assertTrue(entered.wait(2))
                restart = workers.submit(reset)
                self.assertTrue(resetting.wait(2))
                self.assertFalse(restart.done())
            finally:
                release.set()
            pair = capture.result(timeout=2)
            restart.result(timeout=2)
        self.assertEqual(pair.left, (("b", "left"), 1))
        self.assertEqual(pair.right, (("b", "right"), 1))
        replay = cameras.capture("b", [0, 0, 0], IDENTITY, 0)
        self.assertEqual(replay, pair)
        self.assertEqual(cameras._seed, 99)


if __name__ == "__main__":
    unittest.main()
