"""Offscreen-camera pack fields: robot visuals, stereo right eye, pool water tint and lighting."""

import tempfile
import unittest
from pathlib import Path
from typing import Any

from nereus.packs import PackError, load_pack, resolve_scenario
from test_packs_fixtures import write_generic_packs

CONTENT = Path(__file__).resolve().parents[2] / "content" / "packs"
IDENTITY = [1, 0, 0, 0]
ORIGIN = [0, 0, 0]
MESH = b"o hull\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"

ASSET = "assets:\n- {id: hull_mesh, path: assets/hull.obj}\n"
CAMERA_FRAMES = """\
  - {parent: base_link, child: cam_left, position_m: [0.2, 0, 0], orientation_wxyz: [0.5, -0.5, 0.5, -0.5]}
  - {parent: cam_left, child: cam_right, position_m: [0.05, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
visuals:
- {asset: hull_mesh, frame: base_link, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
collision_boxes:
"""
CAMERA = """\
- id: cam
  type: stereo_camera
  frame: cam_left
  mount_frame: base_link
  enabled: false
  rate_hz: 15
  parameters:
    baseline_m: 0.05
    right_frame: cam_right
    resolution_px: [64, 48]
    intrinsics_left: {fx: 50.0, fy: 50.0, cx: 31.5, cy: 23.5}
    intrinsics_right: {fx: 50.0, fy: 50.0, cx: 31.5, cy: 23.5}
    outputs: [rgb_left, depth_left, camera_info, rgb_right, camera_info_right]
    depth:
      min_range_m: 0.15
      max_range_m: 4.0
      noise: {base_sigma_m: 0.002, range_coefficient: 0.0015, range_exponent: 2.0, bias_m: 0.0, dropout: 0.005, range_dropout: 0.1, edge_dropout: 0.2, outliers: 0.002, correlation: 0.5, patch_size_px: 8}
mechanisms:
"""
APPEARANCE = """\
water_optics:
  tint_rgb: [0.025, 0.22, 0.29]
  distance_scale: 1.88
  distance_power: 0.40
  clear_distance_m: 0.0
  scattering: 0.458
  absorption_per_m_rgb: [0.648, 0.145, 0.025]
lighting:
  profile: outdoor
  direct_light: 1.0
  ambient_light: 0.8
  sun_azimuth_deg: 225.0
  sun_elevation_deg: 55.0
  glare: 0.5
"""


def mutate(text: str, old: str, new: str) -> str:
    assert text.count(old) == 1, f"expected exactly one {old!r}, found {text.count(old)}"
    return text.replace(old, new)


class TalosCameraFieldTests(unittest.TestCase):
    """The Talos camera and 2026 pool appearance data."""

    robot: dict[str, Any]
    pool: dict[str, Any]

    @classmethod
    def setUpClass(cls) -> None:
        cls.robot = load_pack(CONTENT / "robots" / "talos").plain()
        cls.pool = load_pack(CONTENT / "pools" / "robosub_2026").plain()

    def test_visuals_use_present_assets_at_their_initial_configuration(self) -> None:
        present = {item["id"] for item in self.robot["assets"]}
        placed: dict[str, list[str]] = {}
        for visual in self.robot["visuals"]:
            self.assertIn(visual["asset"], present)
            magnet = next(item for item in self.robot["mechanisms"] if item["type"] == "magnet")
            # The magnet mesh origin sits at the mechanism tip; every other visual is at its frame origin.
            self.assertEqual(
                visual["position_m"],
                magnet["parameters"]["tip_position_m"]
                if visual["asset"] == "robot_magnet_mesh"
                else ORIGIN,
            )
            self.assertEqual(visual["orientation_wxyz"], IDENTITY)
            placed.setdefault(visual["frame"], []).append(visual["asset"])
        rotors = [f"rotor_{item['id']}" for item in self.robot["thrusters"]]
        self.assertEqual(sorted(placed["cad"]), sorted(["body_mesh", *rotors]))
        claw = next(item for item in self.robot["mechanisms"] if item["type"] == "claw")
        self.assertEqual(claw["parameters"]["initial_state"], "closed")
        self.assertEqual(
            sorted(placed[claw["frame"]]),
            sorted(
                [
                    "claw_static_mesh",
                    "claw_left_mesh",
                    "claw_right_mesh",
                    "claw_left_pad_mesh",
                    "claw_right_pad_mesh",
                ]
            ),
        )
        self.assertEqual(placed.pop("magnet_mount"), ["robot_magnet_mesh"])
        self.assertEqual(set(placed), {"cad", claw["frame"]})

    def test_stereo_cameras_are_enabled_with_rectified_right_eyes(self) -> None:
        transforms = {item["child"]: item for item in self.robot["frames"]["transforms"]}
        cameras = [item for item in self.robot["sensors"] if item["type"] == "stereo_camera"]
        self.assertEqual({item["id"] for item in cameras}, {"ffc", "dfc"})
        for camera in cameras:
            parameters = camera["parameters"]
            with self.subTest(camera=camera["id"]):
                self.assertTrue(camera["enabled"])
                self.assertEqual(
                    parameters["outputs"], ["rgb_left", "depth_left", "camera_info", "point_cloud"]
                )
                self.assertEqual(parameters["intrinsics_left"], parameters["intrinsics_right"])
                right = transforms[parameters["right_frame"]]
                self.assertEqual(right["parent"], camera["frame"])
                self.assertEqual(right["position_m"], [parameters["baseline_m"], 0, 0])
                self.assertEqual(right["orientation_wxyz"], IDENTITY)
        baselines = {item["id"]: item["parameters"]["baseline_m"] for item in cameras}
        self.assertEqual(baselines, {"ffc": 0.04975591649077412, "dfc": 0.05030758651145167})

    def test_pool_sensor_appearance_is_the_original_startup_look(self) -> None:
        optics = self.pool["water_optics"]
        self.assertEqual(optics["tint_rgb"], [0.025, 0.22, 0.29])
        self.assertEqual(optics["absorption_per_m_rgb"], [0.648, 0.145, 0.025])
        self.assertEqual(
            self.pool["lighting"],
            {
                "profile": "outdoor",
                "direct_light": 1.0,
                "ambient_light": 0.8,
                "sun_azimuth_deg": 225.0,
                "sun_elevation_deg": 55.0,
                "glare": 0.5,
            },
        )

    def test_unedited_packs_save_byte_exact(self) -> None:
        for relative, name in (("robots/talos", "robot.yaml"), ("pools/robosub_2026", "pool.yaml")):
            with self.subTest(pack=relative):
                folder = CONTENT / relative
                self.assertEqual(load_pack(folder).dumps(), (folder / name).read_text("utf-8"))


class GenericCameraFieldTests(unittest.TestCase):
    """A generic robot/pool with every new field; one mutation per rejection."""

    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / "packs"
        self.root.mkdir()
        self.scenario = write_generic_packs(self.root)
        (self.root / "robot" / "assets").mkdir()
        (self.root / "robot" / "assets" / "hull.obj").write_bytes(MESH)
        self.edit("robot", "assets: []\n", ASSET)
        self.edit("robot", "collision_boxes:\n", CAMERA_FRAMES)
        self.edit("robot", "mechanisms:\n", CAMERA)
        pool = self.root / "pool" / "pool.yaml"
        pool.write_text(pool.read_text("utf-8") + APPEARANCE, encoding="utf-8")

    def edit(self, pack: str, old: str, new: str) -> None:
        target = self.root / pack / f"{pack}.yaml"
        target.write_text(mutate(target.read_text("utf-8"), old, new), encoding="utf-8")

    def rejects(self, pack: str, old: str, new: str, fragment: str) -> None:
        self.edit(pack, old, new)
        with self.assertRaises(PackError) as caught:
            load_pack(self.root / pack)
        self.assertIn(fragment, str(caught.exception))

    def test_all_new_fields_validate_resolve_and_round_trip(self) -> None:
        resolve_scenario(self.scenario)
        for pack in ("robot", "pool"):
            with self.subTest(pack=pack):
                document = load_pack(self.root / pack)
                source = (self.root / pack / f"{pack}.yaml").read_text("utf-8")
                self.assertEqual(document.dumps(), source)
                copy = document.save(self.root / pack / "copy.yaml")
                self.assertEqual(copy.read_text("utf-8"), source)
        robot = load_pack(self.root / "robot").plain()
        self.assertEqual(robot["visuals"][0]["asset"], "hull_mesh")
        self.assertEqual(robot["sensors"][-1]["parameters"]["right_frame"], "cam_right")
        pool = load_pack(self.root / "pool").plain()
        self.assertEqual(pool["lighting"]["profile"], "outdoor")
        self.assertEqual(pool["water_optics"]["tint_rgb"], [0.025, 0.22, 0.29])

    def test_optional_fields_may_be_omitted(self) -> None:
        self.edit("robot", "    right_frame: cam_right\n", "")
        self.edit("robot", "rgb_right, camera_info_right", "rgb_right")
        with self.assertRaises(PackError):  # right outputs still listed
            load_pack(self.root / "robot")
        self.edit("robot", "camera_info, rgb_right]", "camera_info]")
        self.edit("pool", "  tint_rgb: [0.025, 0.22, 0.29]\n", "")
        load_pack(self.root / "robot")
        load_pack(self.root / "pool")

    def test_visual_references_are_checked(self) -> None:
        cases = [
            ("asset: hull_mesh, frame", "asset: ghost, frame", "unknown asset 'ghost'"),
            (
                "frame: base_link, position_m: [0, 0, 0]",
                "frame: nowhere, position_m: [0, 0, 0]",
                "unknown frame 'nowhere'",
            ),
            (
                "frame: base_link, position_m: [0, 0, 0]",
                "frame: world, position_m: [0, 0, 0]",
                "unknown frame 'world'",
            ),
            (
                "orientation_wxyz: [1, 0, 0, 0]}\ncollision_boxes",
                "orientation_wxyz: [1, 0, 0, 0.1]}\ncollision_boxes",
                "must have unit norm",
            ),
            (
                "orientation_wxyz: [1, 0, 0, 0]}\ncollision_boxes",
                "orientation_wxyz: [1, 0, 0, 0], scale: 2}\ncollision_boxes",
                "scale",
            ),
        ]
        for old, new, fragment in cases:
            with self.subTest(fragment=fragment):
                self.setUp()
                self.rejects("robot", old, new, fragment)

    def test_right_eye_must_be_the_rectified_baseline(self) -> None:
        cases = [
            ("    right_frame: cam_right\n", "", "right-eye outputs require right_frame"),
            ("    right_frame: cam_right\n", "    right_frame: ghost\n", "must be a child"),
            ("    right_frame: cam_right\n", "    right_frame: base_link\n", "must be a child"),
            (
                "child: cam_right, position_m: [0.05, 0, 0]",
                "child: cam_right, position_m: [0.045, 0, 0]",
                "must be at [baseline_m, 0, 0]",
            ),
            (
                "child: cam_right, position_m: [0.05, 0, 0]",
                "child: cam_right, position_m: [0.05, 0.001, 0]",
                "must be at [baseline_m, 0, 0]",
            ),
            (
                "[0.05, 0, 0], orientation_wxyz: [1, 0, 0, 0]}",
                "[0.05, 0, 0], orientation_wxyz: [0, 0, 0, 1]}",
                "identity orientation",
            ),
        ]
        for old, new, fragment in cases:
            with self.subTest(fragment=fragment):
                self.setUp()
                self.rejects("robot", old, new, fragment)

    def test_camera_values_outside_native_invariants_are_rejected(self) -> None:
        cases = [
            ("resolution_px: [64, 48]", "resolution_px: [4097, 48]", "resolution_px"),
            ("resolution_px: [64, 48]", "resolution_px: [64, 0]", "resolution_px"),
            ("rgb_right, camera_info_right]", "rgb_right, depth_right]", "outputs"),
            ("min_range_m: 0.15", "min_range_m: 0.0", "min_range_m"),
            ("max_range_m: 4.0", "max_range_m: 101.0", "max_range_m"),
            ("max_range_m: 4.0", "max_range_m: 0.1", "min_range_m must be < max_range_m"),
            ("range_exponent: 2.0", "range_exponent: 4.5", "range_exponent"),
            ("patch_size_px: 8", "patch_size_px: 65", "patch_size_px"),
            (
                "fx: 50.0, fy: 50.0, cx: 31.5, cy: 23.5}\n    outputs",
                "fx: 0.0, fy: 50.0, cx: 31.5, cy: 23.5}\n    outputs",
                "fx",
            ),
        ]
        for old, new, fragment in cases:
            with self.subTest(case=new):
                self.setUp()
                self.rejects("robot", old, new, fragment)

    def test_pool_appearance_outside_original_ranges_is_rejected(self) -> None:
        cases = [
            ("tint_rgb: [0.025, 0.22, 0.29]", "tint_rgb: [0.025, 1.2, 0.29]", "tint_rgb"),
            ("tint_rgb: [0.025, 0.22, 0.29]", "tint_rgb: [0.025, 0.22]", "tint_rgb"),
            (
                "absorption_per_m_rgb: [0.648, 0.145, 0.025]",
                "absorption_per_m_rgb: [5.1, 0.145, 0.025]",
                "absorption_per_m_rgb",
            ),
            ("scattering: 0.458", "scattering: 5.5", "scattering"),
            ("distance_power: 0.40", "distance_power: 0.2", "distance_power"),
            ("distance_scale: 1.88", "distance_scale: 5.5", "distance_scale"),
            ("clear_distance_m: 0.0", "clear_distance_m: 21.0", "clear_distance_m"),
            ("profile: outdoor", "profile: sunset", "profile"),
            ("sun_elevation_deg: 55.0", "sun_elevation_deg: 91.0", "sun_elevation_deg"),
            ("ambient_light: 0.8", "ambient_light: -0.1", "ambient_light"),
            ("glare: 0.5", "glare: -0.5", "glare"),
            ("  glare: 0.5\n", "", "glare"),
            ("  glare: 0.5\n", "  glare: 0.5\n  exposure: 1.0\n", "exposure"),
        ]
        for old, new, fragment in cases:
            with self.subTest(case=new):
                self.setUp()
                self.rejects("pool", old, new, fragment)


if __name__ == "__main__":
    unittest.main()
