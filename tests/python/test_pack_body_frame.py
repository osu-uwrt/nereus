"""A robot measured from a datum resolves to the same body-frame geometry as one rooted at its COM."""

import math
import unittest
from typing import Any

from nereus.packs import resolve_scenario
from test_packs_rejections import PackRejectionCase

ROBOT_FILE = "robot/robot.yaml"
T0 = "- {id: t0, type: lagged_force, position_m: [-0.3, 0.2, 0], direction: [1, 0, 0], parameters:"
TARGET = "parameters: {target_position_body_m: [0, 0, -0.05], noise:"
HULL = "- {id: hull, size_m: [0.6, 0.4, 0.3], center_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}"
# The COM sits at [0.5, 0.25, -0.125] in `datum`, yawed +90 degrees: body [x, y, z] is datum [-y, x, z].
YAW = "[0.7071067811865476, 0, 0, 0.7071067811865476]"
DATUM_FRAMES = f"""\
frames:
  root: datum
  body: com
  transforms:
  - {{parent: datum, child: com, position_m: [0.5, 0.25, -0.125], orientation_wxyz: {YAW}}}
  - {{parent: datum, child: base_link, position_m: [0.5, 0.25, -0.175], orientation_wxyz: {YAW}}}
  - {{parent: base_link, child: imu_mount, position_m: [0.1, 0, 0], orientation_wxyz: [1, 0, 0, 0]}}
  - {{parent: datum, child: thruster_t0, position_m: [0.3, -0.05, -0.125], orientation_wxyz: {YAW}}}
"""
COM_FRAMES = """\
frames:
  root: com
  transforms:
  - {parent: com, child: base_link, position_m: [0, 0, -0.05], orientation_wxyz: [1, 0, 0, 0]}
  - {parent: base_link, child: imu_mount, position_m: [0.1, 0, 0], orientation_wxyz: [1, 0, 0, 0]}
"""


def from_root(frames: dict[str, Any]) -> dict[str, list[float]]:
    """Frame positions in the root frame (every rotation in these packs is a yaw)."""
    edges = {item["child"]: item for item in frames["transforms"]}

    def pose(name: str) -> tuple[list[float], float]:
        if name == frames["root"]:
            return [0.0, 0.0, 0.0], 0.0
        edge = edges[name]
        (x, y, z), yaw = pose(edge["parent"])
        px, py, pz = edge["position_m"]
        w, _, _, qz = edge["orientation_wxyz"]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        position = [x + cosine * px - sine * py, y + sine * px + cosine * py, z + pz]
        return position, yaw + 2 * math.atan2(qz, w)

    return {name: pose(name)[0] for name in [frames["root"], *edges]}


class DatumRootedRobotTests(PackRejectionCase):
    def assert_close(self, actual: Any, expected: Any, where: str = "") -> None:
        if isinstance(expected, dict):
            self.assertEqual(sorted(actual), sorted(expected), where)
            for key in expected:
                self.assert_close(actual[key], expected[key], f"{where}/{key}")
        elif isinstance(expected, list):
            self.assertEqual(len(actual), len(expected), where)
            for index, item in enumerate(expected):
                self.assert_close(actual[index], item, f"{where}/{index}")
        elif isinstance(expected, float) or isinstance(actual, float):
            self.assertAlmostEqual(actual, expected, delta=1e-12, msg=where)
        else:
            self.assertEqual(actual, expected, where)

    def measure_from_datum(self) -> None:
        self.edit(ROBOT_FILE, COM_FRAMES, DATUM_FRAMES)
        self.edit(ROBOT_FILE, T0, "- {id: t0, type: lagged_force, frame: thruster_t0, parameters:")
        self.edit(ROBOT_FILE, TARGET, "parameters: {target_frame: base_link, noise:")
        self.edit(
            ROBOT_FILE,
            HULL,
            "- {id: hull, frame: datum, size_m: [0.6, 0.4, 0.3], center_m: [0.5, 0.25, -0.125], "
            f"orientation_wxyz: {YAW}}}",
        )

    def test_resolves_to_the_com_rooted_geometry(self) -> None:
        expected = resolve_scenario(self.scenario).robot
        self.measure_from_datum()
        robot = resolve_scenario(self.scenario).robot
        frames = robot.pop("frames")
        expected_frames = expected.pop("frames")
        self.assert_close(robot, expected)  # thruster, hull box and altitude target among the rest
        self.assertEqual(frames["root"], "com")
        self.assertNotIn("body", frames)
        positions = from_root(frames)
        self.assertEqual(set(positions), {"com", "datum", "base_link", "imu_mount", "thruster_t0"})
        self.assert_close(positions["datum"], [-0.25, 0.5, 0.125])
        self.assert_close(positions["thruster_t0"], [-0.3, 0.2, 0.0])
        for name, position in from_root(expected_frames).items():
            self.assert_close(positions[name], position, name)

    def test_com_rooted_pack_is_left_as_authored(self) -> None:
        robot = resolve_scenario(self.scenario).robot
        self.assertEqual(robot["frames"]["transforms"][0]["position_m"], [0, 0, -0.05])
        self.assertEqual(robot["thrusters"][0]["position_m"], [-0.3, 0.2, 0])
        self.assertEqual(robot["collision_boxes"][0]["center_m"], [0, 0, 0])

    def test_frame_references_are_checked(self) -> None:
        self.measure_from_datum()
        for old, new, fragment in (
            ("  body: com\n", "  body: ghost\n", "/frames/body: unknown frame 'ghost'"),
            ("frame: thruster_t0,", "frame: ghost,", "/thrusters/t0/frame: unknown frame 'ghost'"),
            ("target_frame: base_link", "target_frame: ghost", "/parameters/target_frame: unknown"),
            ("{id: hull, frame: datum,", "{id: hull, frame: ghost,", "/collision_boxes/hull/frame"),
            (
                "frame: thruster_t0,",
                "frame: thruster_t0, direction: [1, 0, 0],",
                "/thrusters/t0: give frame, or position_m and direction, not both",
            ),
            (
                "target_frame: base_link,",
                "target_frame: base_link, target_position_body_m: [0, 0, 0],",
                "give target_frame or target_position_body_m, not both",
            ),
        ):
            with self.subTest(new=new):
                self.edit(ROBOT_FILE, old, new)
                self.assert_load_rejects("robot", fragment)
                self.edit(ROBOT_FILE, new, old)

    def test_thruster_needs_a_placement(self) -> None:
        self.edit(
            ROBOT_FILE,
            "position_m: [-0.3, 0.2, 0], direction: [1, 0, 0], ",
            "position_m: [-0.3, 0.2, 0], ",
        )
        self.assert_load_rejects("robot", "/thrusters/t0: needs frame, or position_m and direction")


if __name__ == "__main__":
    unittest.main()
