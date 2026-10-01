"""Equipment packs and the scenario placement tree: anything may be placed relative to the world, the pool, a
task, an equipment placement or an item frame, and the world itself may be placed (the pool is then the root)."""

import json
import math
import unittest
from pathlib import Path
from typing import Any

from nereus.packs import load_pack, resolve_scenario
from test_packs import TALOS
from test_packs_rejections import PackRejectionCase

# A board whose `tag` frame is 0.1 m up its face, with apriltag-style axes (x right, y up, z out of the face).
EQUIPMENT = """\
kind: equipment
id: kit
assets:
- {id: board, path: assets/board.obj}
items:
- id: board
  asset: board
  frames:
  - {id: tag, position_m: [0.003, 0, 0.1], orientation_wxyz: [0.5, 0.5, 0.5, 0.5]}
"""
POOL_PLACEMENT = "pool_placement: {position_m: [0, 0, 0], yaw_deg: 0}\n"
HOOP = "- {task: hoop, position_m: [5, 2.5, -1], yaw_deg: 90}\n"
INITIAL = "initial: {frame: com, position_m: [2, 2, -1],"


def close(test: unittest.TestCase, actual: Any, expected: Any, where: str = "") -> None:
    if isinstance(expected, dict):
        test.assertEqual(sorted(actual), sorted(expected), where)
        for key in expected:
            close(test, actual[key], expected[key], f"{where}/{key}")
    elif isinstance(expected, list):
        test.assertEqual(len(actual), len(expected), where)
        for index, item in enumerate(expected):
            close(test, actual[index], item, f"{where}/{index}")
    elif isinstance(expected, float):
        test.assertAlmostEqual(actual, expected, delta=1e-9, msg=where)
    else:
        test.assertEqual(actual, expected, where)


class EquipmentCase(PackRejectionCase):
    def add_equipment(self, placements: str = "") -> None:
        folder = self.root / "equipment"
        (folder / "assets").mkdir(parents=True)
        (folder / "assets" / "board.obj").write_bytes(b"mesh")
        (folder / "equipment.yaml").write_text(EQUIPMENT, encoding="utf-8")
        self.edit(
            "scenario/scenario.yaml",
            "bridge: ../bridge\n",
            "bridge: ../bridge\nequipment: ../equipment\n",
        )
        if placements:
            self.edit(
                "scenario/scenario.yaml", "task_placements:\n", placements + "task_placements:\n"
            )

    def placed(self) -> dict[str, Any]:
        resolved = resolve_scenario(self.scenario)
        assert resolved.equipment is not None
        return {item["id"]: item for item in resolved.equipment["placed"]}


class EquipmentPackTests(EquipmentCase):
    def test_scenario_without_equipment_resolves_as_before(self) -> None:
        resolved = resolve_scenario(self.scenario)
        self.assertIsNone(resolved.equipment)
        self.assertNotIn("equipment", resolved.manifest())
        self.assertNotIn("equipment", resolved.asset_paths())

    def test_unplaced_items_are_not_drawn(self) -> None:
        self.add_equipment()
        resolved = resolve_scenario(self.scenario)
        assert resolved.equipment is not None
        self.assertEqual(resolved.equipment["placed"], [])
        self.assertEqual(resolved.manifest()["equipment"], resolved.equipment)
        path = Path(resolved.asset_paths()["equipment"]["board"])
        self.assertEqual(path, (self.root / "equipment" / "assets" / "board.obj").resolve())

    def test_item_must_name_a_declared_asset(self) -> None:
        self.add_equipment()
        self.edit("equipment/equipment.yaml", "  asset: board\n", "  asset: ghost\n")
        self.assert_load_rejects("equipment", "/items/board/asset: unknown asset 'ghost'")

    def test_item_frames_need_unit_orientations(self) -> None:
        self.add_equipment()
        self.edit("equipment/equipment.yaml", "[0.5, 0.5, 0.5, 0.5]", "[0.5, 0.5, 0.5, 0.4]")
        self.assert_load_rejects("equipment", "must have unit norm")


class PlacementTreeTests(EquipmentCase):
    def test_world_placements_are_left_as_authored(self) -> None:
        scenario = resolve_scenario(self.scenario).scenario
        self.assertEqual(scenario["pool_placement"], {"position_m": [0, 0, 0], "yaw_deg": 0})
        self.assertEqual(scenario["task_placements"][0]["position_m"], [5, 2.5, -1])

    def test_equipment_relative_to_the_pool_follows_it(self) -> None:
        self.edit(
            "scenario/scenario.yaml",
            POOL_PLACEMENT,
            "pool_placement: {position_m: [10, 0, 0], yaw_deg: 90}\n",
        )
        self.add_equipment(
            "equipment_placements:\n- {item: board, relative_to: pool, position_m: [1, 0, -0.3]}\n"
        )
        board = self.placed()["board"]
        close(self, board["position_m"], [10.0, 1.0, -0.3])
        half = math.sqrt(0.5)
        close(self, board["orientation_wxyz"], [half, 0.0, 0.0, half])

    def test_tasks_and_start_may_hang_from_equipment(self) -> None:
        self.add_equipment(
            "equipment_placements:\n- {item: board, position_m: [1, 2, -0.5], yaw_deg: 180}\n"
        )
        self.edit(
            "scenario/scenario.yaml",
            HOOP,
            "- {task: hoop, relative_to: board, position_m: [3, 0, -0.5], yaw_deg: 90}\n",
        )
        self.edit(
            "scenario/scenario.yaml",
            INITIAL,
            "initial: {frame: com, relative_to: board, position_m: [2, 0, -0.5],",
        )
        scenario = resolve_scenario(self.scenario).scenario
        close(
            self,
            scenario["task_placements"][0],
            {"task": "hoop", "position_m": [-2.0, 2.0, -1.0], "yaw_deg": -90.0},
        )
        close(self, scenario["initial"]["position_m"], [-1.0, 2.0, -1.0])
        close(self, scenario["initial"]["orientation_wxyz"], [0.0, 0.0, 0.0, 1.0])
        self.assertNotIn("relative_to", scenario["initial"])

    def test_world_placed_from_an_item_frame_makes_the_pool_the_root(self) -> None:
        # The board hangs on the pool's x = 0 wall facing +x; the world is 0.4 m up the tag's +y, axes like a map
        # (x out of the face, z up): the static transform a stack would publish from the tag.
        self.add_equipment(
            "equipment_placements:\n- {item: board, relative_to: pool, position_m: [0, 4, -0.5]}\n"
        )
        self.edit(
            "scenario/scenario.yaml",
            POOL_PLACEMENT,
            "world_placement: {relative_to: board/tag, position_m: [0, 0.4, 0], rpy_deg: [-90, -90, 0]}\n",
        )
        resolved = resolve_scenario(self.scenario)
        # World origin = pool (0.003, 4, 0): the tag face, 0.1 + 0.4 m above the board origin.
        close(
            self,
            resolved.scenario["pool_placement"],
            {"position_m": [-0.003, -4.0, 0.0], "yaw_deg": 0.0},
        )
        assert resolved.equipment is not None
        close(self, resolved.equipment["placed"][0]["position_m"], [-0.003, 0.0, -0.5])

    def test_rejections(self) -> None:
        self.add_equipment(
            "equipment_placements:\n- {item: board, relative_to: pool, position_m: [0, 4, -0.5]}\n"
        )
        scenario = self.root / "scenario" / "scenario.yaml"
        original = scenario.read_text(encoding="utf-8")
        world = "world_placement: {relative_to: board/tag, position_m: [0, 0.4, 0], rpy_deg: [-90, -90, 0]}\n"
        hoop = "- {task: hoop, %sposition_m: [5, 2.5, -1]%s}\n"
        for old, new, fragment in (
            (
                "relative_to: pool,",
                "relative_to: ghost,",
                "/equipment_placements/0/relative_to: unknown frame 'ghost'",
            ),
            ("{item: board,", "{item: crate,", "unknown equipment item 'crate'"),
            (POOL_PLACEMENT, "", "needs pool_placement or world_placement"),
            (
                POOL_PLACEMENT,
                POOL_PLACEMENT + world,
                "give pool_placement or world_placement, not both",
            ),
            (
                POOL_PLACEMENT,
                "world_placement: {position_m: [0, 0, 0], yaw_deg: 0}\n",
                "must name another frame",
            ),
            (HOOP, hoop % ("", ""), "/task_placements/0: needs yaw_deg or rpy_deg"),
            (
                HOOP,
                hoop % ("", ", yaw_deg: 90, rpy_deg: [0, 0, 90]"),
                "give yaw_deg or rpy_deg, not both",
            ),
            (
                HOOP,
                hoop % ("relative_to: board/tag, ", ", yaw_deg: 0"),
                "/task_placements/0: must be upright",
            ),
            (HOOP, hoop % ("relative_to: hoop, ", ", yaw_deg: 90"), "placement cycle"),
        ):
            with self.subTest(new=new):
                scenario.write_text(original.replace(old, new, 1), encoding="utf-8")
                self.assert_resolve_rejects(fragment)
        scenario.write_text(original, encoding="utf-8")
        resolve_scenario(self.scenario)

    def test_cycle_through_equipment(self) -> None:
        self.add_equipment(
            "equipment_placements:\n- {item: board, relative_to: hoop, position_m: [0, 0, 0]}\n"
        )
        self.edit(
            "scenario/scenario.yaml",
            HOOP,
            "- {task: hoop, relative_to: board, position_m: [5, 2.5, -1], yaw_deg: 90}\n",
        )
        self.assert_resolve_rejects("placement cycle")

    def test_placements_without_an_equipment_pack(self) -> None:
        self.edit(
            "scenario/scenario.yaml",
            "task_placements:\n",
            "equipment_placements:\n- {item: board, position_m: [0, 0, 0]}\ntask_placements:\n",
        )
        self.assert_resolve_rejects("the scenario selects no equipment pack")


class UwrtEquipmentTests(unittest.TestCase):
    def test_board_tag_frame_matches_the_print(self) -> None:
        board = load_pack(TALOS / "equipment" / "uwrt").plain()["items"][0]
        self.assertEqual([frame["id"] for frame in board["frames"]], ["tag"])

    def test_rpac_map_origin_is_on_the_board_and_the_course_is_unchanged(self) -> None:
        rpac = resolve_scenario(TALOS / "scenarios" / "talos_uwrt_rpac")
        robosub = resolve_scenario(TALOS / "scenarios" / "talos_uwrt")
        # Map origin at the tag face, 3 mm out from the deep-end wall, centred on lap line 3; level and upright.
        close(
            self,
            rpac.scenario["pool_placement"],
            {"position_m": [-0.003, -5.334, 0.0], "yaw_deg": 0.0},
        )
        self.assertEqual(rpac.scenario["task_placements"], robosub.scenario["task_placements"])
        for resolved in (rpac, robosub):
            assert resolved.equipment is not None
            self.assertEqual(
                [item["id"] for item in resolved.equipment["placed"]], ["calibration_board"]
            )
            self.assertTrue(
                Path(resolved.asset_paths()["equipment"]["calibration_board"]).is_file()
            )
            json.dumps(resolved.manifest(), allow_nan=False)
        # RoboSub keeps its authored world placements: the board hangs on the wall at the map origin.
        self.assertEqual(
            robosub.scenario["pool_placement"],
            {"position_m": [0.0, 19.5136, 0.0], "yaw_deg": -90.0},
        )
        assert robosub.equipment is not None
        close(self, robosub.equipment["placed"][0]["position_m"], [0.0, 0.0, -0.3394])


if __name__ == "__main__":
    unittest.main()
