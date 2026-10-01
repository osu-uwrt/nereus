"""Negative tests for the pack loader: one textual mutation of a valid set per test."""

import tempfile
import unittest
from pathlib import Path

from nereus.packs import PackError, load_pack, resolve_scenario
from test_packs_fixtures import BRIDGE, HOOP, POOL, ROBOT, SCENARIO, TASKS, write_generic_packs

THRUSTERS = "thrusters: {{order: {order}, input_scales: {scales}, reject: []}}\n"
SENSOR_LINE = "  fields: {data: {from: reading.target_world_z}}\n"


def mutate(text: str, old: str, new: str) -> str:
    """Replace one substring, failing loudly if it is not present exactly once."""
    assert text.count(old) == 1, f"expected exactly one {old!r}, found {text.count(old)}"
    return text.replace(old, new)


class PackRejectionCase(unittest.TestCase):
    """Base: a fresh valid generic pack set in a temporary directory."""

    def setUp(self) -> None:
        self._temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary.cleanup)
        self.root = Path(self._temporary.name) / "packs"
        self.root.mkdir()
        self.scenario = write_generic_packs(self.root)

    def edit(self, relative: str, old: str, new: str) -> Path:
        target = self.root / relative
        target.write_text(mutate(target.read_text(encoding="utf-8"), old, new), encoding="utf-8")
        return target

    def assert_load_rejects(self, folder: str, fragment: str) -> PackError:
        with self.assertRaises(PackError) as caught:
            load_pack(self.root / folder)
        self.assertIn(fragment, str(caught.exception))
        return caught.exception

    def assert_resolve_rejects(self, fragment: str) -> PackError:
        with self.assertRaises(PackError) as caught:
            resolve_scenario(self.scenario)
        self.assertIn(fragment, str(caught.exception))
        return caught.exception


class ValidBaselineTests(PackRejectionCase):
    def test_unmutated_set_is_valid(self) -> None:
        for folder in ("robot", "pool", "tasks", "bridge", "scenario"):
            load_pack(self.root / folder)
        resolve_scenario(self.scenario)

    def test_mutate_helper_rejects_missing_and_repeated_text(self) -> None:
        with self.assertRaises(AssertionError):
            mutate(ROBOT, "no such text anywhere", "x")
        with self.assertRaises(AssertionError):
            mutate(ROBOT, "type: lagged_force", "x")


class RobotRejectionTests(PackRejectionCase):
    def test_duplicate_mapping_key(self) -> None:
        self.edit(
            "robot/robot.yaml", "kind: robot\nid: synth\n", "kind: robot\nid: synth\nid: synth\n"
        )
        self.assert_load_rejects("robot", "duplicate key")

    def test_infinite_number(self) -> None:
        self.edit("robot/robot.yaml", "mass_kg: 20.0", "mass_kg: .inf")
        self.assert_load_rejects("robot", "non-finite number")

    def test_nan_number(self) -> None:
        self.edit("robot/robot.yaml", "mass_kg: 20.0", "mass_kg: .nan")
        self.assert_load_rejects("robot", "non-finite number")

    def test_unknown_root_field(self) -> None:
        self.edit(
            "robot/robot.yaml", "kind: robot\nid: synth\n", "kind: robot\nid: synth\ncolour: red\n"
        )
        self.assert_load_rejects("robot", "'colour' was unexpected")

    def test_unknown_sensor_parameter_field(self) -> None:
        self.edit(
            "robot/robot.yaml",
            "    attitude: {angle_stddev_rad: 0.001}\n",
            "    attitude: {angle_stddev_rad: 0.001}\n    colour: red\n",
        )
        self.assert_load_rejects("robot", "'colour' was unexpected")

    def test_unknown_sensor_type(self) -> None:
        self.edit("robot/robot.yaml", "type: ahrs", "type: sonar")
        self.assert_load_rejects("robot", "'sonar' is not one of")

    def test_non_unit_frame_quaternion(self) -> None:
        self.edit(
            "robot/robot.yaml",
            "child: base_link, position_m: [0, 0, -0.05], orientation_wxyz: [1, 0, 0, 0]",
            "child: base_link, position_m: [0, 0, -0.05], orientation_wxyz: [0.9, 0, 0, 0]",
        )
        self.assert_load_rejects("robot", "/frames/transforms/0/orientation_wxyz: must have unit")

    def test_non_unit_thruster_direction(self) -> None:
        self.edit(
            "robot/robot.yaml",
            "position_m: [-0.3, 0.2, 0], direction: [1, 0, 0]",
            "position_m: [-0.3, 0.2, 0], direction: [2, 0, 0]",
        )
        self.assert_load_rejects("robot", "/thrusters/t0/direction: must have unit norm")

    def test_sensor_mount_frame_missing(self) -> None:
        self.edit("robot/robot.yaml", "mount_frame: imu_mount", "mount_frame: ghost_mount")
        self.assert_load_rejects("robot", "unknown frame 'ghost_mount'")

    def test_frame_tree_cycle(self) -> None:
        self.edit(
            "robot/robot.yaml",
            "{parent: com, child: base_link",
            "{parent: imu_mount, child: base_link",
        )
        self.assert_load_rejects("robot", "cycle in frame tree")


class AssetRejectionTests(PackRejectionCase):
    def test_asset_path_escaping_pack(self) -> None:
        self.edit("tasks/tasks.yaml", "path: assets/hoop.dae", "path: ../outside.dae")
        error = self.assert_load_rejects("tasks", "/assets/0/path")
        self.assertIn("does not match", str(error))

    def test_asset_path_escaping_pack_through_symlink(self) -> None:
        outside = Path(self._temporary.name) / "outside"
        outside.mkdir()
        (self.root / "tasks" / "assets").mkdir(exist_ok=True)
        try:
            (self.root / "tasks" / "assets" / "link").symlink_to(outside, target_is_directory=True)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        self.edit("tasks/tasks.yaml", "path: assets/hoop.dae", "path: assets/link/x.dae")
        self.assert_load_rejects("tasks", "escapes the pack")

    def test_asset_without_file(self) -> None:
        (self.root / "tasks" / "assets" / "hoop.dae").unlink()
        self.assert_load_rejects("tasks", "is not a file")


class ScoringRulesRejectionTests(PackRejectionCase):
    def test_rules_entry_needs_a_name(self) -> None:
        self.edit("tasks/tasks.yaml", "scoring_rules: []", "scoring_rules: [{parameters: {}}]")
        self.assert_load_rejects("tasks", "'name' is a required property")

    def test_rules_entry_rejects_python_hook_fields(self) -> None:
        self.edit(
            "tasks/tasks.yaml",
            "scoring_rules: []",
            "scoring_rules: [{name: practice, module: rules, parameters: {}}]",
        )
        self.assert_load_rejects("tasks", "'module' was unexpected")


class ScenarioRejectionTests(PackRejectionCase):
    PLACEMENT = "- {task: hoop, position_m: [5, 2.5, -1], yaw_deg: 90}\n"

    def test_unknown_task_placement(self) -> None:
        self.edit(
            "scenario/scenario.yaml",
            self.PLACEMENT,
            self.PLACEMENT + "- {task: ghost, position_m: [0, 0, 0], yaw_deg: 0}\n",
        )
        self.assert_resolve_rejects("unknown task 'ghost'")

    def test_defined_task_without_placement(self) -> None:
        self.edit("scenario/scenario.yaml", "task: hoop, position_m", "task: ghost, position_m")
        self.assert_resolve_rejects("task 'hoop' has no placement")

    def test_unknown_run_option(self) -> None:
        self.edit("scenario/scenario.yaml", "options: {}", "options: {speed: true}")
        self.assert_resolve_rejects("/run/options/speed: unknown run option")

    def test_bad_run_option_type(self) -> None:
        self.edit("scenario/scenario.yaml", "options: {}", "options: {timed: 'yes'}")
        self.assert_resolve_rejects("/run/options/timed: must be a boolean")

    def test_run_option_number_rejected_by_schema(self) -> None:
        self.edit("scenario/scenario.yaml", "options: {}", "options: {timed: 1}")
        self.assert_resolve_rejects("/run/options/timed: 1 is not of type")

    def test_requires_more_droppers_than_robot_has(self) -> None:
        self.edit("tasks/tasks.yaml", "min_count: 1", "min_count: 2")
        self.assert_resolve_rejects("requires 2 dropper mechanism(s); robot has 1")

    def test_unknown_initial_frame(self) -> None:
        self.edit("scenario/scenario.yaml", "initial: {frame: com", "initial: {frame: ghost")
        self.assert_resolve_rejects("unknown robot frame 'ghost'")


class BridgeRejectionTests(PackRejectionCase):
    def test_native_sensor_unknown(self) -> None:
        self.edit("bridge/bridge.yaml", "'sensor:altitude'", "'sensor:nope'")
        self.assert_resolve_rejects("unknown robot sensor 'nope'")

    def test_subscribe_stream_with_nonzero_rate(self) -> None:
        self.edit(
            "bridge/bridge.yaml",
            SENSOR_LINE,
            SENSOR_LINE + "- id: cmd\n  direction: subscribe\n  topic: cmd\n"
            "  message_type: std_msgs/msg/Float64\n  native: 'command:mechanisms.marker.drop'\n"
            "  frame_id: ''\n  rate_hz: 5\n"
            "  qos: {history: keep_last, depth: 10, reliability: reliable, durability: volatile}\n"
            "  fields: {data: {from: data}}\n",
        )
        error = self.assert_load_rejects("bridge", "/streams/1")
        self.assertIn("0 was expected", str(error))

    def test_publish_stream_with_accept_if(self) -> None:
        self.edit(
            "bridge/bridge.yaml",
            "  rate_hz: 20\n",
            "  rate_hz: 20\n  accept_if: [{field: data, equals: 1}]\n",
        )
        self.assert_load_rejects("bridge", "/streams/0")

    def test_kill_referencing_missing_stream(self) -> None:
        self.edit(
            "bridge/bridge.yaml",
            SENSOR_LINE,
            SENSOR_LINE + "kill: {command_stream: nope, state_stream: altitude}\n",
        )
        self.assert_load_rejects("bridge", "/kill/command_stream: unknown stream 'nope'")

    def test_kill_state_stream_missing(self) -> None:
        self.edit(
            "bridge/bridge.yaml",
            SENSOR_LINE,
            SENSOR_LINE + "kill: {command_stream: altitude, state_stream: nope}\n",
        )
        self.assert_load_rejects("bridge", "unknown stream 'nope'")

    def test_estimator_alignment_referencing_missing_stream(self) -> None:
        self.edit(
            "bridge/bridge.yaml",
            SENSOR_LINE,
            SENSOR_LINE + "placement:\n  estimator_alignment: {client: /align,"
            " service_type: std_srvs/srv/Trigger,"
            " triggers: [startup], estimate_stream: nope, pose: reference_frame,"
            " covariance_diagonal: 1.0}\n",
        )
        self.assert_load_rejects("bridge", "estimate_stream: unknown stream 'nope'")

    def test_thruster_order_not_a_permutation(self) -> None:
        block = THRUSTERS.format(order="[t0, t1, t2, t9]", scales="[1, 1, 1, 1]")
        self.edit("bridge/bridge.yaml", SENSOR_LINE, SENSOR_LINE + block)
        load_pack(self.root / "bridge")
        self.assert_resolve_rejects("must be a permutation of the robot thruster ids")

    def test_thruster_order_missing_a_thruster(self) -> None:
        block = THRUSTERS.format(order="[t0, t1, t2]", scales="[1, 1, 1]")
        self.edit("bridge/bridge.yaml", SENSOR_LINE, SENSOR_LINE + block)
        self.assert_resolve_rejects("must be a permutation of the robot thruster ids")

    def test_thruster_scales_length_mismatch(self) -> None:
        block = THRUSTERS.format(order="[t0, t1, t2, t3]", scales="[1, 1]")
        self.edit("bridge/bridge.yaml", SENSOR_LINE, SENSOR_LINE + block)
        self.assert_load_rejects("bridge", "order and input_scales must have equal length")

    def test_field_path_expression(self) -> None:
        self.edit("bridge/bridge.yaml", "{from: reading.target_world_z}", "{from: 'a+b'}")
        self.assert_load_rejects("bridge", "/streams/0/fields/data")


class TaskRejectionTests(PackRejectionCase):
    def test_pass_through_event_names_missing_region(self) -> None:
        self.edit("tasks/hoop.yaml", "region: opening, from_side", "region: ghost, from_side")
        self.assert_resolve_rejects("must name a rectangular_portal region")

    def test_event_points_on_unknown_event(self) -> None:
        self.edit("tasks/hoop.yaml", "event: through, points", "event: ghost, points")
        self.assert_resolve_rejects("unknown event 'ghost'")

    def test_visual_referencing_unknown_asset(self) -> None:
        self.edit("tasks/hoop.yaml", "asset: hoop_mesh", "asset: ghost_mesh")
        self.assert_resolve_rejects("unknown asset 'ghost_mesh'")

    def test_visual_texture_must_be_a_declared_asset(self) -> None:
        self.edit(
            "tasks/hoop.yaml",
            "frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}\nregions",
            "frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0], texture: ghost_png}\nregions",
        )
        self.assert_resolve_rejects("unknown texture asset 'ghost_png'")

    def test_pass_through_sides_must_differ(self) -> None:
        self.edit("tasks/hoop.yaml", "to_side: negative", "to_side: positive")
        self.assert_resolve_rejects("from_side and to_side must differ")


class PoolMarkingTests(PackRejectionCase):
    """The fixture pool is 10 m x 5 m x 3 m deep with a 0.3 m deck."""

    COLLISION = "collision_boxes:\n"

    def add_markings(self, markings: str) -> None:
        self.edit("pool/pool.yaml", self.COLLISION, f"markings:\n{markings}{self.COLLISION}")

    def test_irregular_lines_grid_and_finish_load(self) -> None:
        self.add_markings(
            "  width_m: 0.2\n"
            "  lane_grid: {along_x: {count: 2, spacing_m: 1.5, first_m: 0.5}, inset_m: 1, ends: t}\n"
            "  lines:\n"
            "  - {from: [1, 1], to: [9, 4], ends: [t, none], color_rgb: [1, 0, 0]}\n"
            "  wall_lines:\n"
            "  - {wall: y_max, from: [2, -3], to: [2, 0.3]}\n"
        )
        self.edit(
            "pool/pool.yaml",
            self.COLLISION,
            "surface: {tile_size_m: 0, waterline_band_m: [0, 0]}\n" + self.COLLISION,
        )
        load_pack(self.root / "pool")
        resolve_scenario(self.scenario)

    def test_floor_line_outside_the_pool(self) -> None:
        self.add_markings("  lines: [{from: [1, 1], to: [11, 1]}]\n")
        self.assert_load_rejects("pool", "/markings/lines/0/to: x 11 m is outside 0..10 m")

    def test_zero_length_line(self) -> None:
        self.add_markings("  lines: [{from: [1, 1], to: [1, 1]}]\n")
        self.assert_load_rejects("pool", "/markings/lines/0: from and to must differ")

    def test_wall_line_above_the_deck(self) -> None:
        self.add_markings("  wall_lines: [{wall: x_min, from: [1, -3], to: [1, 0.5]}]\n")
        self.assert_load_rejects("pool", "/markings/wall_lines/0/to: z 0.5 m is outside -3..0.3 m")

    def test_lane_grid_wider_than_the_pool(self) -> None:
        self.add_markings("  lane_grid: {along_x: {count: 4, spacing_m: 2}}\n")
        self.assert_load_rejects("pool", "/markings/lane_grid/along_x: lines at -0.5..5.5 m")

    def test_lane_grid_inset_leaves_no_line(self) -> None:
        self.add_markings("  lane_grid: {along_y: {count: 2, spacing_m: 2}, inset_m: 2.5}\n")
        self.assert_load_rejects("pool", "inset_m 2.5 leaves no line on a 5 m floor")

    def test_lane_grid_needs_a_family(self) -> None:
        self.add_markings("  lane_grid: {inset_m: 1}\n")
        self.assert_load_rejects("pool", "/markings/lane_grid")

    def test_unknown_end_style(self) -> None:
        self.add_markings("  ends: arrow\n")
        self.assert_load_rejects("pool", "/markings/ends")


class PoolFloorProfileTests(PackRejectionCase):
    """The fixture pool is 10 m long, 3 m deep; profiled here to rise to 2 m at the far end."""

    FLOOR_BOX = "- {id: floor, size_m: [10, 5, 1], center_m: [5, 2.5, -3.5], orientation_wxyz: [1, 0, 0, 0]}\n"
    WALL_BOX = "- {id: end_wall, size_m: [1, 5, 4], center_m: [10.5, 2.5, -1], orientation_wxyz: [1, 0, 0, 0]}\n"

    def profile(self, points: str = "[[0, 3], [4, 3], [7, 2], [10, 2]]", along: str = "x") -> None:
        self.edit(
            "pool/pool.yaml",
            "current_oscillation_frequency_hz: 0}",
            f"current_oscillation_frequency_hz: 0, floor_profile: {{along: {along}, points_m: {points}}}}}",
        )
        self.edit("pool/pool.yaml", self.FLOOR_BOX, self.WALL_BOX)

    def test_profiled_floor_loads_and_resolves(self) -> None:
        self.profile()
        self.edit(
            "pool/pool.yaml",
            "collision_boxes:\n",
            "markings:\n"
            "  wall_lines:\n"
            "  - {wall: x_min, from: [1, -3], to: [1, 0]}\n"
            "  - {wall: x_max, from: [1, -2], to: [1, 0]}\n"
            "  - {wall: y_min, from: [2, -3], to: [8, -2]}\n"
            "collision_boxes:\n",
        )
        load_pack(self.root / "pool")
        resolve_scenario(self.scenario)

    def test_profile_must_span_the_pool(self) -> None:
        self.profile("[[0, 3], [8, 2]]")
        self.assert_load_rejects("pool", "must run from 0 to 10 m along x, not 0..8 m")

    def test_profile_along_y_spans_the_width(self) -> None:
        self.profile("[[0, 3], [10, 2]]", along="y")
        self.assert_load_rejects("pool", "must run from 0 to 5 m along y")

    def test_profile_positions_increase(self) -> None:
        self.profile("[[0, 3], [6, 3], [5, 2], [10, 2]]")
        self.assert_load_rejects("pool", "positions must increase")

    def test_profile_depths_positive(self) -> None:
        self.profile("[[0, 3], [5, 0], [10, 3]]")
        self.assert_load_rejects("pool", "depths must be positive")

    def test_depth_is_the_deepest_point(self) -> None:
        self.profile("[[0, 2.5], [10, 2]]")
        self.assert_load_rejects("pool", "3 m must be the floor's deepest point (2.5 m)")

    def test_flat_floor_box_is_rejected(self) -> None:
        self.profile()
        self.edit("pool/pool.yaml", self.WALL_BOX, self.WALL_BOX + self.FLOOR_BOX)
        self.assert_load_rejects("pool", "'floor' is a flat floor box")

    def test_wall_line_below_the_shallow_end(self) -> None:
        self.profile()
        self.edit(
            "pool/pool.yaml",
            "collision_boxes:\n",
            "markings: {wall_lines: [{wall: x_max, from: [1, -3], to: [1, 0]}]}\ncollision_boxes:\n",
        )
        self.assert_load_rejects("pool", "/markings/wall_lines/0/from: z -3 m is outside -2..0.3 m")

    def test_profile_list_rising_toward_two_walls(self) -> None:
        self.profile("[[0, 3], [4, 3], [7, 2], [10, 2]]}, {along: y, points_m: [[0, 3], [3, 3], [5, 2.5]]")
        self.edit("pool/pool.yaml", "floor_profile: {along: x", "floor_profile: [{along: x")
        self.edit("pool/pool.yaml", "[5, 2.5]]}", "[5, 2.5]]}]")
        self.edit(
            "pool/pool.yaml",
            "collision_boxes:\n",
            "markings: {wall_lines: [{wall: y_max, from: [2, -2.5], to: [2, 0]}]}\ncollision_boxes:\n",
        )
        load_pack(self.root / "pool")
        resolve_scenario(self.scenario)

    def test_profile_list_entries_are_checked(self) -> None:
        self.profile("[[0, 3], [10, 2]]}, {along: y, points_m: [[0, 3], [4, 2.5]]")
        self.edit("pool/pool.yaml", "floor_profile: {along: x", "floor_profile: [{along: x")
        self.edit("pool/pool.yaml", "[4, 2.5]]}", "[4, 2.5]]}]")
        self.assert_load_rejects("pool", "/parameters/floor_profile/1/points_m: must run from 0 to 5 m along y")

    def test_wall_line_below_the_risen_side(self) -> None:
        self.profile("[[0, 3], [10, 3]]}, {along: y, points_m: [[0, 3], [3, 3], [5, 2.5]]")
        self.edit("pool/pool.yaml", "floor_profile: {along: x", "floor_profile: [{along: x")
        self.edit("pool/pool.yaml", "[5, 2.5]]}", "[5, 2.5]]}]")
        self.edit(
            "pool/pool.yaml",
            "collision_boxes:\n",
            "markings: {wall_lines: [{wall: y_max, from: [2, -3], to: [2, 0]}]}\ncollision_boxes:\n",
        )
        self.assert_load_rejects("pool", "/markings/wall_lines/0/from: z -3 m is outside -2.5..0.3 m")

    def test_sphere_pool_contacts_need_a_flat_floor(self) -> None:
        self.profile()
        self.edit("scenario/scenario.yaml", "model: box_scene", "model: sphere_pool")
        self.assert_resolve_rejects("sphere_pool needs a flat pool floor")


class FolderRejectionTests(PackRejectionCase):
    def test_folder_with_two_canonical_files(self) -> None:
        (self.root / "robot" / "pool.yaml").write_text(POOL, encoding="utf-8")
        self.assert_load_rejects("robot", "exactly one of")

    def test_folder_canonical_file_declares_another_kind(self) -> None:
        self.edit("robot/robot.yaml", "kind: robot\n", "kind: pool\n")
        self.assert_load_rejects("robot", "declares kind 'pool'")

    def test_folder_without_canonical_file(self) -> None:
        (self.root / "empty").mkdir()
        self.assert_load_rejects("empty", "exactly one of")

    def test_scenario_selecting_wrong_pack_kind(self) -> None:
        self.edit("scenario/scenario.yaml", "robot: ../robot", "robot: ../pool")
        self.assert_resolve_rejects("selects a 'pool' pack")


class FixtureTests(PackRejectionCase):
    def test_valid_set_resolves(self) -> None:
        self.assertEqual(resolve_scenario(self.scenario).robot["id"], "synth")

    def test_fixture_texts_match_written_files(self) -> None:
        self.assertEqual((self.root / "tasks/tasks.yaml").read_text(encoding="utf-8"), TASKS)
        self.assertEqual((self.root / "tasks/hoop.yaml").read_text(encoding="utf-8"), HOOP)
        self.assertEqual((self.root / "bridge/bridge.yaml").read_text(encoding="utf-8"), BRIDGE)
        self.assertEqual(
            (self.root / "scenario/scenario.yaml").read_text(encoding="utf-8"), SCENARIO
        )


if __name__ == "__main__":
    unittest.main()
