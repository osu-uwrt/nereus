"""Negative tests for the pack loader: one textual mutation of a valid set per test."""

import sys
import tempfile
import unittest
from pathlib import Path

from robotics_platform.packs import PackError, load_pack, resolve_scenario
from test_packs_fixtures import BRIDGE, HOOP, POOL, ROBOT, SCENARIO, TASKS, write_generic_packs

HOOP_ASSET = "path: assets/hoop.dae, source: modelled for tests, required_from_step: 3"
THRUSTERS = "thrusters: {{order: {order}, input_unit: N, input_scales: {scales}, reject: []}}\n"
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
        (self.root / "tasks" / "assets").mkdir()
        try:
            (self.root / "tasks" / "assets" / "link").symlink_to(outside, target_is_directory=True)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        self.edit("tasks/tasks.yaml", "path: assets/hoop.dae", "path: assets/link/x.dae")
        self.assert_load_rejects("tasks", "escapes the pack")

    def test_present_asset_with_wrong_sha256(self) -> None:
        (self.root / "tasks" / "assets").mkdir()
        (self.root / "tasks" / "assets" / "hoop.dae").write_bytes(b"mesh")
        self.edit(
            "tasks/tasks.yaml",
            f"{HOOP_ASSET}, status: missing",
            f"{HOOP_ASSET}, status: present, sha256: '{'0' * 64}'",
        )
        self.assert_load_rejects("tasks", "sha256 mismatch")

    def test_present_asset_without_file(self) -> None:
        self.edit(
            "tasks/tasks.yaml",
            f"{HOOP_ASSET}, status: missing",
            f"{HOOP_ASSET}, status: present, sha256: '{'0' * 64}'",
        )
        self.assert_load_rejects("tasks", "is not a file")

    def test_missing_asset_whose_file_exists(self) -> None:
        (self.root / "tasks" / "assets").mkdir()
        (self.root / "tasks" / "assets" / "hoop.dae").write_bytes(b"mesh")
        self.assert_load_rejects("tasks", "marked missing but")


class HookRejectionTests(PackRejectionCase):
    HOOKS = "scoring_hooks: [{module: 'x', function: f, parameters: {}}]"

    def test_absent_hook_module_is_unresolved_not_an_error(self) -> None:
        self.edit("tasks/tasks.yaml", "scoring_hooks: []", self.HOOKS)
        load_pack(self.root / "tasks")
        resolved = resolve_scenario(self.scenario)
        hooks = [item for item in resolved.unresolved if item["kind"] == "hook_module"]
        self.assertEqual(len(hooks), 1)
        self.assertEqual(hooks[0]["id"], "x:f")
        self.assertIsNone(hooks[0]["required_from_step"])
        with self.assertRaises(PackError) as caught:
            resolve_scenario(self.scenario, strict=True)
        self.assertIn("hook_module 'x:f'", str(caught.exception))

    def test_hook_file_is_never_imported(self) -> None:
        self.edit(
            "tasks/tasks.yaml",
            "scoring_hooks: []",
            "scoring_hooks: [{module: 'never_imported_hook', function: f, parameters: {}}]",
        )
        marker = self.root / "tasks" / "imported.marker"
        (self.root / "tasks" / "never_imported_hook.py").write_text(
            f"from pathlib import Path\nPath({str(marker)!r}).write_text('x')\n"
            "raise SystemExit('hook imported')\n",
            encoding="utf-8",
        )
        load_pack(self.root / "tasks")
        resolved = resolve_scenario(self.scenario)
        self.assertFalse(marker.exists())
        self.assertNotIn("never_imported_hook", sys.modules)
        self.assertFalse([item for item in resolved.unresolved if item["kind"] == "hook_module"])
        self.assertIn(
            (self.root / "tasks" / "never_imported_hook.py").resolve(), resolved.sources
        )

    def test_hook_module_escaping_pack(self) -> None:
        self.edit(
            "tasks/tasks.yaml",
            "scoring_hooks: []",
            "scoring_hooks: [{module: '..outside', function: f, parameters: {}}]",
        )
        self.assert_load_rejects("tasks", "/scoring_hooks/0/module")

    def test_hook_module_resolving_through_symlink_out_of_pack(self) -> None:
        outside = Path(self._temporary.name) / "outside"
        outside.mkdir()
        (outside / "hook.py").write_text("raise SystemExit('imported')\n", encoding="utf-8")
        try:
            (self.root / "tasks" / "linked").symlink_to(outside, target_is_directory=True)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"symlinks unavailable: {error}")
        self.edit(
            "tasks/tasks.yaml",
            "scoring_hooks: []",
            "scoring_hooks: [{module: 'linked.hook', function: f, parameters: {}}]",
        )
        self.assert_load_rejects("tasks", "escapes the pack")


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
            SENSOR_LINE
            + "- id: cmd\n  direction: subscribe\n  topic: cmd\n"
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
            SENSOR_LINE
            + "placement:\n  estimator_alignment: {client: /align,"
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
        self.edit("tasks/hoop.yaml", "frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0]}\nregions",
                  "frame: task, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0], texture: ghost_png}\nregions")
        self.assert_resolve_rejects("unknown texture asset 'ghost_png'")

    def test_pass_through_sides_must_differ(self) -> None:
        self.edit("tasks/hoop.yaml", "to_side: negative", "to_side: positive")
        self.assert_resolve_rejects("from_side and to_side must differ")


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


class StrictCompletenessTests(PackRejectionCase):
    def test_valid_set_has_exactly_one_unresolved_asset(self) -> None:
        resolved = resolve_scenario(self.scenario)
        self.assertEqual(len(resolved.unresolved), 1)
        item = resolved.unresolved[0]
        self.assertEqual(item["kind"], "asset")
        self.assertEqual(item["id"], "hoop_mesh")
        self.assertEqual(item["required_from_step"], 3)

    def test_strict_rejects_unresolved_asset(self) -> None:
        with self.assertRaises(PackError) as caught:
            resolve_scenario(self.scenario, strict=True)
        self.assertIn("hoop_mesh", str(caught.exception))
        self.assertIn("step 3", str(caught.exception))

    def test_fixture_texts_match_written_files(self) -> None:
        self.assertEqual((self.root / "tasks/tasks.yaml").read_text(encoding="utf-8"), TASKS)
        self.assertEqual((self.root / "tasks/hoop.yaml").read_text(encoding="utf-8"), HOOP)
        self.assertEqual((self.root / "bridge/bridge.yaml").read_text(encoding="utf-8"), BRIDGE)
        self.assertEqual(
            (self.root / "scenario/scenario.yaml").read_text(encoding="utf-8"), SCENARIO
        )


if __name__ == "__main__":
    unittest.main()
