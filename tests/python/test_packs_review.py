"""Regressions for the loader review: includes, key/alias/newline fidelity, snapshot
integrity, native physical invariants and inactive metadata."""

import tempfile
import unittest
from pathlib import Path

from nereus.packs import PackError, load_pack, resolve_scenario
from test_packs_fixtures import write_generic_packs


class ReviewCase(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.scenario = write_generic_packs(self.root)

    def edit(self, relative: str, old: str, new: str) -> None:
        target = self.root / relative
        text = target.read_text()
        self.assertEqual(text.count(old), 1, old)
        target.write_text(text.replace(old, new))

    def rejects(self, folder: str, fragment: str) -> None:
        with self.assertRaises(PackError) as caught:
            load_pack(self.root / folder)
        self.assertIn(fragment, str(caught.exception))

    def resolve_rejects(self, fragment: str) -> None:
        with self.assertRaises(PackError) as caught:
            resolve_scenario(self.scenario)
        self.assertIn(fragment, str(caught.exception))


class StandaloneTasksTests(ReviewCase):
    def test_tasks_pack_validates_includes_without_scenario(self) -> None:
        document = load_pack(self.root / "tasks")
        self.assertEqual([item.kind for item in document.includes], ["task"])

    def test_missing_include_file(self) -> None:
        self.edit("tasks/tasks.yaml", "tasks: [hoop.yaml]", "tasks: [hoop.yaml, gone.yaml]")
        self.rejects("tasks", "include 'gone.yaml' does not exist")

    def test_include_problems_and_asset_references(self) -> None:
        self.edit("tasks/hoop.yaml", "asset: hoop_mesh", "asset: nothing")
        self.rejects("tasks", "unknown asset 'nothing'")


class FidelityTests(ReviewCase):
    def test_non_string_keys_are_rejected(self) -> None:
        for key in ("1", "true", "null"):
            with self.subTest(key=key):
                self.scenario = write_generic_packs(self.root)
                self.edit("robot/robot.yaml", "note: free", f"{key}: numeric, note: free")
                self.rejects("robot", "must be a string")

    def test_recursive_alias_is_a_pack_error(self) -> None:
        for loop in ("&loop {self: *loop}", "&loop [1, [*loop]]"):
            with self.subTest(loop=loop):
                self.scenario = write_generic_packs(self.root)
                self.edit("robot/robot.yaml", "metadata: {author: tests, note: free "
                          "non-executable annotation}", f"metadata: {loop}")
                self.rejects("robot", "recursive alias '*loop'")
        self.scenario = write_generic_packs(self.root)  # a plain (acyclic) alias stays valid
        self.edit("robot/robot.yaml", "metadata: {author: tests, note: free non-executable "
                  "annotation}", "metadata: {a: &shared [1, 2], b: *shared}")
        self.assertEqual(load_pack(self.root / "robot").plain()["metadata"]["b"], [1, 2])

    def test_crlf_bytes_survive_unedited_save(self) -> None:
        path = self.root / "pool" / "pool.yaml"
        original = path.read_bytes().replace(b"\n", b"\r\n")
        path.write_bytes(original)
        document = load_pack(self.root / "pool")
        self.assertEqual(document.save(self.root / "copy.yaml").read_bytes(), original)


class SnapshotTests(ReviewCase):
    def test_changed_source_is_detected_never_mixed(self) -> None:
        resolved = resolve_scenario(self.scenario)
        before = resolved.manifest()
        pool = (self.root / "pool" / "pool.yaml").resolve()
        pool.write_text(pool.read_text() + "# edited after resolution\n")
        self.assertEqual(resolved.changed_sources(), [pool])
        self.assertEqual(resolved.manifest(), before)  # captured digests, not live files
        with self.assertRaises(PackError) as caught:
            resolved.dump(self.root / "resolved.json")
        self.assertIn("changed since the scenario was resolved", str(caught.exception))


class PhysicalInvariantTests(ReviewCase):
    def test_negative_added_mass_diagonal(self) -> None:
        self.edit("robot/robot.yaml", "added_mass_matrix: [[5,", "added_mass_matrix: [[-5,")
        self.rejects("robot", "added_mass_matrix: must be positive semidefinite")

    def test_indefinite_linear_damping(self) -> None:
        self.edit("robot/robot.yaml", "linear_damping_matrix: [[10,", "linear_damping_matrix: [[-1,")
        self.rejects("robot", "linear_damping_matrix: must be positive semidefinite")

    def test_impossible_principal_moments(self) -> None:
        self.edit("robot/robot.yaml", "[0.0, 0.0, 1.2]]", "[0.0, 0.0, 3.0]]")
        self.rejects("robot", "triangle inequalities")

    def test_near_symmetry_uses_native_threshold(self) -> None:
        self.edit("robot/robot.yaml", "[[1.0, 0.0, 0.0], [0.0, 1.5",
                  "[[1.0, 0.000001, 0.0], [0.0, 1.5")
        self.rejects("robot", "inertia_matrix: must be symmetric")

    def test_direction_and_quaternion_tolerances(self) -> None:
        self.edit("robot/robot.yaml", "position_m: [-0.3, 0.2, 0], direction: [1, 0, 0]",
                  "position_m: [-0.3, 0.2, 0], direction: [1.00000001, 0, 0]")
        self.rejects("robot", "/thrusters/t0/direction: must have unit norm")
        self.scenario = write_generic_packs(self.root)
        self.edit("robot/robot.yaml", "child: imu_mount, position_m: [0.1, 0, 0], "
                  "orientation_wxyz: [1, 0, 0, 0]", "child: imu_mount, position_m: [0.1, 0, 0], "
                  "orientation_wxyz: [0.99999995, 0, 0, 0]")  # |q| - 1 = 5e-8 > 1e-8
        self.rejects("robot", "orientation_wxyz: must have unit norm")

    def test_sensor_period_shorter_than_timestep(self) -> None:
        self.edit("robot/robot.yaml", "rate_hz: 100", "rate_hz: 1000")
        self.resolve_rejects("rate_hz 1000 is faster than the 0.002 s physics step")

    def test_timestep_above_native_limit(self) -> None:
        self.edit("scenario/scenario.yaml", "timestep_s: 0.002", "timestep_s: 0.2")
        self.rejects("scenario", "timestep_s")


class InactiveMetadataTests(ReviewCase):
    def test_pose_like_inactive_data_is_not_a_runtime_pose(self) -> None:
        self.edit("robot/robot.yaml", "metadata: {author: tests,",
                  "metadata: {orientation_wxyz: [9, 9, 9, 9], author: tests,")
        load_pack(self.root / "robot")


if __name__ == "__main__":
    unittest.main()
