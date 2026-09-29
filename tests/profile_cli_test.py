"""End-to-end profile resolution and typed sensor CSV through the installed-style runner."""
import csv
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

EXECUTABLE, CONTENT = sys.argv[1:3]
del sys.argv[1:3]


class ProfileRunnerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        shutil.copytree(CONTENT, self.root / "content")
        self.scenario = self.root / "content/examples/profile_pool.yaml"

    def run_profile(self, output=None):
        args = [EXECUTABLE, str(self.scenario)]
        if output is not None:
            args += ["--sensors", str(output)]
        return subprocess.run(args, cwd=self.root, capture_output=True, text=True, timeout=60)

    def test_complete_run_and_replay_preserve_measurements_and_timing(self):
        output = self.root / "sensors.csv"
        first = self.run_profile(output)
        self.assertEqual(first.returncode, 0, first.stderr)
        first_csv = output.read_text()
        second = self.run_profile(output)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual(first.stdout, second.stdout)
        self.assertEqual(first_csv, output.read_text())
        silent = self.run_profile()
        self.assertEqual(silent.returncode, 0, silent.stderr)
        self.assertEqual(first.stdout, silent.stdout)
        rows = list(csv.DictReader(io.StringIO(first_csv)))
        self.assertEqual({row["device"] for row in rows}, {"imu", "fog", "dvl", "pressure"})
        samples = {(row["device"], row["sequence"]) for row in rows}
        self.assertEqual(len(samples), 300 + 150 + 29 + 60)
        for row in rows:
            self.assertEqual(row["valid"], "1")
            self.assertGreaterEqual(int(row["delivered_ns"]), int(row["acquired_ns"]))
            if row["device"] == "dvl":
                self.assertEqual(int(row["delivered_ns"]) - int(row["acquired_ns"]), 20000000)
        depths = [float(row["value"]) for row in rows if row["device"] == "pressure" and row["field"] == "depth"]
        self.assertEqual(len(depths), 60)
        self.assertAlmostEqual(depths[0], 2.1, delta=0.01)
        self.assertEqual(len(list(csv.DictReader(io.StringIO(first.stdout)))), 1501)

    def test_composed_attitude_csv_uses_one_acquisition_header(self):
        profile = self.root / "content/sensors/imu.yaml"
        profile.write_text("schema_version: 1\nkind: sensor\nmodel: ahrs\nparameters:\n"
                           "  inertial: {}\n  attitude: {heading_drift_rad_s: 0.2}\n")
        output = self.root / "sensors.csv"
        result = self.run_profile(output)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [r for r in csv.DictReader(io.StringIO(output.read_text())) if r["device"] == "imu"]
        first = [r for r in rows if r["sequence"] == "0"]
        self.assertEqual(len(first), 37)
        self.assertEqual({r["acquired_ns"] for r in first}, {"10000000"})
        self.assertEqual({r["tick"] for r in first}, {"5"})
        self.assertIn("orientation.w", {r["field"] for r in first})
        self.assertIn("specific_force.z", {r["field"] for r in first})
        self.assertEqual(self.run_profile().returncode, 0)

    def test_native_talos_navigation_reports_all_samples_without_range(self):
        self.scenario = self.root / "content/examples/talos_navigation_pool.yaml"
        output = self.root / "sensors.csv"
        result = self.run_profile(output)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(output.read_text())))
        for name, count in (("imu", 150), ("fog", 1500), ("dvl", 24)):
            device = [r for r in rows if r["device"] == name]
            self.assertEqual(len({r["sequence"] for r in device}), count)
            self.assertEqual({r["valid"] for r in device}, {"1"})
        dvl = [r for r in rows if r["device"] == "dvl"]
        self.assertIn("reference_relative_velocity.z", {r["field"] for r in dvl})
        self.assertNotIn("bottom_distance", {r["field"] for r in dvl})

    def test_invalid_nested_profile_fails_before_creating_output(self):
        profile = self.root / "content/sensors/imu.yaml"
        profile.write_text(profile.read_text().replace("white_stddev:", "unknown:", 1))
        output = self.root / "sensors.csv"
        result = self.run_profile(output)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertFalse(output.exists())
        self.assertIn("sensors/imu.yaml", result.stderr)

    def test_output_cannot_overwrite_a_referenced_profile(self):
        profile = self.root / "content/robots/synthetic_auv.yaml"
        original = profile.read_text()
        result = self.run_profile(profile)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "")
        self.assertEqual(profile.read_text(), original)
        self.assertIn("overwrite", result.stderr)

    def test_missing_bottom_is_recorded_as_unavailable_and_ids_are_escaped(self):
        robot = self.root / "content/robots/synthetic_auv.yaml"
        robot.write_text(robot.read_text().replace("maximum_range_m: 10", "maximum_range_m: 1")
                         .replace("id: pressure", "id: 'pressure,\"depth\"'"))
        output = self.root / "sensors.csv"
        result = self.run_profile(output)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = list(csv.DictReader(io.StringIO(output.read_text())))
        self.assertIn('pressure,"depth"', {row["device"] for row in rows})
        unavailable = [row for row in rows if row["device"] == "dvl"]
        self.assertEqual(len(unavailable), 29)
        for row in unavailable:
            self.assertEqual(row["valid"], "0")
            self.assertEqual(row["reason"], "bottom out of range")
            self.assertEqual(row["value"], "")


if __name__ == "__main__":
    unittest.main()
