"""Behavior checks against the real runner; no ROS or third-party Python packages."""
import csv
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXECUTABLE, SCENARIO = sys.argv[1:3]
del sys.argv[1:3]


class RunnerTests(unittest.TestCase):
    def run_text(self, text):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "scenario.yaml"
            path.write_text(text)
            return subprocess.run([EXECUTABLE, str(path)], capture_output=True, text=True, timeout=30)

    def test_example_repeatability_and_timestamps(self):
        text = Path(SCENARIO).read_text()
        first = self.run_text(text)
        second = self.run_text(text)
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(second.returncode, 0, second.stderr)
        self.assertEqual(first.stdout, second.stdout)
        rows = list(csv.DictReader(io.StringIO(first.stdout)))
        self.assertEqual(len(rows), 1501)
        for tick, row in enumerate(rows):
            self.assertEqual(int(row["tick"]), tick)
            self.assertEqual(int(row["time_ns"]), tick * 2_000_000)
        self.assertGreater(float(rows[-1]["x_m"]), 3)
        self.assertAlmostEqual(float(rows[-1]["z_m"]), -2)

    def test_invalid_scenarios_fail_before_output(self):
        original = Path(SCENARIO).read_text()
        mutations = [
            ("schema_version: 1", "schema_version: 99", "schema_version"),
            ("ticks: 1500", "ticks: -1", "ticks"),
            ("ticks: 1500", "ticks: 18446744073709551616", "ticks"),
            ("timestep_ns: 2000000", "timestep_ns: 0", "timestep_ns"),
            ("mass_kg: 10", "mass_kg: .nan", "mass_kg"),
            ("mass_kg: 10", "mass_kg: 10\n  mass_kg: 11", "duplicate"),
            ("mass_kg: 10", "mass_kg: 10\n  mystery: 11", "mystery"),
            ("direction: [1, 0, 0]", "direction: [1, 0]", "direction"),
            ("direction: [1, 0, 0]", "direction: [0, 0, 0]", "unit vector"),
            ("tick: 200", "tick: 0", "increase strictly"),
            ("tick: 1000", "tick: 1500", "less than ticks"),
            ("forces_n: [10, 10, 0, 0]", "forces_n: [10]", "forces_n"),
            ("horizontal_starboard", "horizontal_port", "unique"),
        ]
        for old, new, expected in mutations:
            with self.subTest(expected=expected):
                result = self.run_text(original.replace(old, new, 1))
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, "")
                self.assertIn("scenario.yaml", result.stderr)
                self.assertIn(expected, result.stderr)

    def test_usage(self):
        result = subprocess.run([EXECUTABLE], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        result = subprocess.run([EXECUTABLE, "--help"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertIn("Usage:", result.stdout)


if __name__ == "__main__":
    unittest.main()
