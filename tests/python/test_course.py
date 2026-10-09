"""Scenario course edits: task_frames (loose objects) when resolving, and set_course's validated lossless save."""

import contextlib
import io
import json
import os
import re
import tempfile
import unittest
from pathlib import Path
from typing import Any

from nereus.packs import PackError, load_pack, resolve_scenario, set_course
from nereus.packs.__main__ import main

PACKS = Path(__file__).resolve().parents[2] / "content" / "packs"
TALOS = PACKS / "scenarios" / "talos_uwrt"


def talos_copy(folder: Path) -> Path:
    """The Talos scenario written into `folder`, its pack paths re-pointed at the shipped packs."""
    folder.mkdir(parents=True, exist_ok=True)

    def repoint(match: re.Match[str]) -> str:
        target = (TALOS / match.group(2)).resolve()
        return f"{match.group(1)}: {os.path.relpath(target, folder)}"

    text = (TALOS / "scenario.yaml").read_text()
    text = re.sub(r"^(robot|pool|tasks|bridge|equipment): (\S+)", repoint, text, flags=re.M)
    (folder / "scenario.yaml").write_text(text)
    return folder / "scenario.yaml"


def frame(resolved_task: dict[str, Any], name: str) -> dict[str, Any]:
    found: dict[str, Any] = next(item for item in resolved_task["frames"] if item["id"] == name)
    return found


class CourseTests(unittest.TestCase):
    def setUp(self) -> None:
        self._directory = tempfile.TemporaryDirectory()
        self.addCleanup(self._directory.cleanup)
        self.scenario = talos_copy(Path(self._directory.name) / "talos")

    def resolved(self) -> dict[str, Any]:
        resolved = resolve_scenario(self.scenario)
        return {
            "placements": {item["task"]: item for item in resolved.scenario["task_placements"]},
            "tasks": {item["id"]: item for item in resolved.task_definitions},
            "options": resolved.run_options,
        }

    def test_set_course_moves_tasks_objects_and_options(self) -> None:
        set_course(
            self.scenario,
            {
                "task_placements": [
                    {"task": "gate", "position_m": [5, -2, -0.75], "yaw_deg": 190.0}
                ],
                "task_frames": [
                    {
                        "task": "table",
                        "frame": "plug",
                        "position_m": [0.15, -0.13, 0.03],
                        "yaw_deg": 90,
                    }
                ],
                "run_options": {"bin_vinyl1_class": "fire"},
            },
        )
        after = self.resolved()
        self.assertEqual(after["placements"]["gate"]["position_m"], [5.0, -2.0, -0.75])
        self.assertEqual(after["placements"]["gate"]["yaw_deg"], -170.0)  # wrapped
        plug = frame(after["tasks"]["table"], "plug")
        self.assertEqual(plug["position_m"], [0.15, -0.13, 0.03])
        self.assertAlmostEqual(plug["orientation_wxyz"][0], 2**-0.5)
        self.assertAlmostEqual(plug["orientation_wxyz"][3], 2**-0.5)
        crate = next(r for r in after["tasks"]["bins"]["regions"] if r["id"] == "bin_vinyl1")
        self.assertEqual(crate["parameters"]["class"], "fire")

        # Comments, key order and the untouched placements stay as authored; the file round-trips byte-exact
        text = self.scenario.read_text()
        self.assertTrue(text.startswith("# Talos on the RoboSub 2026 course."))
        self.assertIn("# The octagon sits above the table.\n", text)
        self.assertIn("- {task: gate, position_m: [5.0, -2.0, -0.75], yaw_deg: -170.0}\n", text)
        self.assertIn(
            "- {task: torpedo, position_m: [18.35222, 2.480545, -1.372], yaw_deg: -169.835651}",
            text,
        )
        self.assertIn(
            "task_frames:\n- {task: table, frame: plug, position_m: [0.15, -0.13, 0.03], yaw_deg: 90.0}\n",
            text,
        )
        self.assertLess(text.index("task_placements:"), text.index("task_frames:"))
        self.assertLess(text.index("task_frames:"), text.index("contacts:"))
        self.assertEqual(load_pack(self.scenario).dumps(), text)

    def test_a_loose_object_moved_again_replaces_its_entry(self) -> None:
        for x in (0.1, 0.2):
            set_course(
                self.scenario,
                {
                    "task_frames": [
                        {"task": "table", "frame": "pill", "position_m": [x, 0, 0.05], "yaw_deg": 0}
                    ]
                },
            )
        self.assertEqual(self.scenario.read_text().count("frame: pill"), 1)
        self.assertEqual(
            frame(self.resolved()["tasks"]["table"], "pill")["position_m"], [0.2, 0.0, 0.05]
        )

    def test_bad_edits_are_not_written(self) -> None:
        before = self.scenario.read_text()
        cases: list[tuple[dict[str, Any], str]] = [
            # only loose objects move
            (
                {
                    "task_frames": [
                        {
                            "task": "table",
                            "frame": "helmet_basket",
                            "position_m": [0, 0, 0],
                            "yaw_deg": 0,
                        }
                    ]
                },
                "'helmet_basket' is not a loose object of task 'table'",
            ),
            (
                {
                    "task_frames": [
                        {"task": "ghost", "frame": "x", "position_m": [0, 0, 0], "yaw_deg": 0}
                    ]
                },
                "unknown task 'ghost'",
            ),
            ({"run_options": {"bin_vinyl1_class": "ink"}}, "must be one of ['blood', 'fire']"),
            (
                {"task_placements": [{"task": "ghost", "position_m": [0, 0, 0], "yaw_deg": 0}]},
                "task 'ghost' has no placement",
            ),
            ({"task_placements": [{"task": "gate"}]}, "needs position_m"),
        ]
        for course, message in cases:
            with self.assertRaises(PackError) as caught:
                set_course(self.scenario, course)
            self.assertIn(message, str(caught.exception))
            self.assertNotIn(
                ".scenario.yaml.", str(caught.exception)
            )  # the scenario is named, not the copy
            self.assertEqual(self.scenario.read_text(), before)
        self.assertEqual(sorted(p.name for p in self.scenario.parent.iterdir()), ["scenario.yaml"])

    def test_a_loose_object_moved_twice_in_the_file_is_rejected(self) -> None:
        entry = "- {task: table, frame: pill, position_m: [0, 0, 0.05], yaw_deg: 0}\n"
        text = self.scenario.read_text().replace(
            "\ncontacts:", f"\ntask_frames:\n{entry}{entry}\ncontacts:", 1
        )
        self.scenario.write_text(text)
        with self.assertRaises(PackError) as caught:
            resolve_scenario(self.scenario)
        self.assertIn("/task_frames/1: table/pill is moved twice", str(caught.exception))

    def test_command_line(self) -> None:
        course = Path(self._directory.name) / "course.json"
        course.write_text(json.dumps({"run_options": {"bin_vinyl4_class": "fire"}}))
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main(["set-course", str(self.scenario), "--course", str(course)])
        self.assertEqual(code, 0, err.getvalue())
        self.assertIn("0 placements, 0 loose objects, 1 options", out.getvalue())
        self.assertEqual(self.resolved()["options"]["bin_vinyl4_class"], "fire")

        course.write_text(json.dumps({"run_options": {"bin_vinyl4_class": "ink"}}))
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main(["set-course", str(self.scenario), "--course", str(course)])
        self.assertEqual(code, 1)
        self.assertIn("NOT SAVED", err.getvalue())


if __name__ == "__main__":
    unittest.main()
