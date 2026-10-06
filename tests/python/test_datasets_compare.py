"""Environment comparison: variants, --set, view slicing, the stage-2 job and the sheet layout."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from typing import Any

import numpy as np
from nereus.datasets.compare import (
    SETTINGS_WIDTH,
    TILE_WIDTH,
    Cell,
    Row,
    View,
    _prepare,
    apply_settings,
    grid_job,
    parse_setting,
    settings_lines,
    sheet,
    variant,
    view_blocks,
)
from nereus.packs import PackError

HAS_CV2 = importlib.util.find_spec("cv2") is not None

# One randomization environment: [low, high] lists are ranges, everything else is a fixed value.
ENVIRONMENT: dict[str, Any] = {
    "id": "murky",
    "weight": 2.0,
    "water": {"scattering_scale": [1.3, 1.8], "tint_rgb": [0.1, 0.2, 0.3]},
    "lighting": {"exposure": [0.8, 1.0], "profile": "outdoor", "glare": 0.5},
    "image": {"noise_sigma": [1, 5]},
    "time_s": [0, 600],
}


def _record(name: str, scenario: int = 0, frame: str | None = "task") -> dict[str, Any]:
    """A minimal render record: the robot pose and target frame a comparison view reuses."""
    pose = {"position_m": [1.0, 2.0, -1.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}
    return {
        "name": name,
        "scenario_index": scenario,
        "robot": {"world_from_root": pose},
        "randomization": {"target_frame": frame},
    }


class VariantTests(unittest.TestCase):
    def test_min_mid_max(self) -> None:
        """variant() pins every range to its low, middle or high end; fixed values pass through."""
        low, mid, high = (variant(ENVIRONMENT, which) for which in ("min", "mid", "max"))
        self.assertEqual(
            [item["water"]["scattering_scale"] for item in (low, mid, high)], [1.3, 1.55, 1.8]
        )
        self.assertEqual([low["lighting"]["exposure"], high["lighting"]["exposure"]], [0.8, 1.0])
        for item in (low, mid, high):
            self.assertEqual(item["water"]["tint_rgb"], [0.1, 0.2, 0.3])  # a vector, not a range
            self.assertEqual(item["lighting"]["glare"], 0.5)
            self.assertEqual(item["lighting"]["profile"], "outdoor")
            self.assertEqual(item["time_s"], 300.0)  # always its middle
            self.assertEqual(item["weight"], 1.0)
        self.assertEqual((low["id"], high["id"]), ("murky:min", "murky:max"))

        # A pair of vectors is a range too: its middle is the element-wise mean.
        pair = dict(ENVIRONMENT, water={"tint_rgb": [[0.0, 0.1, 0.2], [0.2, 0.3, 0.4]]})
        np.testing.assert_allclose(variant(pair, "mid")["water"]["tint_rgb"], [0.1, 0.2, 0.3])
        with self.assertRaises(PackError):
            variant(ENVIRONMENT, "high")

    def test_set_values(self) -> None:
        """--set parses group.key=VALUE (JSON or a bare word) and overrides a copy of the env."""
        self.assertEqual(parse_setting("water.scattering=0.6"), ("water.scattering", 0.6))
        self.assertEqual(
            parse_setting("lighting.exposure=[0.7, 0.9]"), ("lighting.exposure", [0.7, 0.9])
        )
        self.assertEqual(parse_setting("lighting.profile=indoor"), ("lighting.profile", "indoor"))
        self.assertEqual(parse_setting("time_s=12"), ("time_s", 12))
        for bad in ("scattering=1", "water=1", "sky.blue=1", "water.scattering"):
            with self.assertRaises(PackError, msg=bad):
                parse_setting(bad)

        # Applying settings: overrides merge into a copy; invalid results are rejected.
        changed = apply_settings(
            ENVIRONMENT, [parse_setting("water.scattering=0.6"), parse_setting("image.blur_px=1")]
        )
        self.assertEqual(changed["water"]["scattering"], 0.6)
        self.assertNotIn("scattering_scale", changed["water"])  # the absolute form replaces it
        self.assertEqual(changed["image"], {"noise_sigma": [1, 5], "blur_px": 1})
        self.assertEqual(ENVIRONMENT["water"]["scattering_scale"], [1.3, 1.8])  # untouched
        for bad in ("water.scattering_scale=-1", "lighting.exposure=[1.2, 0.8]", "water.murk=2"):
            with self.assertRaises(PackError, msg=bad):
                apply_settings(ENVIRONMENT, [parse_setting(bad)])


class JobTests(unittest.TestCase):
    def test_views_split_the_range_near_to_far(self) -> None:
        """Each task block becomes N one-sample views over equal slices of its range/altitude."""
        job: dict[str, Any] = {
            "samples": [
                {"task": "torpedo", "count": 9, "sampler": {"type": "approach", "range_m": [1, 4]}},
                {
                    "task": "table",
                    "count": 9,
                    "sampler": {"type": "overhead", "altitude_m": [0.5, 1.4]},
                },
                {"task": None, "count": 9, "sampler": {"type": "free", "depth_m": [0.5, 1]}},
            ]
        }
        blocks = view_blocks(job, 3)
        self.assertEqual(
            [(b["task"], b["view"], b["count"]) for b in blocks][:4],
            [("torpedo", 0, 1), ("torpedo", 1, 1), ("torpedo", 2, 1), ("table", 0, 1)],
        )
        self.assertEqual([b["sampler"]["range_m"] for b in blocks[:3]], [[1, 2], [2, 3], [3, 4]])
        np.testing.assert_allclose(blocks[5]["sampler"]["altitude_m"], [1.1, 1.4])
        self.assertEqual(len(blocks), 6)  # no background views
        self.assertEqual(job["samples"][0]["sampler"]["range_m"], [1, 4])  # spec untouched

    def test_grid_job_fixes_poses_and_forces_environments(self) -> None:
        """The stage-2 job re-renders one scenario's view poses under every variant, no jitter."""
        base: dict[str, Any] = {
            "output": "/old",
            "scenarios": [{"id": "a", "resolved": "/a.json"}, {"id": "b", "resolved": "/b.json"}],
            "samples": [],
            "randomize": {
                "placement": {"task_yaw_deg": 10, "task_offset_m": 0.3, "groups": [["s", "t"]]},
                "indicators": {"latched_probability": 0.2},
                "environments": [],
                "environment_mode": "sweep",
            },
        }
        views = [
            View("torpedo", 0, {"type": "approach"}, _record("torpedo_000000")),
            View("gate", 1, {"type": "approach"}, _record("gate_000001", frame=None)),
            View("bins", 0, {"type": "approach"}, _record("bins_000002", scenario=1)),
        ]
        variants = [variant(ENVIRONMENT, "min"), variant(ENVIRONMENT, "max")]
        job, layout = grid_job(base, views, variants, 0, Path("/out/grid"))

        # Only scenario 0, the variants as weighted environments, and placement jitter off.
        self.assertEqual(job["output"], "/out/grid")
        self.assertEqual(job["scenarios"], [{"id": "a", "resolved": "/a.json"}])
        self.assertEqual(job["randomize"]["environments"], variants)
        self.assertEqual(job["randomize"]["environment_mode"], "weighted")
        placement = job["randomize"]["placement"]
        self.assertEqual((placement["task_yaw_deg"], placement["task_offset_m"]), (0, 0))
        self.assertEqual(job["randomize"]["indicators"]["latched_probability"], 0)

        # One fixed-pose sample per (view, variant); a view without a target frame omits it.
        self.assertEqual(len(job["samples"]), 4)  # two views of scenario 0 x two variants
        first, _, third, _ = job["samples"]
        self.assertEqual(
            first["sampler"],
            {"type": "fixed", "world_from_root": views[0].pose, "target_frame": "task"},
        )
        self.assertEqual((first["task"], first["count"], first["environment"]), ("torpedo", 1, 0))
        self.assertEqual(job["samples"][1]["environment"], 1)
        self.assertNotIn("target_frame", third["sampler"])
        self.assertEqual(
            [(view.task, index) for view, index in layout],
            [("torpedo", 0), ("torpedo", 1), ("gate", 0), ("gate", 1)],
        )
        self.assertEqual(base["randomize"]["placement"]["task_yaw_deg"], 10)  # base untouched

    def test_out_folder_guard(self) -> None:
        """The output folder is reused only for the same spec (or --force), never a foreign one."""
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory) / "cmp"
            _prepare(out, Path("/a/dataset.yaml"), force=False)
            (out / "views").mkdir()
            _prepare(out, Path("/a/dataset.yaml"), force=False)  # same spec: fresh views/
            self.assertFalse((out / "views").exists())
            with self.assertRaises(PackError):
                _prepare(out, Path("/b/dataset.yaml"), force=False)
            _prepare(out, Path("/b/dataset.yaml"), force=True)
            other = Path(directory) / "notes"
            other.mkdir()
            (other / "x.txt").write_text("keep")
            with self.assertRaises(PackError):
                _prepare(other, Path("/a/dataset.yaml"), force=False)


class SettingsTests(unittest.TestCase):
    def test_lines_and_pool_multiples(self) -> None:
        """Sheet setting lines show each value and, where the pool has one, its multiple of it."""
        randomization = {
            "water": {
                "scattering": 0.687,
                "absorption_per_m_rgb": [0.972, 0.2175, 0.0375],
                "tint_rgb": [0.025, 0.22, 0.29],
                "distance_scale": 1.88,
            },
            "lighting": {"profile": "outdoor", "exposure": 0.9, "direct_light": 0.5},
            "image": {"noise_sigma": 2.0, "blur_px": 0.4},
        }
        pool = {
            "water_optics": {
                "scattering": 0.458,
                "absorption_per_m_rgb": [0.648, 0.145, 0.025],
                "tint_rgb": [0.025, 0.22, 0.29],
            },
            "lighting": {"direct_light": 1.0},
        }
        lines = dict(settings_lines(randomization, pool))
        self.assertEqual(lines["scattering"], "0.687  x1.50")
        self.assertEqual(lines["absorption /m"], "0.972 0.217 0.037  x1.50")
        self.assertEqual(lines["tint rgb"], "0.025 0.220 0.290  x1.00")
        self.assertEqual(lines["direct light"], "0.50  x0.50")
        self.assertEqual(lines["ambient light"], "-")
        self.assertEqual(lines["noise sigma"], "2.00")


@unittest.skipUnless(HAS_CV2, "OpenCV (nereus[datasets]) not installed")
class SheetTests(unittest.TestCase):
    def test_layout(self) -> None:
        """Sheet size follows settings column + tiles + gaps; skipped cells show as None."""
        from nereus.datasets._mapping import ClassMap
        from nereus.packs._document import plain
        from ruamel.yaml import YAML
        from test_datasets_export import LABELS, write_render

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            render = write_render(root)
            classes = ClassMap(plain(YAML(typ="rt").load(LABELS)), "ffc")
            record = json.loads((render / "records" / "torpedo_000000.json").read_text())
            views = [
                View("torpedo", 0, {"type": "approach", "range_m": [1, 2]}, record),
                View("torpedo", 1, {"type": "approach", "range_m": [2, 3]}, record),
            ]

            # Two environments: murky (min, max) and clear (mid); murky:min's second view skipped.
            log = {"status": "skipped", "reasons": {"too_far": 1}}
            rows = [
                Row(
                    "murky",
                    "min",
                    [
                        Cell(render, record["name"], record, None),
                        Cell(render, "torpedo_000009", None, log),
                    ],
                ),
                Row("murky", "max", [Cell(render, record["name"], record, None)] * 2),
                Row("clear", "mid", [Cell(render, record["name"], record, None)] * 2),
            ]
            image, summary = sheet("torpedo", views, rows, classes, labels=True)

            # The fixture render is 40x20 px, so tiles keep that 2:1 aspect.
            tile_height = round(TILE_WIDTH * 20 / 40)
            self.assertEqual(image.shape[1], SETTINGS_WIDTH + 2 * (TILE_WIDTH + 6) + 6)
            # header + 3 rows (each with a gap) + one separator between environments
            self.assertEqual(image.shape[0], 58 + 3 * (tile_height + 6) + 12)
            self.assertEqual(summary[0]["cells"], ["torpedo_000000", None])
            self.assertEqual([row["variant"] for row in summary], ["min", "max", "mid"])


if __name__ == "__main__":
    unittest.main()
