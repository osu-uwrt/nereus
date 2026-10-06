"""The task pack's parts.yaml resolves, its part masks match the generator, and the UWRT label and dataset
packs refer to parts, frames and regions that exist."""

import fnmatch
import importlib.util
import re
import struct
import subprocess
import sys
import unittest
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

ROOT = Path(__file__).resolve().parents[2]
PACK = ROOT / "content/packs/tasks/robosub_2026"
LABELS = ROOT / "content/packs/labels/uwrt/labels.yaml"
DATASETS = ROOT / "content/packs/datasets"
TOOL = ROOT / "tools/generate_part_masks.py"
# Part names are lower-case snake_case identifiers.
PART_NAME = re.compile(r"[a-z0-9_]+")


def load(path: Path) -> Any:
    """Plain YAML load (no pack validation: these tests check the raw files)."""
    return YAML(typ="safe").load(path.read_text())


def png_header(path: Path) -> tuple[int, int, int, int]:
    """(width, height, bit depth, colour type) from the IHDR chunk."""
    data = path.read_bytes()[:26]
    assert data[:8] == b"\x89PNG\r\n\x1a\n" and data[12:16] == b"IHDR", path
    width, height, depth, colour = struct.unpack(">IIBB", data[16:26])
    return width, height, depth, colour


def task_files() -> dict[str, Any]:
    """The tasks pack's included task documents, by task id."""
    tasks = load(PACK / "tasks.yaml")
    return {doc["id"]: doc for doc in (load(PACK / name) for name in tasks["tasks"])}


def part_names() -> set[str]:
    """Every part name parts.yaml defines, from texture masks and visual meshes."""
    parts = load(PACK / "parts.yaml")
    names = {p["part"] for t in parts["textures"] for p in t["parts"]}
    for v in parts["visuals"]:
        names |= {v["part"]} if "part" in v else set(v["materials"].values())
    return names


class PartMasksTest(unittest.TestCase):
    @unittest.skipUnless(
        importlib.util.find_spec("PIL") and importlib.util.find_spec("scipy"),
        "the mask generator needs Pillow and scipy",
    )
    def test_committed_masks_match_generator(self) -> None:
        """generate_part_masks.py --check finds every committed mask PNG up to date."""
        result = subprocess.run(
            [sys.executable, str(TOOL), "--check"], capture_output=True, text=True, check=False
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_textures_resolve(self) -> None:
        """Each texture entry names a PNG asset whose mask is 8-bit grey at the same size."""
        parts = load(PACK / "parts.yaml")
        tasks = load(PACK / "tasks.yaml")
        assets = {a["id"]: a["path"] for a in tasks["assets"]}
        self.assertEqual(parts["kind"], "parts")
        self.assertEqual(parts["tasks"], tasks["id"])
        self.assertTrue(parts["textures"])
        for entry in parts["textures"]:
            texture_id = entry["texture"]
            self.assertIn(texture_id, assets)
            texture, mask = PACK / assets[texture_id], PACK / entry["mask"]
            self.assertEqual(texture.suffix, ".png", texture_id)
            self.assertTrue(mask.is_file(), mask)
            width, height = png_header(texture)[:2]
            self.assertEqual(
                png_header(mask), (width, height, 8, 0), f"{mask}: 8-bit gray, texture size"
            )

            # Mask values are unique and non-zero; seeds lie inside the texture.
            values = [p["value"] for p in entry["parts"]]
            self.assertEqual(len(values), len(set(values)), texture_id)
            for p in entry["parts"]:
                self.assertTrue(1 <= p["value"] <= 255, p)
                self.assertRegex(p["part"], PART_NAME)
                x, y = p["seed_px"]
                self.assertTrue(0 <= x < width and 0 <= y < height, p)
                self.assertIn(p.get("fill", "small"), ("small", "all", "none"), p)

    def test_visuals_resolve(self) -> None:
        """Each visual names an asset its task draws, real frames, and part XOR materials."""
        parts = load(PACK / "parts.yaml")
        assets = {a["id"] for a in load(PACK / "tasks.yaml")["assets"]}
        tasks = task_files()
        for v in parts["visuals"]:
            task = tasks[v["task"]]
            self.assertIn(v["asset"], assets)

            # Assets the task's props draw, via `visuals` lists or a single visual_asset.
            used = [
                visual["asset"]
                for prop in task["props"]
                for visual in prop["parameters"].get("visuals", [])
                + [{"asset": prop["parameters"].get("visual_asset")}]
            ]
            self.assertIn(v["asset"], used, f"{v['task']} draws no {v['asset']}")
            frames = {f["id"] for f in task["frames"]}
            if "frame" in v:
                self.assertIn(v["frame"], frames)
            if "indicator" in v:
                self.assertIn(v["indicator"], {r["id"] for r in task["regions"]})
            self.assertEqual(("part" in v) + ("materials" in v), 1, v)
            for name in [v["part"]] if "part" in v else v["materials"].values():
                self.assertRegex(name, PART_NAME)
            self.assertIn(v.get("split", "none"), ("none", "connected"))

    def test_labels_name_existing_parts(self) -> None:
        """Every label rule names a real task, a model class, and globs matching some part."""
        labels = load(LABELS)
        names = part_names()
        tasks = task_files()
        classes = {c for model in labels["models"].values() for c in model["classes"]}
        for task, mapping in labels["tasks"].items():
            self.assertIn(task, tasks)
            for cls, source in mapping.items():
                self.assertIn(cls, classes)
                patterns = source["parts"] if isinstance(source, dict) else source
                for pattern in patterns:
                    self.assertTrue(fnmatch.filter(names, pattern), f"{task}.{cls}: {pattern}")

    def test_datasets_resolve(self) -> None:
        """Each dataset spec's id, labels, model, scenarios and sampler frames exist."""
        labels = load(LABELS)
        tasks = task_files()
        specs = sorted(DATASETS.glob("*/dataset.yaml"))
        self.assertTrue(specs)
        for spec in specs:
            doc = load(spec)
            self.assertEqual(doc["id"], spec.parent.name)
            self.assertTrue((spec.parent / doc["labels"] / "labels.yaml").is_file(), spec)
            self.assertIn(doc["model"], labels["models"])
            for scenario in doc["scenarios"]:
                self.assertTrue((spec.parent / scenario / "scenario.yaml").is_file(), scenario)
            for task, block in doc["tasks"].items():
                frame = block["sampler"].get("frame", "task")
                frames = {f["id"] for f in tasks[task]["frames"]} | {"task"}
                for f in frame if isinstance(frame, list) else [frame]:
                    self.assertIn(f, frames, f"{spec.parent.name}.{task}")


if __name__ == "__main__":
    unittest.main()
