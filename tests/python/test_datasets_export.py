"""YOLO export and previews from synthetic renderer records (needs the datasets extra: OpenCV)."""

import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from typing import Any

import numpy as np
from nereus.datasets.__main__ import main
from nereus.datasets.export import (
    bbox_line,
    cv2_module,
    export,
    fill_holes,
    merge_multi_segment,
    obb_line,
    seg_line,
    split_of,
)
from nereus.datasets.preview import ALPHA, PALETTE, preview
from nereus.packs import PackError

# The CI python job does not install the datasets extra; OpenCV is imported through the package.
HAS_CV2 = importlib.util.find_spec("cv2") is not None
cv2: Any = cv2_module() if HAS_CV2 else None

LABELS = """\
kind: labels
id: test_labels
tasks_pack: robosub_2026
models:
  ffc: {camera: ffc, max_range_m: 5.0, classes: [blood, circle, fire, magnet]}
  dfc: {camera: dfc, max_range_m: 3.0, classes: [pill, fire]}
classes:
  circle: {shape: outer}
tasks:
  torpedo: {fire: [icon_fire], blood: [icon_blood], circle: ['ring*']}
  bins: {magnet: {parts: [magnet_cover], when: {indicator: red}}}
  table: {pill: [icon_pill]}
export:
  min_visible_px: 10
  fragments: merge
  min_fragment_px: 10
"""

WIDTH, HEIGHT = 40, 20
SCENARIO = Path(__file__).resolve().parents[2] / "content/packs/scenarios/talos_uwrt"


def _instance(ident: int, task: str, part: str, **extra: Any) -> dict[str, Any]:
    instance = {
        "id": ident,
        "task": task,
        "part": part,
        "source": f"{task}/x/y#0",
        "piece": 0,
        "indicator": None,
        "pixels": 0,
        "bbox_xywh": [0, 0, 0, 0],
        "depth_m": {"min": 1.0, "median": 1.2, "max": 1.4},
        "truncated": False,
    }
    instance.update(extra)
    return instance


def write_render(root: Path, model: str = "ffc", folder: str = "render") -> Path:
    """A render folder with four samples covering the label-pack rules, seen by ``model``."""
    render = root / folder
    for folder in ("images", "ids", "records", "job"):
        (render / folder).mkdir(parents=True)
    (root / "labels.yaml").write_text(LABELS)
    settings = {
        "labels": str(root / "labels.yaml"),
        "model": model,
        "split": {"train": 1, "val": 0},
    }
    (render / "job" / "export.json").write_text(json.dumps(settings))

    samples: dict[str, tuple[str | None, Any, list[dict[str, Any]]]] = {}
    # Torpedo: a ring with a hole, a fire emoji split by an unlabelled pole, a truncated blood
    # emoji at the bottom-right border and a 2x2 sliver under min_visible_px.
    ids = np.zeros((HEIGHT, WIDTH), np.uint16)
    ids[2:12, 2:12] = 1
    ids[5:9, 5:9] = 0
    ids[2:8, 15:25] = 2
    ids[:, 19:21] = 9
    ids[10:20, 34:40] = 3
    ids[15:17, 15:17] = 4
    samples["torpedo_000000"] = (
        "torpedo",
        ids,
        [
            _instance(1, "torpedo", "ring"),
            _instance(2, "torpedo", "icon_fire"),
            _instance(3, "torpedo", "icon_blood", truncated=True),
            _instance(4, "torpedo", "icon_fire"),
            _instance(9, "slalom", "pole_white"),
        ],
    )
    # Bins: the magnet cover is labelled while red, not while green; a table pill (not an ffc class).
    ids = np.zeros((HEIGHT, WIDTH), np.uint16)
    ids[0:10, 0:10] = 1
    ids[0:10, 20:30] = 2
    ids[12:18, 2:8] = 3
    samples["bins_000001"] = (
        "bins",
        ids,
        [
            _instance(1, "bins", "magnet_cover", indicator="red"),
            _instance(2, "bins", "magnet_cover", indicator="green"),
            _instance(3, "table", "icon_pill"),
        ],
    )
    # Background: only an unlabelled part in view.
    ids = np.zeros((HEIGHT, WIDTH), np.uint16)
    ids[0:20, 10:12] = 1
    samples["background_000002"] = (None, ids, [_instance(1, "slalom", "pole_white")])
    # A labelled instance beyond the ffc range (5 m): the image is skipped.
    ids = np.zeros((HEIGHT, WIDTH), np.uint16)
    ids[5:15, 5:15] = 1
    far = {"min": 5.5, "median": 6.0, "max": 6.5}
    samples["torpedo_000003"] = (
        "torpedo",
        ids,
        [_instance(1, "torpedo", "icon_fire", depth_m=far)],
    )

    for k, (name, (task, ids, instances)) in enumerate(samples.items()):
        write_sample(render, model, k, name, task, ids, instances)
    return render


def write_sample(
    render: Path,
    model: str,
    k: int,
    name: str,
    task: str | None,
    ids: Any,
    instances: list[dict[str, Any]],
) -> None:
    image = np.full((HEIGHT, WIDTH, 3), 90, np.uint8)
    cv2.imwrite(str(render / "images" / f"{name}.png"), image)
    cv2.imwrite(str(render / "ids" / f"{name}.png"), ids)
    environment = ["nominal", "murky"][k % 2]
    record = {
        "format": "nereus.dataset_record.v1",
        "dataset": "test",
        "sample": k,
        "name": name,
        "task": task,
        "scenario": "s",
        "scenario_index": 0,
        "attempts": 1,
        "environment": environment,
        "environment_index": k % 2,
        "image": f"images/{name}.png",
        "ids": f"ids/{name}.png",
        "camera": {"sensor": model, "width": WIDTH, "height": HEIGHT},
        "instances": instances,
    }
    (render / "records" / f"{name}.json").write_text(json.dumps(record))


def _labels(out: Path, name: str) -> list[list[float]]:
    path = out / "labels" / "train" / f"{name}.txt"
    return [[float(value) for value in line.split()] for line in path.read_text().splitlines()]


@unittest.skipUnless(HAS_CV2, "OpenCV (nereus[datasets]) not installed")
class GeometryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ids = np.zeros((HEIGHT, WIDTH), np.uint16)
        self.ids[2:12, 2:12] = 1
        self.ids[5:9, 5:9] = 0
        self.ids[2:8, 15:25] = 2
        self.ids[:, 19:21] = 9
        self.ids[10:20, 34:40] = 3

    def assertLine(self, line: str | None, expected: list[float]) -> None:
        assert line is not None
        values = [float(value) for value in line.split()]
        self.assertEqual(len(values), len(expected), line)
        for value, want in zip(values, expected):
            self.assertAlmostEqual(value, want, places=6, msg=line)

    def test_ring_with_hole(self) -> None:
        ring = self.ids == 1
        self.assertLine(bbox_line(ring, 1), [1, 0.175, 0.35, 0.25, 0.5])
        # Only the outer contour: the hole never appears in the polygon.
        self.assertLine(seg_line(ring, 1), [1, 0.05, 0.1, 0.05, 0.55, 0.275, 0.55, 0.275, 0.1])
        self.assertLine(obb_line(ring, 1), [1, 0.05, 0.1, 0.3, 0.1, 0.3, 0.6, 0.05, 0.6])
        self.assertEqual(int(ring.sum()), 84)
        self.assertEqual(int(fill_holes(ring).sum()), 100)

    def test_split_mask_joins_like_ultralytics(self) -> None:
        split = self.ids == 2
        self.assertLine(bbox_line(split, 2), [2, 0.5, 0.25, 0.25, 0.3])
        # Left piece from its exit corner (18, 7), bridge to (21, 7), right piece, closed.
        left = [18, 7, 18, 2, 15, 2, 15, 7, 18, 7]
        right = [21, 7, 24, 7, 24, 2, 21, 2, 21, 7]
        scale = [WIDTH, HEIGHT] * 5
        expected = [v / s for v, s in zip(left, scale)] + [v / s for v, s in zip(right, scale)]
        self.assertLine(seg_line(split, 2), [2, *expected])
        self.assertLine(obb_line(split, 2), [2, 0.375, 0.1, 0.625, 0.1, 0.625, 0.4, 0.375, 0.4])

    def test_truncated_mask_reaches_the_border(self) -> None:
        edge = self.ids == 3
        self.assertLine(bbox_line(edge, 0), [0, 0.925, 0.75, 0.15, 0.5])
        self.assertLine(obb_line(edge, 0), [0, 0.85, 0.5, 1.0, 0.5, 1.0, 1.0, 0.85, 1.0])
        self.assertLine(seg_line(edge, 0), [0, 0.85, 0.5, 0.85, 0.95, 0.975, 0.95, 0.975, 0.5])

    def test_three_segments_walk_back_through_the_middle(self) -> None:
        a = np.array([[2, 2], [2, 5], [5, 5], [5, 2]], dtype=np.float64)
        b = np.asarray(a + [8, 0], dtype=np.float64)
        c = np.asarray(a + [18, 0], dtype=np.float64)
        merged = merge_multi_segment([a, b, c]).tolist()
        self.assertEqual(
            merged,
            [[5, 5], [5, 2], [2, 2], [2, 5], [5, 5]]
            + [[10, 5], [13, 5]]
            + [[20, 5], [23, 5], [23, 2], [20, 2], [20, 5]]
            + [[13, 5], [13, 2], [10, 2], [10, 5]],
        )

    def test_one_pixel_wide_instance_still_gets_a_polygon(self) -> None:
        line = np.zeros((HEIGHT, WIDTH), bool)
        line[2:12, 5] = True
        # The contour collapses to two points: the pixel-edge rectangle stands in.
        self.assertLine(seg_line(line, 0), [0, 0.125, 0.1, 0.125, 0.6, 0.15, 0.6, 0.15, 0.1])
        self.assertLine(bbox_line(line, 0), [0, 0.1375, 0.35, 0.025, 0.5])

    def test_rotated_obb(self) -> None:
        mask = np.zeros((100, 100), np.uint8)
        corners = cv2.boxPoints(((50, 50), (40, 20), 30)).astype(np.int32)
        cv2.fillPoly(mask, [corners], 1)
        line = obb_line(mask.astype(bool), 0)
        assert line is not None
        values = np.array([float(value) for value in line.split()[1:]]).reshape(4, 2) * 100
        np.testing.assert_allclose(values.mean(axis=0), [50, 50], atol=0.6)
        sides = np.linalg.norm(np.roll(values, -1, axis=0) - values, axis=1)
        np.testing.assert_allclose(sorted(sides), [21, 21, 41, 41], atol=1.5)
        self.assertEqual(int(np.argmin(values[:, 1])), 0)  # starts at the top corner


class SplitTests(unittest.TestCase):
    def test_split_is_stable_and_proportional(self) -> None:
        fractions = {"train": 0.7, "val": 0.2, "test": 0.1}
        names = [f"torpedo_{k:06d}" for k in range(4000)]
        splits = [split_of(name, fractions) for name in names]
        self.assertEqual(splits, [split_of(name, fractions) for name in names])
        for split, fraction in fractions.items():
            self.assertAlmostEqual(splits.count(split) / len(names), fraction, delta=0.03)
        self.assertEqual({split_of(name, {"train": 1.0, "val": 0.0}) for name in names}, {"train"})


@unittest.skipUnless(HAS_CV2, "OpenCV (nereus[datasets]) not installed")
class ExportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.render = write_render(self.root)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_seg_applies_the_label_pack(self) -> None:
        out = self.root / "seg"
        summary = export(self.render, "yolo-seg", out)
        torpedo = _labels(out, "torpedo_000000")
        # ring -> circle (1), split fire (2) and truncated blood (0); sliver and white pole dropped.
        self.assertEqual([int(line[0]) for line in torpedo], [1, 2, 0])
        self.assertEqual(len(torpedo[1]), 1 + 20)  # two 4-corner pieces, each closed
        bins = _labels(out, "bins_000001")
        self.assertEqual(bins, [[3, 0.0, 0.0, 0.0, 0.45, 0.225, 0.45, 0.225, 0.0]])
        self.assertEqual((out / "labels/train/background_000002.txt").read_text(), "")
        self.assertFalse((out / "labels/train/torpedo_000003.txt").exists())
        self.assertEqual(summary.skipped_far, ["torpedo_000003"])
        self.assertEqual(summary.dropped_small, 1)
        self.assertEqual(summary.backgrounds, 1)
        self.assertEqual(summary.counts, [1, 1, 1, 1])
        self.assertEqual(summary.images["train"], 3)
        self.assertEqual(summary.environments, {"nominal": 2, "murky": 1})

    def test_bbox_obb_and_layout(self) -> None:
        out = self.root / "bbox"
        export(self.render, "yolo-bbox", out)
        torpedo = _labels(out, "torpedo_000000")
        expected = [
            [1, 0.175, 0.35, 0.25, 0.5],
            [2, 0.5, 0.25, 0.25, 0.3],
            [0, 0.925, 0.75, 0.15, 0.5],
        ]
        np.testing.assert_allclose(torpedo, expected, atol=1e-6)
        image = out / "images/train/torpedo_000000.png"
        self.assertEqual(
            image.stat().st_ino, (self.render / "images/torpedo_000000.png").stat().st_ino
        )
        self.assertEqual(
            (out / "data.yaml").read_text(),
            "train: images/train\nval: images/val\nnc: 4\n"
            'names: ["blood", "circle", "fire", "magnet"]\n',
        )
        obb = self.root / "obb"
        export(self.render, "yolo-obb", obb)
        ring = _labels(obb, "torpedo_000000")[0]
        np.testing.assert_allclose(ring, [1, 0.05, 0.1, 0.3, 0.1, 0.3, 0.6, 0.05, 0.6], atol=1e-6)

    def test_class_missing_from_model_and_balance_warning(self) -> None:
        out = self.root / "dfc"
        summary = export(write_render(self.root, "dfc", "render_dfc"), "yolo-bbox", out)
        # dfc has pill and fire: the pill is labelled, the torpedo fire too (range 3 m > 1.2 m).
        self.assertEqual(_labels(out, "bins_000001")[0][0], 0)
        self.assertEqual(summary.counts, [1, 1])
        self.assertEqual(summary.unbalanced(), [])
        uneven = export(self.render, "yolo-bbox", self.root / "ffc")
        self.assertEqual(uneven.unbalanced(), [])
        uneven.counts = [10, 1, 1, 1]
        self.assertEqual(uneven.unbalanced(), ["blood", "circle", "fire", "magnet"])
        self.assertTrue(any(line.startswith("WARNING") for line in uneven.report()))

    def test_fragment_policies(self) -> None:
        # One fire emoji in three pieces: 100 px, 30 px and a 9 px crumb.
        ids = np.zeros((HEIGHT, WIDTH), np.uint16)
        ids[2:12, 2:12] = 1
        ids[2:7, 20:26] = 1
        ids[15:18, 30:33] = 1
        render = write_render(self.root, folder="fragments")
        write_sample(
            render,
            "ffc",
            4,
            "torpedo_000004",
            "torpedo",
            ids,
            [_instance(1, "torpedo", "icon_fire")],
        )
        for policy, expected in [
            ("merge", [2, 0.35, 0.35, 0.6, 0.5]),  # both pieces, never the crumb
            ("keep_largest", [2, 0.175, 0.35, 0.25, 0.5]),
        ]:
            labels = self.root / f"{policy}.yaml"
            text = LABELS.replace("fragments: merge", f"fragments: {policy}")
            labels.write_text(text.replace("min_fragment_px: 10", "min_fragment_px: 25"))
            out = self.root / policy
            summary = export(render, "yolo-bbox", out, labels=labels)
            np.testing.assert_allclose(_labels(out, "torpedo_000004"), [expected], atol=1e-6)
            self.assertGreaterEqual(summary.crumbs, 1)
            self.assertEqual(summary.skipped_fragmented, [])
        labels = self.root / "reject.yaml"
        labels.write_text(LABELS.replace("fragments: merge", "fragments: reject"))
        summary = export(render, "yolo-seg", self.root / "reject", labels=labels)
        # The split fire of torpedo_000000 (two 24 px pieces) is fragmented too.
        self.assertEqual(summary.skipped_fragmented, ["torpedo_000000", "torpedo_000004"])
        self.assertIn("  fragmented images skipped: 2", summary.report())
        self.assertFalse((self.root / "reject/labels/train/torpedo_000004.txt").exists())

    def test_model_camera_must_match_the_render(self) -> None:
        with self.assertRaises(PackError) as caught:
            export(self.render, "yolo-bbox", self.root / "wrong", model="dfc")
        self.assertIn("rendered by camera 'ffc', but model 'dfc'", caught.exception.problems[0])

    def test_label_override_is_checked_against_the_course(self) -> None:
        settings = self.render / "job" / "export.json"
        data = json.loads(settings.read_text())
        data["scenarios"] = [str(SCENARIO / "scenario.yaml")]
        settings.write_text(json.dumps(data))
        good = self.root / "labels.yaml"
        export(self.render, "yolo-bbox", self.root / "good", labels=good)
        typo = self.root / "typo.yaml"
        typo.write_text(LABELS.replace("[icon_fire]", "[icon_fyre]"))
        with self.assertRaises(PackError) as caught:
            export(self.render, "yolo-bbox", self.root / "typo", labels=typo)
        self.assertIn("'icon_fyre' matches no part of task 'torpedo'", caught.exception.problems[0])

    def test_reexport_replaces_and_refuses_foreign_folders(self) -> None:
        out = self.root / "again"
        export(self.render, "yolo-bbox", out)
        stale = out / "labels/train/stale.txt"
        stale.write_text("0 0 0 0 0\n")
        export(self.render, "yolo-bbox", out)
        self.assertFalse(stale.exists())
        foreign = self.root / "foreign"
        foreign.mkdir()
        (foreign / "notes.txt").write_text("keep")
        with contextlib.redirect_stderr(io.StringIO()):
            code = main(["export", str(self.render), "--format", "yolo-seg", "--out", str(foreign)])
        self.assertEqual(code, 1)

    def test_cli_export_and_preview(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(main(["export", str(self.render), "--format", "yolo-obb"]), 0)
            self.assertEqual(main(["preview", str(self.render), "--count", "3"]), 0)
        self.assertIn("yolo-obb: 3 images", stdout.getvalue())
        self.assertTrue((self.render / "yolo-obb" / "data.yaml").is_file())
        sheet = cv2.imread(str(self.render / "preview.jpg"))
        self.assertEqual(sheet.shape[1], 3 * 480 + 4 * 4)

    def test_preview_colours_labelled_pixels(self) -> None:
        sheet = preview(self.render, self.root / "sheet.png", count=4, tile_width=WIDTH * 10)
        image = cv2.imread(str(sheet)).astype(int)
        # The red magnet cover: its class colour blended 45 % over grey 90.
        blended = np.array(PALETTE[3]) * ALPHA + 90 * (1 - ALPHA)
        close = np.abs(image - blended).max(axis=2) <= 2
        self.assertGreater(int(close.sum()), 100 * 100 // 2)


if __name__ == "__main__":
    unittest.main()
