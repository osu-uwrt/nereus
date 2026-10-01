"""Parts, labels and dataset documents: schemas, cross-checks and the class-order check."""

import contextlib
import copy
import io
import tempfile
import unittest
from pathlib import Path
from typing import Any

from nereus.datasets import ClassMap, load_document
from nereus.datasets.__main__ import main
from nereus.datasets._documents import (
    Course,
    Document,
    check_data,
    check_dataset,
    check_labels,
    check_parts,
    course,
    label_warnings,
    parts_by_task,
)
from nereus.datasets._mapping import class_order_problems
from nereus.packs import PackError, ResolvedScenario, resolve_scenario

ROOT = Path(__file__).resolve().parents[2]
PACKS = ROOT / "content" / "packs"
LABELS = PACKS / "labels" / "uwrt"
PARTS = PACKS / "tasks" / "robosub_2026" / "parts.yaml"
DATASETS = PACKS / "datasets"
# The UWRT detector's parameter file, when this checkout sits in the UWRT workspace.
YOLO_CONFIG = ROOT.parent / "src/riptide_perception/tensor_detector/config/yolo_orientation.yaml"


def _labels() -> dict[str, Any]:
    return {
        "kind": "labels",
        "id": "test",
        "tasks_pack": "robosub_2026",
        "models": {"ffc": {"camera": "ffc", "max_range_m": 5.0, "classes": ["fire", "circle"]}},
        "tasks": {"torpedo": {"fire": ["icon_fire"], "circle": ["ring"]}},
    }


def _dataset() -> dict[str, Any]:
    return {
        "kind": "dataset",
        "id": "test",
        "labels": "../labels",
        "model": "ffc",
        "scenarios": ["../scenario"],
        "seed": 1,
        "split": {"train": 0.8, "val": 0.2},
        "tasks": {
            "torpedo": {
                "count": 5,
                "sampler": {"type": "approach", "frame": ["task"], "range_m": [1, 2]},
            }
        },
    }


class SchemaTests(unittest.TestCase):
    def test_minimal_documents_pass(self) -> None:
        self.assertEqual(check_data("labels", _labels()), [])
        self.assertEqual(check_data("dataset", _dataset()), [])
        parts = {"kind": "parts", "id": "p", "tasks": "robosub_2026"}
        self.assertEqual(check_data("parts", parts), [])

    def test_parts_rejections(self) -> None:
        texture: dict[str, Any] = {
            "texture": "torpedo_texture",
            "mask": "assets/x.png",
            "parts": [
                {"value": 1, "part": "icon_fire", "seed_px": [1, 2], "fill": "all"},
                {"value": 1, "part": "ring", "seed_px": [3, 4], "threshold": 6, "close_px": 2},
            ],
        }
        both = {"task": "bins", "asset": "m", "part": "a", "materials": {"M": "b"}}
        parts = {"kind": "parts", "id": "p", "tasks": "t", "textures": [texture]}
        self.assertEqual(check_data("parts", parts), ["/textures/0/parts: duplicate value '1'"])
        parts["visuals"] = [both]
        self.assertTrue(check_data("parts", parts)[0].startswith("/visuals/0"))
        texture["parts"][0]["value"] = 0
        self.assertIn("/textures/0/parts/0/value", check_data("parts", parts)[0])
        texture["parts"][0].update(value=1, fill="some")
        self.assertIn("/textures/0/parts/0/fill", check_data("parts", parts)[0])

    def test_labels_rejections(self) -> None:
        labels = _labels()
        labels["models"]["ffc"]["classes"].append("fire")
        self.assertEqual(
            check_data("labels", labels), ["/models/ffc/classes: duplicate class 'fire'"]
        )
        labels = _labels()
        labels["tasks"]["bins"] = {"magnet": {"parts": ["cover"], "when": {"colour": "red"}}}
        self.assertTrue(check_data("labels", labels))
        labels["tasks"]["bins"] = {"magnet": {"parts": ["cover"], "when": {"indicator": "red"}}}
        labels["classes"] = {"circle": {"shape": "outer"}}
        self.assertEqual(check_data("labels", labels), [])
        labels["classes"] = {"circle": {"shape": "convex"}}
        self.assertTrue(check_data("labels", labels))

    def test_dataset_rejections(self) -> None:
        dataset = _dataset()
        dataset["split"] = {"train": 0.8, "val": 0.3}
        self.assertEqual(check_data("dataset", dataset), ["/split: fractions sum to 1.1, not 1"])
        dataset = _dataset()
        dataset["tasks"]["torpedo"]["sampler"]["range_m"] = [3, 1]
        self.assertEqual(
            check_data("dataset", dataset),
            ["/tasks/torpedo/sampler/range_m: range low 3 exceeds high 1"],
        )
        dataset = _dataset()
        dataset["tasks"]["torpedo"]["sampler"]["altitude_m"] = [1, 2]  # not an approach key
        self.assertTrue(check_data("dataset", dataset))
        dataset = _dataset()
        dataset["randomize"] = {"water": {"tint": [0, 1]}}
        self.assertTrue(check_data("dataset", dataset))
        dataset = _dataset()
        dataset["image"] = {"resolution_px": [640, 0]}
        self.assertTrue(check_data("dataset", dataset))
        dataset = _dataset()
        dataset["tasks"]["background"] = dataset["tasks"].pop("torpedo")
        self.assertIn("/tasks/background", check_data("dataset", dataset)[0])

    def test_environment_values(self) -> None:
        def problems(randomize: dict[str, Any]) -> list[str]:
            dataset = _dataset()
            dataset["randomize"] = randomize
            return check_data("dataset", dataset)

        good = {"list": [{"id": "a", "water": {"scattering_scale": [0.5, 1.0]}}]}
        self.assertEqual(problems({"environments": good}), [])
        bad = [
            {"id": "neg", "water": {"scattering_scale": [-0.5, 1.0]}},
            {"id": "tint", "water": {"tint_rgb": [0.1, 0.2, 1.5]}},
            {"id": "order", "lighting": {"exposure": [1.2, 0.8]}},
            {"id": "both", "water": {"scattering": 0.4, "scattering_scale": 1.2}},
            {"id": "glare", "lighting": {"glare": -1}},
        ]
        for entry in bad:
            found = problems({"environments": {"list": [entry]}})
            self.assertTrue(found, entry["id"])
        found = problems({"environments": {"list": [bad[2]]}})
        self.assertEqual(
            found,
            ["/randomize/environments/list/0/lighting/exposure: range low 1.2 exceeds high 0.8"],
        )
        # Sweep values are checked once laid into each environment.
        sweep = {"sweep": {"lighting.caustics": [0.5, -1]}}
        self.assertTrue(any("caustics" in item for item in problems({"environments": sweep})))
        typo = {"sweep": {"water.scatering_scale": [1.0]}}
        self.assertTrue(any("scatering_scale" in item for item in problems({"environments": typo})))
        both = {"list": [{"id": "a"}, {"id": "a"}]}
        self.assertIn("duplicate environment 'a'", problems({"environments": both})[0])

    def test_load_document_reports_the_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            (folder / "labels.yaml").write_text("kind: labels\nid: x\n")
            with self.assertRaises(PackError) as caught:
                load_document(folder, "labels")
            self.assertTrue(caught.exception.problems[0].startswith(str(folder / "labels.yaml")))
            with self.assertRaises(PackError):
                load_document(folder, "dataset")


class CrossCheckTests(unittest.TestCase):
    resolved: ResolvedScenario
    place: Course

    @classmethod
    def setUpClass(cls) -> None:
        cls.resolved = resolve_scenario(PACKS / "scenarios" / "talos_uwrt")
        cls.place = course(cls.resolved)

    def _parts(self, data: dict[str, Any]) -> Document:
        return Document(self.place.parts_file, "parts", data)

    def _base_parts(self) -> dict[str, Any]:
        return {
            "kind": "parts",
            "id": "p",
            "tasks": "robosub_2026",
            "textures": [
                {
                    "texture": "torpedo_texture",
                    "mask": "assets/torpedo/Task4_ver1_Fixed.png",
                    "parts": [
                        {"value": 1, "part": "icon_fire", "seed_px": [0, 0]},
                        {"value": 5, "part": "ring", "seed_px": [0, 0]},
                    ],
                }
            ],
            "visuals": [
                {
                    "task": "bins",
                    "asset": "magnet_cover_mesh",
                    "frame": "magnet_target1",
                    "part": "magnet_cover",
                    "indicator": "magnet_target1",
                }
            ],
        }

    def test_textures_belong_to_the_tasks_drawing_them(self) -> None:
        self.assertEqual(self.place.texture_tasks["torpedo_texture"], {"torpedo"})
        self.assertEqual(self.place.texture_tasks["bin_vinyl_fire_texture"], {"bins"})
        self.assertEqual(self.place.texture_tasks["pill_visual_task5_pill_fixed"], {"table"})
        parts = self._parts(self._base_parts())
        self.assertEqual(check_parts(parts, self.place), [])
        available = parts_by_task(parts, self.place)
        self.assertEqual(available["torpedo"], {"icon_fire", "ring"})
        self.assertEqual(available["bins"], {"magnet_cover"})

    def test_parts_rejections(self) -> None:
        data = self._base_parts()
        data["tasks"] = "robosub_2025"
        data["textures"][0]["texture"] = "no_such_texture"
        data["textures"][0]["mask"] = "assets/missing.png"
        data["visuals"][0].update(frame="nowhere", indicator="nothing")
        data["visuals"].append({"task": "slalom", "asset": "gate_mesh", "part": "x"})
        problems = [item.split(":", 1)[1] for item in check_parts(self._parts(data), self.place)]
        self.assertEqual(
            problems,
            [
                "/tasks: 'robosub_2025' is not the tasks pack id 'robosub_2026'",
                "/textures/0/texture: no tasks-pack asset 'no_such_texture'",
                "/textures/0/mask: assets/missing.png is not a file",
                "/visuals/0/frame: task 'bins' has no frame 'nowhere'",
                "/visuals/0/indicator: task 'bins' has no region 'nothing'",
                "/visuals/1/asset: task 'slalom' draws no visual 'gate_mesh'",
            ],
        )

    def test_label_rejections(self) -> None:
        labels = _labels()
        labels["models"]["ffc"]["classes"] += ["blood", "fire_too", "magnet"]
        labels["tasks_pack"] = "other"
        labels["tasks"]["torpedo"]["blood"] = ["icon_blood"]  # no such part in these parts
        labels["tasks"]["torpedo"]["fire_too"] = ["icon_f*"]  # icon_fire is already fire
        labels["tasks"]["bins"] = {"magnet": {"parts": ["magnet_*"], "when": {"indicator": "blue"}}}
        labels["tasks"]["nowhere"] = {"fire": ["icon_fire"]}
        document = Document(Path("labels.yaml"), "labels", labels)
        problems = check_labels(document, self._parts(self._base_parts()), self.place)
        self.assertEqual(
            [item.split(":", 1)[1] for item in problems],
            [
                "/tasks_pack: 'other' but the scenario selects 'robosub_2026'",
                "/tasks/torpedo/blood: 'icon_blood' matches no part of task 'torpedo'",
                "/tasks/torpedo/fire_too: part 'icon_fire' is already class 'fire'",
                "/tasks/bins/magnet/when/indicator: 'blue' never shown (task: green, red)",
                "/tasks/nowhere: no task 'nowhere' in tasks pack 'robosub_2026'",
            ],
        )

    def test_dataset_frames(self) -> None:
        dataset = _dataset()
        dataset["tasks"]["torpedo"]["sampler"]["frame"] = ["task", "slalom_front"]
        dataset["tasks"]["nowhere"] = copy.deepcopy(dataset["tasks"]["torpedo"])
        problems = check_dataset(Document(Path("d.yaml"), "dataset", dataset), self.place)
        self.assertEqual(
            [item.split(":", 1)[1] for item in problems],
            [
                "/tasks/torpedo/sampler/frame: task 'torpedo' has no frame 'slalom_front'",
                "/tasks/nowhere: no task 'nowhere' in tasks pack 'robosub_2026'",
            ],
        )

    def test_classes_in_no_model(self) -> None:
        labels = _labels()
        labels["models"]["ffc"]["classes"].append("blood")
        labels["tasks"]["torpedo"]["hammer_wrench"] = ["icon_blood"]  # typo of a class
        labels["classes"] = {"circel": {"shape": "outer"}}
        document = Document(Path("labels.yaml"), "labels", labels)
        problems = check_labels(document, self._parts(self._base_parts()), self.place)
        self.assertEqual(
            [item.split(":", 1)[1] for item in problems[:2]],
            [
                "/tasks/torpedo/hammer_wrench: class 'hammer_wrench' is in no model's classes",
                "/classes/circel: class 'circel' is in no model's classes",
            ],
        )
        self.assertEqual(
            label_warnings(document),
            ["labels.yaml:/models/ffc: class 'blood' has no mapping in any task"],
        )

    def test_placement_groups(self) -> None:
        dataset = _dataset()
        dataset["randomize"] = {"placement": {"groups": [["surface", "table"], ["table", "nope"]]}}
        self.assertEqual(check_data("dataset", dataset), [])
        problems = check_dataset(Document(Path("d.yaml"), "dataset", dataset), self.place)
        self.assertEqual(
            [item.split(":", 1)[1] for item in problems],
            [
                "/randomize/placement/groups/1: task 'table' is already in a group",
                "/randomize/placement/groups/1: no task 'nope' in tasks pack 'robosub_2026'",
            ],
        )


class ClassMapTests(unittest.TestCase):
    def test_classify(self) -> None:
        labels = _labels()
        labels["tasks"]["bins"] = {
            "fire": ["icon_*"],
            "magnet": {"parts": ["magnet_cover"], "when": {"indicator": "red"}},
        }
        classes = ClassMap(labels, "ffc")
        self.assertEqual(classes.classify("torpedo", "ring", None), 1)
        self.assertEqual(classes.classify("bins", "icon_fire", None), 0)
        self.assertIsNone(classes.classify("bins", "magnet_cover", "red"))  # not an ffc class
        self.assertIsNone(classes.classify("slalom", "pole_red", None))
        parts = {"bins": {"icon_fire", "magnet_cover"}, "torpedo": {"ring", "icon_blood"}}
        self.assertEqual(
            classes.labelled(parts),
            [{"task": "torpedo", "part": "ring"}, {"task": "bins", "part": "icon_fire"}],
        )

    def test_classes_outside_the_model_never_shadow_later_rules(self) -> None:
        labels = _labels()
        labels["models"]["dfc"] = {"camera": "dfc", "max_range_m": 3.0, "classes": ["magnet"]}
        labels["tasks"]["bins"] = {
            "magnet": {"parts": ["magnet_cover"], "when": {"indicator": "red"}},
            "fire": ["magnet_*"],
        }
        classes = ClassMap(labels, "ffc")  # no magnet: the red cover falls through to fire
        self.assertEqual(classes.classify("bins", "magnet_cover", "red"), 0)
        self.assertEqual(
            classes.labelled({"bins": {"magnet_cover"}}), [{"task": "bins", "part": "magnet_cover"}]
        )


class ClassOrderTests(unittest.TestCase):
    CONFIG = """\
/**/yolo_orientation:
  ros__parameters:
    ffc_class_id_map: "{
      0: 'fire',
      1: 'circle'
    }"
    # dfc_class_id_map: "{0: 'fire'}"
"""

    def test_matching_and_mismatching_maps(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "yolo.yaml"
            config.write_text(self.CONFIG)
            labels = _labels()
            self.assertEqual(class_order_problems(labels, config)[0], [])
            labels["models"]["ffc"]["classes"].reverse()
            problems, _ = class_order_problems(labels, config)
            self.assertEqual(
                problems,
                [
                    "model 'ffc': classes differ from ffc_class_id_map",
                    "  id 0: label pack 'circle', config 'fire'",
                    "  id 1: label pack 'fire', config 'circle'",
                ],
            )
            labels_file = Path(directory) / "labels.yaml"
            labels_file.write_text(
                "kind: labels\nid: t\ntasks_pack: x\n"
                "models: {ffc: {camera: ffc, max_range_m: 5, classes: [circle, fire]}}\n"
                "tasks: {torpedo: {fire: [icon_fire]}}\n"
            )
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                code = main(["check-classes", str(labels_file), "--yolo-config", str(config)])
            self.assertEqual(code, 1)
            self.assertIn("id 0: label pack 'circle'", stderr.getvalue())


@unittest.skipUnless((LABELS / "labels.yaml").is_file() and PARTS.is_file(), "no UWRT label pack")
class RealContentTests(unittest.TestCase):
    def test_label_pack_parts_and_specs_cross_check(self) -> None:
        labels = load_document(LABELS, "labels")
        parts = load_document(PARTS, "parts")
        place = course(resolve_scenario(PACKS / "scenarios" / "talos_uwrt"))
        self.assertEqual(check_parts(parts, place), [])
        self.assertEqual(check_labels(labels, parts, place), [])
        for folder in sorted(DATASETS.glob("*/dataset.yaml")):
            dataset = load_document(folder, "dataset")
            self.assertEqual(check_dataset(dataset, place), [], folder)

    @unittest.skipUnless(YOLO_CONFIG.is_file(), "UWRT detector config not in this checkout")
    def test_models_match_the_deployed_detector(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            code = main(["check-classes", str(LABELS), "--yolo-config", str(YOLO_CONFIG)])
        self.assertEqual(code, 0)
        self.assertIn("model 'ffc': 11 classes match", stdout.getvalue())
        self.assertIn("model 'dfc': 8 classes match", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
