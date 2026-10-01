"""Planning renderer jobs from dataset specs on the real talos_uwrt scenario."""

import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from typing import Any

from nereus.datasets.__main__ import main
from nereus.datasets._environments import DEFAULTS, expand
from nereus.datasets.plan import Overrides, _randomize, apply_overrides, plan
from nereus.packs import PackError, resolve_scenario

ROOT = Path(__file__).resolve().parents[2]
PACKS = ROOT / "content" / "packs"
SCENARIO = PACKS / "scenarios" / "talos_uwrt"
FFC = PACKS / "datasets" / "uwrt_ffc_2026"
DFC = PACKS / "datasets" / "uwrt_dfc_2026"
HAS_CONTENT = (FFC / "dataset.yaml").is_file() and (PACKS / "labels/uwrt/labels.yaml").is_file()

LABELS = """\
kind: labels
id: test_labels
tasks_pack: robosub_2026
models:
  ffc: {camera: ffc, max_range_m: 4.5, classes: [fire, circle, magnet, slalom]}
  dfc: {camera: dfc, max_range_m: 3.0, classes: [pill]}
tasks:
  torpedo: {fire: [icon_fire], circle: ['r*']}
  slalom: {slalom: [pole_red]}
  bins: {magnet: {parts: [magnet_cover], when: {indicator: red}}}
  table: {pill: [icon_pill]}
"""


def _dataset(scenario: str) -> str:
    return f"""\
kind: dataset
id: test_ffc
labels: ../labels
model: ffc
scenarios: [{scenario}]
seed: 9
image: {{resolution_px: native, format: png}}
split: {{train: 0.5, val: 0.25, test: 0.25}}
acceptance: {{min_target_px: 40}}
tasks:
  torpedo:
    count: 7
    sampler: {{type: approach, frame: task, facing: [1, 0, 0], range_m: [0.8, 4.0], bearing_deg: 50}}
  slalom:
    count: 3
    sampler: {{type: approach, frame: [slalom_front, slalom_back], both_sides: true, range_m: [1, 3]}}
  table:
    count: 2
    sampler: {{type: overhead, altitude_m: [0.5, 1.0], radius_m: 0.3}}
background: {{count: 4, sampler: {{type: free, depth_m: [0.5, 1.5]}}}}
randomize:
  water: {{scattering: [0.1, 0.2]}}
  indicators: {{latched_probability: 0.5}}
"""


class PlanTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        (self.root / "labels").mkdir()
        (self.root / "labels" / "labels.yaml").write_text(LABELS)
        (self.root / "spec").mkdir()
        scenario = Path(os.path.relpath(SCENARIO, self.root / "spec")).as_posix()
        (self.root / "spec" / "dataset.yaml").write_text(_dataset(scenario))
        self.out = self.root / "out"

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_job_from_a_fixture_spec(self) -> None:
        result = plan(self.root / "spec", self.out)
        job = result.job
        self.assertEqual(json.loads((self.out / "job.json").read_text()), job)
        self.assertEqual(job["format"], "nereus.dataset_job.v1")
        self.assertEqual(
            (job["dataset"], job["seed"], job["output"]), ("test_ffc", 9, str(self.out))
        )
        self.assertEqual(
            job["camera"],
            {
                "sensor": "ffc",
                "resolution_px": [1920, 1200],
                "crop": "center",
                "format": "png",
                "jpeg_quality": 92,
                "robot_visuals": True,
            },
        )
        self.assertEqual(
            [(block["task"], block["count"]) for block in job["samples"]],
            [("torpedo", 7), ("slalom", 3), ("table", 2), (None, 4)],
        )
        self.assertEqual(job["samples"][1]["sampler"]["frame"], ["slalom_front", "slalom_back"])
        # Globs expand to parts; pill is not an ffc class; the magnet counts in any indicator state.
        self.assertEqual(
            job["labelled"],
            [
                {"task": "torpedo", "part": "icon_fire"},
                {"task": "torpedo", "part": "ring"},
                {"task": "slalom", "part": "pole_red"},
                {"task": "bins", "part": "magnet_cover"},
            ],
        )
        self.assertEqual(job["acceptance"]["max_range_m"], 4.5)
        self.assertEqual(job["acceptance"]["min_target_px"], 40)
        self.assertEqual(job["acceptance"]["max_attempts"], 200)
        self.assertEqual(job["acceptance"]["fragments"], "reject")  # the label pack default
        self.assertEqual(job["acceptance"]["min_fragment_px"], 25)
        self.assertEqual(job["acceptance"]["min_visible_px"], 25)
        randomize = job["randomize"]
        self.assertEqual(
            sorted(randomize),
            ["environment_mode", "environments", "indicators", "placement"],
        )
        (default,) = randomize["environments"]
        self.assertEqual((default["id"], default["weight"]), ("default", 1.0))
        # The absolute scattering replaces the default pool-relative scattering_scale.
        self.assertEqual(
            default["water"],
            {"tint_scale": [0.85, 1.15], "absorption_scale": [0.7, 1.4], "scattering": [0.1, 0.2]},
        )
        self.assertEqual(default["lighting"], DEFAULTS["lighting"])
        self.assertEqual(default["time_s"], [0, 600])
        self.assertEqual(randomize["environment_mode"], "weighted")
        self.assertEqual(randomize["indicators"], {"latched_probability": 0.5})
        self.assertEqual(randomize["placement"]["groups"], [])
        export = json.loads((self.out / "job" / "export.json").read_text())
        self.assertEqual(export["model"], "ffc")
        self.assertEqual(export["labels"], str((self.root / "labels/labels.yaml").resolve()))
        self.assertEqual(export["split"], {"train": 0.5, "val": 0.25, "test": 0.25})

    def test_scenarios_parts_and_paths(self) -> None:
        job = plan(self.root / "spec" / "dataset.yaml", self.out).job
        (scenario,) = job["scenarios"]
        self.assertEqual(scenario["id"], "talos_uwrt_repair_2026")
        resolved = json.loads(Path(scenario["resolved"]).read_text())
        expected = resolve_scenario(SCENARIO).manifest()
        self.assertEqual(resolved["content_sha256"], expected["content_sha256"])
        self.assertIn("asset_paths", resolved)
        textures = {Path(item["texture"]).name: item for item in job["parts"]["textures"]}
        torpedo = textures["Task4_ver1_Fixed.png"]
        self.assertTrue(Path(torpedo["texture"]).is_absolute())
        self.assertEqual(Path(torpedo["texture"]), Path(torpedo["texture"]).resolve())
        self.assertTrue(Path(torpedo["mask"]).is_file())
        self.assertEqual(torpedo["values"]["1"], "icon_fire")
        self.assertIsNone(torpedo["task"])
        slalom = next(item for item in job["parts"]["visuals"] if item["task"] == "slalom")
        self.assertEqual(slalom["split"], "none")
        self.assertEqual(slalom["materials"]["Material.001"], "pole_red")
        self.assertIsNone(slalom["part"])

    def test_cli_overrides(self) -> None:
        arguments = ["plan", str(self.root / "spec"), "--out", str(self.out)]
        arguments += ["--task", "torpedo", "--task", "table", "--count", "4", "--range-m", "1", "2"]
        arguments += [
            "--bearing-deg",
            "30",
            "--elevation-deg",
            "-5",
            "5",
            "--altitude-m",
            "0.6",
            "0.9",
        ]
        arguments += ["--resolution", "960x600", "--seed", "3"]
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            self.assertEqual(main(arguments), 0)
        self.assertIn("planned test_ffc: 8 samples, ffc 960x600, seed 3", stdout.getvalue())
        job = json.loads((self.out / "job.json").read_text())
        torpedo, table = job["samples"]
        self.assertEqual((torpedo["count"], table["count"]), (4, 4))
        self.assertEqual(torpedo["sampler"]["range_m"], [1.0, 2.0])
        self.assertEqual(torpedo["sampler"]["bearing_deg"], 30.0)
        self.assertEqual(torpedo["sampler"]["elevation_deg"], [-5.0, 5.0])
        self.assertNotIn("altitude_m", torpedo["sampler"])
        self.assertEqual(table["sampler"]["altitude_m"], [0.6, 0.9])
        self.assertNotIn("range_m", table["sampler"])
        self.assertEqual(job["camera"]["resolution_px"], [960, 600])
        self.assertEqual(job["seed"], 3)

    def test_override_rejections(self) -> None:
        def fails(overrides: Overrides) -> list[str]:
            with self.assertRaises(PackError) as caught:
                plan(self.root / "spec", self.out, overrides)
            return caught.exception.problems

        self.assertIn("not in the dataset", fails(Overrides(tasks=["gate"]))[0])
        self.assertIn("--altitude-m", fails(Overrides(tasks=["torpedo"], altitude_m=[1, 2]))[0])
        self.assertIn("range low", fails(Overrides(range_m=[3, 1]))[0])
        self.assertIn("--resolution", fails(Overrides(resolution="wide"))[0])
        only_background = apply_overrides(
            {"tasks": {"t": {"count": 1, "sampler": {"type": "free"}}}, "background": {}},
            Overrides(tasks=["background"]),
        )
        self.assertEqual(only_background["tasks"], {})
        self.assertIn("background", only_background)

    def test_replanning_never_mixes_renders(self) -> None:
        plan(self.root / "spec", self.out)
        (self.out / "records").mkdir()
        (self.out / "records" / "torpedo_000000.json").write_text("{}")
        plan(self.root / "spec", self.out)  # same job: the renderer may resume
        with self.assertRaises(PackError) as caught:
            plan(self.root / "spec", self.out, Overrides(seed=2))
        self.assertIn("holds renders of a different job", caught.exception.problems[0])

    def test_environments_list_sweep_and_overrides(self) -> None:
        spec = self.root / "spec" / "dataset.yaml"
        text = spec.read_text().split("randomize:")[0]
        spec.write_text(
            text
            + """randomize:
  water: {scattering_scale: [0.9, 1.1]}
  environments:
    list:
    - {id: nominal, weight: 3}
    - id: murky
      water: {absorption_scale: [1.4, 2.0], scattering_scale: [1.3, 1.6]}
      lighting: {caustics: [0, 0.3], profile: outdoor}
    sweep:
      lighting.exposure: [0.7, 1.3]
"""
        )
        job = plan(spec, self.out).job
        found = job["randomize"]["environments"]
        self.assertEqual(
            [(item["id"], item["weight"]) for item in found],
            [
                ("nominal/lighting.exposure=0.7", 1.5),
                ("nominal/lighting.exposure=1.3", 1.5),
                ("murky/lighting.exposure=0.7", 0.5),
                ("murky/lighting.exposure=1.3", 0.5),
            ],
        )
        nominal, murky = found[0], found[3]
        self.assertEqual(nominal["water"]["scattering_scale"], [0.9, 1.1])  # the spec's base
        self.assertEqual(nominal["lighting"]["exposure"], 0.7)
        self.assertEqual(nominal["lighting"]["caustics"], [0.0, 1.3])  # default
        self.assertEqual(murky["water"]["scattering_scale"], [1.3, 1.6])
        self.assertEqual(murky["lighting"]["exposure"], 1.3)
        self.assertEqual(murky["lighting"]["profile"], "outdoor")
        self.assertEqual(sorted(nominal), ["id", "image", "lighting", "time_s", "water", "weight"])

        overrides = Overrides(environments=["murky*"], environment_mode="sweep")
        job = plan(spec, self.root / "murky", overrides).job
        self.assertEqual(len(job["randomize"]["environments"]), 2)
        self.assertEqual(job["randomize"]["environment_mode"], "sweep")
        with self.assertRaises(PackError) as caught:
            plan(spec, self.root / "none", Overrides(environments=["clear"]))
        self.assertIn("match none of nominal/lighting.exposure=0.7", caught.exception.problems[0])

        spec.write_text(spec.read_text().replace("lighting.exposure", "lighting.exposur"))
        with self.assertRaises(PackError) as caught:
            plan(spec, self.root / "typo")
        self.assertIn("'exposur' was unexpected", "\n".join(caught.exception.problems))

    def test_weight_zero_environments_are_dropped(self) -> None:
        randomize = {
            "environments": {"mode": "sweep", "list": [{"id": "on"}, {"id": "off", "weight": 0}]}
        }
        result = _randomize(randomize, Overrides())
        self.assertEqual([item["id"] for item in result["environments"]], ["on"])
        self.assertEqual(result["environment_mode"], "sweep")
        with self.assertRaises(PackError):
            _randomize(randomize, Overrides(environments=["off"]))

    def test_environment_layers(self) -> None:
        found, mode = expand(
            {
                "water": {"tint_rgb": [0.1, 0.2, 0.3]},
                "environments": {
                    "mode": "sweep",
                    "list": [{"id": "a"}, {"id": "b", "water": {"tint_scale": 1.1}}],
                },
            }
        )
        self.assertEqual(mode, "sweep")
        self.assertEqual(found[0]["water"]["tint_rgb"], [0.1, 0.2, 0.3])
        self.assertNotIn("tint_scale", found[0]["water"])
        self.assertEqual(found[1]["water"]["tint_scale"], 1.1)  # the higher layer's form wins
        self.assertNotIn("tint_rgb", found[1]["water"])
        sweep_only, _ = expand({"environments": {"sweep": {"time_s": [0, 300]}}})
        self.assertEqual([item["id"] for item in sweep_only], ["base/time_s=0", "base/time_s=300"])

    def test_bad_frame_and_model(self) -> None:
        spec = self.root / "spec" / "dataset.yaml"
        spec.write_text(spec.read_text().replace("slalom_back]", "gate_repair]"))
        with self.assertRaises(PackError) as caught:
            plan(spec, self.out)
        self.assertIn("task 'slalom' has no frame 'gate_repair'", caught.exception.problems[0])
        spec.write_text(spec.read_text().replace("model: ffc", "model: bfc"))
        with self.assertRaises(PackError) as caught:
            plan(spec, self.out)
        self.assertTrue(any("no model 'bfc'" in item for item in caught.exception.problems))


@unittest.skipUnless(HAS_CONTENT, "UWRT dataset specs not present")
class RealSpecTests(unittest.TestCase):
    def _plan(self, spec: Path, **overrides: Any) -> dict[str, Any]:
        with tempfile.TemporaryDirectory() as directory:
            return plan(spec, Path(directory), Overrides(**overrides)).job

    def test_ffc_spec(self) -> None:
        job = self._plan(FFC)
        self.assertEqual(job["camera"]["sensor"], "ffc")
        self.assertEqual(job["acceptance"]["max_range_m"], 5.0)
        self.assertEqual(job["randomize"]["placement"]["groups"], [["surface", "table"]])
        found = job["randomize"]["environments"]
        self.assertGreaterEqual(len(found), 3)
        for item in found:  # pool-relative water: never an absolute scattering
            self.assertNotIn("scattering", item["water"], item["id"])
        self.assertEqual(job["acceptance"]["fragments"], "reject")
        tasks = [block["task"] for block in job["samples"]]
        self.assertEqual(tasks[-1], None)
        self.assertIn("torpedo", tasks)
        labelled = {(item["task"], item["part"]) for item in job["labelled"]}
        self.assertIn(("torpedo", "ring"), labelled)
        self.assertIn(("bins", "magnet_cover"), labelled)
        self.assertNotIn(("slalom", "pole_white"), labelled)
        self.assertNotIn(("table", "icon_pill"), labelled)
        small = self._plan(FFC, tasks=["torpedo"], count=4, resolution="960x600")
        self.assertEqual([(b["task"], b["count"]) for b in small["samples"]], [("torpedo", 4)])

    @unittest.skipUnless((DFC / "dataset.yaml").is_file(), "no dfc spec")
    def test_dfc_spec(self) -> None:
        job = self._plan(DFC)
        self.assertEqual(job["camera"]["sensor"], "dfc")
        self.assertEqual(job["acceptance"]["max_range_m"], 3.0)
        labelled = {(item["task"], item["part"]) for item in job["labelled"]}
        self.assertIn(("table", "icon_pill"), labelled)
        self.assertIn(("bins", "icon_fire"), labelled)
        self.assertNotIn(("bins", "magnet_cover"), labelled)
        overhead = [b["sampler"] for b in job["samples"] if b["sampler"]["type"] == "overhead"]
        self.assertTrue(overhead)


if __name__ == "__main__":
    unittest.main()
