"""Pack loading, lossless saving, scenario resolution, manifests and definition exports."""

import contextlib
import hashlib
import io
import json
import shutil
import tempfile
import unittest
from pathlib import Path

from jsonschema import Draft202012Validator
from nereus.datasets._documents import DATASET_KINDS
from nereus.packs import (
    DOCUMENT_KINDS,
    PackError,
    load_pack,
    registry,
    resolve_scenario,
    schema,
    type_catalog,
)
from nereus.packs.__main__ import main
from nereus.packs._document import read_yaml
from test_packs_fixtures import write_generic_packs

TALOS = Path(__file__).resolve().parents[2] / "content" / "packs"


class DocumentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.scenario = write_generic_packs(self.root)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_unedited_save_is_byte_exact(self) -> None:
        for path in sorted(self.root.rglob("*.yaml")):
            original = path.read_bytes()
            document = load_pack(path)
            target = document.save(self.root / "copy.yaml")
            self.assertEqual(target.read_bytes(), original, path.name)

    def test_folder_selects_canonical_file(self) -> None:
        document = load_pack(self.root / "robot")
        self.assertEqual(document.kind, "robot")
        self.assertEqual(document.path.name, "robot.yaml")

    def test_edit_preserves_comments_order_and_metadata(self) -> None:
        document = load_pack(self.root / "robot")
        order = list(document.data)
        document.data["body"]["parameters"]["mass_kg"] = 21.5
        document.save()
        text = (self.root / "robot" / "robot.yaml").read_text()
        self.assertTrue(text.startswith("# Synthetic four-thruster robot;"))
        reloaded = load_pack(self.root / "robot")
        self.assertEqual(list(reloaded.data), order)
        self.assertEqual(reloaded.plain()["body"]["parameters"]["mass_kg"], 21.5)
        self.assertEqual(
            reloaded.plain()["metadata"],
            {"author": "tests", "note": "free non-executable annotation"},
        )
        before = load_pack(self.root / "pool").plain()
        self.assertEqual(before, load_pack(self.root / "pool").plain())

    def test_invalid_edit_is_rejected_on_reload(self) -> None:
        document = load_pack(self.root / "robot")
        document.data["body"]["parameters"]["unexpected"] = 1
        document.save()
        with self.assertRaises(PackError):
            load_pack(self.root / "robot")


class ScenarioTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_generic_sensor_only_scenario(self) -> None:
        resolved = resolve_scenario(write_generic_packs(self.root))
        self.assertEqual(resolved.robot["id"], "synth")
        assert resolved.bridge is not None
        self.assertNotIn("thrusters", resolved.bridge)
        self.assertEqual([item["id"] for item in resolved.task_definitions], ["hoop"])
        self.assertEqual(resolved.run_options, {"timed": False})
        self.assertTrue(all(source.is_file() for source in resolved.sources))

    def test_bridge_is_optional(self) -> None:
        resolved = resolve_scenario(write_generic_packs(self.root, bridge=False))
        self.assertIsNone(resolved.bridge)

    def test_dump_is_reproducible_and_location_independent(self) -> None:
        first = write_generic_packs(self.root / "a")
        one = resolve_scenario(first).dump(self.root / "one.json").read_bytes()
        two = resolve_scenario(first).dump(self.root / "two.json").read_bytes()
        self.assertEqual(one, two)
        shutil.copytree(self.root / "a", self.root / "moved")
        moved = resolve_scenario(self.root / "moved" / "scenario")
        self.assertEqual(moved.dump(self.root / "three.json").read_bytes(), one)
        manifest = json.loads(one)
        claimed = manifest.pop("content_sha256")
        canonical = json.dumps(manifest, sort_keys=True, separators=(",", ":"))
        self.assertEqual(hashlib.sha256(canonical.encode()).hexdigest(), claimed)
        for source in manifest["sources"]:
            path = (self.root / "a" / "scenario" / source["path"]).resolve()
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), source["sha256"])

    def test_any_source_change_changes_manifest_identity(self) -> None:
        scenario = write_generic_packs(self.root)
        before = resolve_scenario(scenario).manifest()["content_sha256"]
        pool = self.root / "pool" / "pool.yaml"
        pool.write_text(pool.read_text() + "# trailing comment only\n")
        after = resolve_scenario(scenario).manifest()
        self.assertNotEqual(after["content_sha256"], before)

    def test_talos_packs_resolve_and_round_trip(self) -> None:
        resolved = resolve_scenario(TALOS / "scenarios" / "talos_uwrt")
        self.assertEqual(resolved.run_options["role"], "repair")
        for path in sorted(TALOS.rglob("*.yaml")):
            # Dataset documents (parts, labels, dataset) are not simulator packs; test_datasets_* covers them.
            if path.name != "exceptions.yaml" and read_yaml(path).get("kind") not in DATASET_KINDS:
                self.assertEqual(load_pack(path).dumps(), path.read_text(), path.name)


class DefinitionTests(unittest.TestCase):
    def test_registry_results_cannot_mutate_loader_definitions(self) -> None:
        original = registry()
        edited = registry()
        category = next(iter(edited))
        edited[category].clear()
        self.assertEqual(registry(), original)

    def test_document_schemas_are_valid_and_self_contained(self) -> None:
        for kind in DOCUMENT_KINDS:
            document = schema(kind)
            Draft202012Validator.check_schema(document)
            text = json.dumps(document)
            self.assertNotIn(".schema.json", text)
            self.assertNotIn("x-typed", text)

    def test_registry_matches_every_typed_schema_enum(self) -> None:
        seen: dict[str, list[str]] = {}

        def visit(node: object) -> None:
            if isinstance(node, dict):
                properties = node.get("properties", {})
                enum = properties.get("type", {}).get("enum")
                if enum is not None and "parameters" in properties and node.get("allOf"):
                    for category, types in registry().items():
                        if list(types) == enum:
                            seen[category] = enum
                    self.assertIn(enum, [list(types) for types in registry().values()])
                for value in node.values():
                    visit(value)
            elif isinstance(node, list):
                for value in node:
                    visit(value)

        for kind in DOCUMENT_KINDS:
            visit(schema(kind))
        self.assertEqual(sorted(seen), sorted(registry()))

    def test_type_catalog_resolves_every_type(self) -> None:
        catalog = type_catalog()
        Draft202012Validator.check_schema(catalog)
        for category, types in catalog["categories"].items():
            self.assertEqual(list(types), list(registry()[category]))
            for entry in types.values():
                name = entry["$ref"].removeprefix("#/$defs/")
                self.assertIn(name, catalog["$defs"])


class CommandLineTests(unittest.TestCase):
    def run_cli(self, *arguments: str) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main(list(arguments))
        return code, out.getvalue(), err.getvalue()

    def test_validate_schema_and_types(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            scenario = write_generic_packs(Path(directory))
            dump = Path(directory) / "resolved.json"
            code, out, _ = self.run_cli("validate", str(scenario), "--dump", str(dump))
            self.assertEqual(code, 0)
            self.assertIn("OK scenario", out)
            self.assertEqual(json.loads(dump.read_text())["format"], "nereus.resolved_scenario")
            (Path(directory) / "tasks" / "assets" / "hoop.dae").unlink()
            code, _, err = self.run_cli("validate", str(scenario))
            self.assertEqual(code, 1)
            self.assertIn("hoop_mesh", err)
        code, out, _ = self.run_cli("schema", "robot")
        self.assertEqual(json.loads(out)["properties"]["kind"], {"const": "robot"})
        code, out, _ = self.run_cli("types", "--json")
        self.assertIn("stereo_camera", json.loads(out)["categories"]["sensor"])


if __name__ == "__main__":
    unittest.main()
