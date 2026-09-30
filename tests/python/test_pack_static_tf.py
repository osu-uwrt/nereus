"""Static TF stays robot data plus bridge names, with one owner per child."""

import copy
import tempfile
import unittest
from pathlib import Path
from typing import Any

from nereus.packs import PackError, load_pack, resolve_scenario
from test_packs_fixtures import write_generic_packs


class PackStaticTfTests(unittest.TestCase):
    def setUp(self) -> None:
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.scenario = write_generic_packs(self.root)
        self.path = self.root / "bridge" / "bridge.yaml"
        document = load_pack(self.path)
        document.data["frame_names"] = {"base_link": "auv/body", "imu_mount": "auv/sensor"}
        document.data["tf"] = {"publish": [], "lookup": [], "never_publish": [], "static": [{
            "parent": "auv/body", "child": "auv/sensor",
            "from_frame": "base_link", "to_frame": "imu_mount"}]}
        document.save()

    def test_generic_fixed_transform_resolves_and_round_trips(self) -> None:
        resolved = resolve_scenario(self.scenario)
        assert resolved.bridge is not None
        self.assertEqual(resolved.bridge["tf"]["static"][0]["to_frame"], "imu_mount")
        original = self.path.read_bytes()
        load_pack(self.path).save()
        self.assertEqual(self.path.read_bytes(), original)

    def test_invalid_native_frame_and_conflicting_alias_fail_resolution(self) -> None:
        for field, value in (("from_frame", "missing"), ("to_frame", "world"),
                             ("parent", "wrong"), ("child", "wrong")):
            with self.subTest(field=field):
                doc = load_pack(self.path)
                old = doc.data["tf"]["static"][0][field]
                doc.data["tf"]["static"][0][field] = value
                doc.save()
                with self.assertRaises(PackError):
                    resolve_scenario(self.scenario)
                doc.data["tf"]["static"][0][field] = old
                doc.save()

    def test_one_tf_owner_no_cycles_and_never_publish_apply_to_static(self) -> None:
        doc = load_pack(self.path)
        original = copy.deepcopy(doc.data["tf"])
        cases: list[dict[str, Any]] = [
            {"lookup": [{"parent": "foreign", "child": "auv/sensor"}]},
            {"lookup": [{"parent": "auv/sensor", "child": "auv/body"}]},
            {"never_publish": ["auv/*"]},
        ]
        for changes in cases:
            with self.subTest(changes=changes):
                doc.data["tf"] = {**copy.deepcopy(original), **changes}
                doc.save()
                with self.assertRaises(PackError):
                    load_pack(self.path)
        doc.data["tf"] = original
        doc.save()


if __name__ == "__main__":
    unittest.main()
