"""The RPAC dive well pack's generated meshes match their generator and the well recess that holds the stairs."""

import importlib.util
import tempfile
import unittest
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML

PACK = Path(__file__).resolve().parents[2] / "content" / "packs" / "pools" / "rpac_divewell"
ASSETS = PACK / "assets"


def load_generator() -> Any:  # the generator script module (its HERE is reassigned)
    spec = importlib.util.spec_from_file_location("rpac_make_meshes", ASSETS / "make_meshes.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_pool() -> dict[str, Any]:
    """The RPAC pool.yaml as plain data."""
    pool: dict[str, Any] = YAML(typ="safe").load((PACK / "pool.yaml").read_text())
    return pool


def vertices(obj: Path) -> list[tuple[float, ...]]:
    """The `v x y z` vertex positions of an OBJ file."""
    return [
        tuple(float(c) for c in line.split()[1:4])
        for line in obj.read_text().splitlines()
        if line.startswith("v ")
    ]


class RpacAssetsTest(unittest.TestCase):
    def test_committed_meshes_match_generator(self) -> None:
        """Regenerating every mesh into a temp dir reproduces the committed OBJ/MTL files."""
        gen = load_generator()
        with tempfile.TemporaryDirectory() as tmp:
            gen.HERE = Path(tmp)  # the script writes to HERE
            for name, build in gen.MESHES.items():
                build().write(name)
            produced = sorted(p.name for p in Path(tmp).iterdir())
            self.assertTrue(produced)

            # Byte-identical contents, then the same set of files.
            for name in produced:
                self.assertEqual(
                    (Path(tmp) / name).read_bytes(),
                    (ASSETS / name).read_bytes(),
                    f"{name} differs from make_meshes.py output; re-run `python3 make_meshes.py` in {ASSETS} and commit",
                )
            committed = sorted(p.name for p in ASSETS.iterdir() if p.suffix in (".obj", ".mtl"))
            self.assertEqual(
                produced,
                committed,
                "committed meshes and generated meshes differ in file set; re-run make_meshes.py",
            )

    def test_tower_stairs_fit_their_recess(self) -> None:
        """The stairs mesh fits the tower_stairs_well recess in width and depth."""
        gen = load_generator()
        recess = next(f for f in load_pool()["fixtures"] if f["id"] == "tower_stairs_well")
        self.assertEqual(recess["type"], "recess")
        self.assertEqual(
            recess["depth_m"],
            gen.WELL_DEPTH,
            "pool.yaml recess depth_m must equal make_meshes.WELL_DEPTH",
        )

        # The mesh x extent must fit the recess width, and its -y extent (into the wall) its depth.
        verts = vertices(ASSETS / "tower_stairs.obj")
        xs = [v[0] for v in verts]
        width = recess["to"][0] - recess["from"][0]
        self.assertGreaterEqual(
            width + 1e-6, max(xs) - min(xs), "stairs mesh is wider than the recess"
        )
        self.assertGreaterEqual(
            min(v[1] for v in verts), -recess["depth_m"], "stairs mesh runs behind the recess"
        )

    def test_assets_exist_and_meshes_are_declared(self) -> None:
        """Every declared asset file exists, and mesh fixtures use declared assets."""
        pool = load_pool()
        declared = {a["id"] for a in pool["assets"]}
        for asset in pool["assets"]:
            self.assertTrue((PACK / asset["path"]).is_file(), asset["path"])
        for fixture in pool["fixtures"]:
            if fixture.get("type") == "mesh":
                self.assertIn(fixture["asset"], declared, fixture["id"])


if __name__ == "__main__":
    unittest.main()
