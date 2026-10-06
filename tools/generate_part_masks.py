#!/usr/bin/env python3
"""Build the part-mask PNGs of a task pack's parts.yaml from the textures they label.

Each `textures[]` entry names a texture (a tasks-pack asset id) and a mask path; each part gives a
value and a seed pixel inside its emoji. Per part: foreground = opaque pixels (alpha >= 128) whose
max channel distance from the texture's background colour exceeds `threshold`, closed with a disk
of `close_px` so one emoji is one component; the component holding `seed_px`, its holes filled per
`fill`, is painted with `value`. `fill`: `small` (default; holes under 15 % of the component area,
so open loops like a helmet strap stay background), `all` (rings: the hole disk too) or `none`.
The background colour is the dominant opaque colour of the texture's border band (`background:
[r, g, b]` overrides it). Run from anywhere:

    python3 tools/generate_part_masks.py            # write the masks
    python3 tools/generate_part_masks.py --check    # exit 1 if a committed mask differs
    python3 tools/generate_part_masks.py PATH/parts.yaml

Requires numpy, Pillow and scipy.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import numpy.typing as npt
from PIL import Image  # type: ignore[import-untyped, unused-ignore]
from ruamel.yaml import YAML
from scipy import ndimage  # type: ignore[import-untyped, unused-ignore]

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PARTS = ROOT / "content/packs/tasks/robosub_2026/parts.yaml"
DEFAULT_THRESHOLD = 24  # 8-bit units; light-grey art (wrench, hammer head) is ~40 from white
DEFAULT_CLOSE_PX = 2
BORDER_FRACTION = 0.1  # outer band sampled for the background colour
SMALL_HOLE_FRACTION = 0.15  # fill: small fills holes below this fraction of the component area
FILLS = ("small", "all", "none")
PART_NAME = re.compile(r"[a-z0-9_]+")

# A per-pixel boolean image, indexed [y, x].
Mask = npt.NDArray[np.bool_]


@dataclass(frozen=True)
class Part:
    """One labelled part: its mask value, name, seed pixel and segmentation settings."""

    value: int
    name: str
    seed: tuple[int, int]  # x, y (texture px, top-left origin)
    threshold: int
    close_px: int
    fill: str  # one of FILLS


@dataclass(frozen=True)
class TextureParts:
    """One textures[] entry: the texture PNG, its output mask path and the parts painted on it."""

    texture: Path
    mask: Path
    background: tuple[float, float, float] | None
    parts: list[Part]


def load_yaml(path: Path) -> Any:
    return YAML(typ="safe").load(path.read_text())


def read_parts(parts_yaml: Path) -> list[TextureParts]:
    """Parse and check the textures of a parts document; texture ids resolve in the tasks pack."""
    pack = parts_yaml.parent
    doc = load_yaml(parts_yaml)
    tasks = load_yaml(pack / "tasks.yaml")
    if doc.get("kind") != "parts":
        raise ValueError(f"{parts_yaml}: kind must be parts")
    if doc.get("tasks") != tasks.get("id"):
        raise ValueError(f"{parts_yaml}: tasks {doc.get('tasks')!r} != pack id {tasks.get('id')!r}")
    assets = {a["id"]: a["path"] for a in tasks.get("assets", [])}
    result = []
    for entry in doc.get("textures", []):
        texture_id = entry["texture"]

        # Entry-level threshold/close_px are the defaults for its parts.
        if texture_id not in assets or not str(assets[texture_id]).endswith(".png"):
            raise ValueError(
                f"{parts_yaml}: texture {texture_id!r} is not a .png asset of the pack"
            )
        threshold = int(entry.get("threshold", DEFAULT_THRESHOLD))
        close_px = int(entry.get("close_px", DEFAULT_CLOSE_PX))
        background = entry.get("background")
        if background is not None:
            r, g, b = (float(c) for c in background)
            background = (r, g, b)

        parts = []
        for p in entry["parts"]:
            value, name = int(p["value"]), str(p["part"])
            if not 1 <= value <= 255:
                raise ValueError(f"{texture_id}: value {value} outside 1..255")
            if not PART_NAME.fullmatch(name):
                raise ValueError(f"{texture_id}: part name {name!r} must match [a-z0-9_]+")
            x, y = (int(c) for c in p["seed_px"])
            fill = str(p.get("fill", "small"))
            if fill not in FILLS:
                raise ValueError(f"{texture_id}: part {name} fill {fill!r} not in {FILLS}")
            parts.append(
                Part(
                    value=value,
                    name=name,
                    seed=(x, y),
                    threshold=int(p.get("threshold", threshold)),
                    close_px=int(p.get("close_px", close_px)),
                    fill=fill,
                )
            )

        values = [p.value for p in parts]
        if len(set(values)) != len(values):
            raise ValueError(f"{texture_id}: duplicate part values {values}")
        result.append(
            TextureParts(
                texture=pack / assets[texture_id],
                mask=pack / entry["mask"],
                background=background,
                parts=parts,
            )
        )
    return result


def background_colour(rgba: npt.NDArray[np.int32]) -> npt.NDArray[np.float64]:
    """Dominant opaque colour of the outer band (frame lines are thin; transparency is skipped)."""
    height, width = rgba.shape[:2]
    band = max(4, int(BORDER_FRACTION * min(height, width)))
    inside = np.ones((height, width), dtype=bool)
    inside[band:-band, band:-band] = False
    pixels = rgba[inside & (rgba[..., 3] >= 128)][:, :3]
    if len(pixels) == 0:
        raise ValueError("no opaque pixels in the border band; set background explicitly")

    # Bucket colours into 16 levels per channel, then take the median of the fullest bucket.
    bins = pixels // 16
    keys = (bins[:, 0] * 16 + bins[:, 1]) * 16 + bins[:, 2]
    mode = np.bincount(keys).argmax()
    result: npt.NDArray[np.float64] = np.median(pixels[keys == mode], axis=0)
    return result


def disk(radius: int) -> Mask:
    """A (2r+1)-square structuring element that is True inside the circle of `radius`."""
    yy, xx = np.mgrid[-radius : radius + 1, -radius : radius + 1]
    result: Mask = xx * xx + yy * yy <= radius * radius
    return result


def part_mask(rgba: npt.NDArray[np.int32], background: npt.NDArray[np.float64], part: Part) -> Mask:
    """The pixels of one part: the closed foreground component under its seed, holes per `fill`."""
    # Foreground: opaque and far enough from the background colour, closed into whole emoji.
    distance = np.abs(rgba[..., :3] - background).max(axis=-1)
    foreground = (rgba[..., 3] >= 128) & (distance > part.threshold)
    if part.close_px > 0:
        pad = part.close_px + 1  # closing must not erode at the image edge
        padded = np.pad(foreground, pad)
        foreground = ndimage.binary_closing(padded, disk(part.close_px))[pad:-pad, pad:-pad]

    # Keep the connected component under the seed; it must not touch the texture border.
    labels, _ = ndimage.label(foreground)
    x, y = part.seed
    height, width = foreground.shape
    if not (0 <= x < width and 0 <= y < height) or labels[y, x] == 0:
        raise ValueError(f"part {part.name} ({part.value}): seed {part.seed} is not on the art")
    component: Mask = labels == labels[y, x]
    edges = (component[0, :], component[-1, :], component[:, 0], component[:, -1])
    if any(edge.any() for edge in edges):  # merged with a frame line
        raise ValueError(f"part {part.name} ({part.value}): component reaches the texture border")

    # Fill holes: none, all, or (small) only those under SMALL_HOLE_FRACTION of its area.
    if part.fill == "none":
        return component
    holes = ndimage.binary_fill_holes(component) & ~component
    if part.fill == "small":
        hole_labels, count = ndimage.label(holes)
        sizes = ndimage.sum(holes, hole_labels, np.arange(1, count + 1))
        small = np.flatnonzero(sizes < SMALL_HOLE_FRACTION * component.sum()) + 1
        holes = np.isin(hole_labels, small)
    result: Mask = component | holes
    return result


def build_mask(entry: TextureParts) -> npt.NDArray[np.uint8]:
    """Paint every part of a texture into one 8-bit mask (0 = unlabelled), rejecting overlaps."""
    with Image.open(entry.texture) as image:
        rgba = np.asarray(image.convert("RGBA")).astype(np.int32)
    if entry.background is not None:
        background = np.asarray(entry.background, dtype=np.float64)
    else:
        background = background_colour(rgba)

    # Check every seed first, so an out-of-range seed is reported before segmentation.
    out = np.zeros(rgba.shape[:2], dtype=np.uint8)
    height, width = out.shape
    for part in entry.parts:
        x, y = part.seed
        if not (0 <= x < width and 0 <= y < height):
            raise ValueError(f"{entry.texture.name}: part {part.name} seed {part.seed} is outside")

    # A part may not swallow another part's seed or overlap pixels already painted.
    for part in entry.parts:
        mask = part_mask(rgba, background, part)
        for other in entry.parts:
            if other is not part and mask[other.seed[1], other.seed[0]]:
                raise ValueError(
                    f"{entry.texture.name}: part {part.name} ({part.value}) contains the seed of "
                    f"{other.name} ({other.value}); raise its threshold or lower close_px"
                )
        clash = np.unique(out[mask & (out != 0)])
        if len(clash):
            raise ValueError(
                f"{entry.texture.name}: part {part.name} ({part.value}) overlaps values "
                f"{clash.tolist()}; raise its threshold or lower close_px"
            )
        out[mask] = part.value
    return out


def read_mask(path: Path) -> npt.NDArray[np.uint8] | None:
    """The committed mask as an array, or None if it is missing or not 8-bit greyscale."""
    if not path.is_file():
        return None
    with Image.open(path) as image:
        if image.mode != "L":
            return None
        result: npt.NDArray[np.uint8] = np.asarray(image)
        return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("parts", nargs="?", type=Path, default=DEFAULT_PARTS)
    parser.add_argument("--check", action="store_true", help="exit 1 if a committed mask differs")
    args = parser.parse_args(argv)

    # Build every mask; --check compares against the committed file instead of writing it.
    stale = []
    for entry in read_parts(args.parts):
        mask = build_mask(entry)
        for part in entry.parts:
            if not (mask == part.value).any():
                raise ValueError(f"{entry.texture.name}: part {part.name} ({part.value}) is empty")
        if args.check:
            committed = read_mask(entry.mask)
            if committed is None or not np.array_equal(committed, mask):
                stale.append(entry.mask)
            continue
        entry.mask.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(mask, mode="L").save(entry.mask, optimize=True)
        print(f"wrote {entry.mask}")

    if stale:
        for path in stale:
            print(f"stale part mask: {path}", file=sys.stderr)
        print(f"re-run python3 tools/generate_part_masks.py {args.parts}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
