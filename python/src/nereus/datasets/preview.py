"""Contact sheets for reviewing labels: class-coloured mask overlays, outlines, boxes and names."""

from __future__ import annotations

import fnmatch
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from nereus.packs import PackError

from .export import class_map, cv2_module, label_record, read_ids, records

# BGR, well separated; class id picks one (cycled). Blues and cyans last: pool water is blue.
# fmt: off
PALETTE = [
    (75, 25, 230), (75, 180, 60), (25, 225, 255), (48, 130, 245), (180, 30, 145),
    (230, 50, 240), (60, 245, 210), (212, 190, 250), (40, 110, 170), (255, 255, 255),
    (0, 0, 128), (195, 255, 170), (128, 128, 128), (0, 128, 128), (180, 215, 255),
    (128, 0, 0), (200, 130, 0), (240, 240, 70), (128, 128, 0), (255, 190, 220),
]
# fmt: on
ALPHA = 0.45

Image = NDArray[np.uint8]


def colour(cls: int) -> tuple[int, int, int]:
    return PALETTE[cls % len(PALETTE)]


def _pick(found: list[dict[str, Any]], count: int) -> list[dict[str, Any]]:
    """``count`` records spread evenly over the sorted list (every task shows up)."""
    if count >= len(found):
        return found
    step = len(found) / count
    return [found[int(index * step)] for index in range(count)]


def _text(image: Image, text: str, origin: tuple[int, int], fill: tuple[int, int, int]) -> int:
    """Draw ``text`` on a filled label at ``origin`` (bottom-left), kept inside; returns its width."""
    cv2 = cv2_module()
    font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1
    (width, height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x = min(max(origin[0], 0), max(image.shape[1] - width - 2, 0))
    y = min(max(origin[1], height + 2), image.shape[0] - baseline - 1)
    cv2.rectangle(image, (x, y - height - 2), (x + width + 2, y + baseline), fill, cv2.FILLED)
    luminance = 0.114 * fill[0] + 0.587 * fill[1] + 0.299 * fill[2]
    ink = (0, 0, 0) if luminance > 140 else (255, 255, 255)
    cv2.putText(image, text, (x + 1, y - 1), font, scale, ink, thickness, cv2.LINE_AA)
    return int(width) + 2


def _legend(names: list[str], width: int) -> Image:
    """Class ids and names in their colours, wrapped to ``width``."""
    cv2 = cv2_module()
    rows: list[list[tuple[int, str]]] = [[]]
    used = 4
    for cls, name in enumerate(names):
        (size, _), _ = cv2.getTextSize(f"{cls} {name}", cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        if rows[-1] and used + size + 10 > width:
            rows.append([])
            used = 4
        rows[-1].append((cls, f"{cls} {name}"))
        used += size + 10
    legend: Image = np.full((22 * len(rows) + 4, width, 3), 32, dtype=np.uint8)
    for row, items in enumerate(rows):
        x = 4
        for cls, text in items:
            x += _text(legend, text, (x, 22 * row + 18), colour(cls)) + 8
    return legend


def tile(render: Path, record: dict[str, Any], classes: Any, width: int) -> Image:
    """One sample: overlay at full resolution, scaled to ``width``, then boxes and names."""
    cv2 = cv2_module()
    path = render / record["image"]
    loaded = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if loaded is None:
        raise PackError(f"{path}: cannot read image")
    image = np.asarray(loaded, dtype=np.uint8)
    labelled = label_record(record, read_ids(render, record), classes)
    overlay = image.copy()
    for label in labelled.labels:
        overlay[label.mask] = colour(label.cls)
    blended = np.asarray(cv2.addWeighted(overlay, ALPHA, image, 1 - ALPHA, 0), dtype=np.uint8)
    scale = width / image.shape[1]
    height = max(1, round(image.shape[0] * scale))
    small = np.asarray(cv2.resize(blended, (width, height), interpolation=cv2.INTER_AREA))
    for label in labelled.labels:
        mask = np.asarray(
            cv2.resize(
                label.mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST
            )
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(small, contours, -1, colour(label.cls), 1, cv2.LINE_AA)
        rows, columns = np.nonzero(label.mask)
        x0, y0 = int(columns.min() * scale), int(rows.min() * scale)
        x1, y1 = (
            int(math.ceil((columns.max() + 1) * scale)),
            int(math.ceil((rows.max() + 1) * scale)),
        )
        cv2.rectangle(small, (x0, y0), (x1 - 1, y1 - 1), colour(label.cls), 1)
        _text(small, classes.names[label.cls], (x0, y0 - 2), colour(label.cls))
    caption = f"{record['name']}  {len(labelled.labels)} labels"
    if labelled.dropped_small:
        caption += f", {labelled.dropped_small} < min px"
    if labelled.crumbs:
        caption += f", {labelled.crumbs} crumbs"
    if labelled.far:
        caption += f"  SKIPPED: {len(labelled.far)} beyond {classes.max_range_m:g} m"
    if labelled.fragmented:
        caption += f"  SKIPPED: {len(labelled.fragmented)} fragmented"
    _text(small, caption, (0, height - 2), (0, 0, 200) if labelled.skipped else (40, 40, 40))
    if record.get("environment") is not None:
        _text(small, str(record["environment"]), (0, 16), (60, 60, 60))
    return small


def preview(
    render: Path,
    out: Path,
    *,
    count: int = 16,
    environments: list[str] | None = None,
    labels: Path | None = None,
    model: str | None = None,
    tile_width: int = 480,
    columns: int = 4,
) -> Path:
    """Write a contact sheet of ``count`` samples; returns its path."""
    cv2 = cv2_module()
    render = Path(render).resolve()
    classes, _ = class_map(render, labels, model)
    found = records(render)
    if environments is not None:
        found = [
            record
            for record in found
            if any(
                fnmatch.fnmatchcase(str(record.get("environment")), item) for item in environments
            )
        ]
    chosen = _pick(found, max(count, 1))
    if not chosen:
        raise PackError(f"{render}: no records to preview")
    tiles = [tile(render, record, classes, tile_width) for record in chosen]
    height = max(item.shape[0] for item in tiles)
    columns = min(columns, len(tiles))
    rows = math.ceil(len(tiles) / columns)
    gap = 4
    sheet: Image = np.full(
        (rows * (height + gap) + gap, columns * (tile_width + gap) + gap, 3), 32, dtype=np.uint8
    )
    for index, item in enumerate(tiles):
        row, column = divmod(index, columns)
        y, x = gap + row * (height + gap), gap + column * (tile_width + gap)
        sheet[y : y + item.shape[0], x : x + item.shape[1]] = item
    full: Image = np.concatenate([_legend(classes.names, sheet.shape[1]), sheet])
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), full, [cv2.IMWRITE_JPEG_QUALITY, 90]):
        raise PackError(f"{out}: cannot write the contact sheet")
    return out
