"""Renderer records + label pack -> YOLO detect (bbox), segment (polygon) or OBB datasets.

The label pack is applied here: instance -> class by task part patterns and indicator state,
classes the model lacks dropped, ``shape: outer`` holes filled, slivers under ``min_visible_px``
dropped, and images with a labelled instance beyond the model's range skipped.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

from nereus.packs import PackError, resolve_scenario

from ._documents import check_labels, course, load_document
from ._mapping import ClassMap

FORMATS = ("yolo-seg", "yolo-bbox", "yolo-obb")
SPLITS = ("train", "val", "test")
POLYGON_EPSILON_PX = 0.75
BALANCE_TOLERANCE = 0.2  # team training guide: classes within 20 % of the mean

Mask = NDArray[np.bool_]
Points = NDArray[np.float64]


def cv2_module() -> Any:
    """OpenCV, imported on first use so planning works without the ``datasets`` extra."""
    try:
        import cv2
    except ImportError:
        raise PackError(
            "export and preview need OpenCV: pip install 'nereus[datasets]' "
            "(opencv-python-headless)"
        ) from None
    return cv2


# ------------------------------------------------------------------ records


def read_json(path: Path) -> dict[str, Any]:
    try:
        result: dict[str, Any] = json.loads(path.read_text("utf-8"))
    except (OSError, ValueError) as error:
        raise PackError(f"{path}: cannot read JSON ({error})") from None
    return result


def records(render: Path) -> list[dict[str, Any]]:
    """Every accepted sample's record, by name."""
    folder = render / "records"
    if not folder.is_dir():
        raise PackError(f"{render}: no records/ folder (render the job first)")
    return [read_json(path) for path in sorted(folder.glob("*.json"))]


def read_ids(render: Path, record: dict[str, Any]) -> NDArray[np.uint16]:
    cv2 = cv2_module()
    path = render / record["ids"]
    ids = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if ids is None or ids.ndim != 2:
        raise PackError(f"{path}: not a single-channel id map")
    camera = record["camera"]
    if ids.shape != (camera["height"], camera["width"]):
        raise PackError(
            f"{path}: {ids.shape[1]}x{ids.shape[0]} but the record says "
            f"{camera['width']}x{camera['height']}"
        )
    return np.asarray(ids, dtype=np.uint16)


def class_map(
    render: Path, labels: Path | None, model: str | None
) -> tuple[ClassMap, dict[str, Any]]:
    """The label-pack model to export with: arguments, else what the plan recorded."""
    settings_path = render / "job" / "export.json"
    settings = read_json(settings_path) if settings_path.is_file() else {}
    labels_path = labels if labels is not None else settings.get("labels")
    model = model if model is not None else settings.get("model")
    if labels_path is None or model is None:
        raise PackError(f"{render}: no job/export.json: pass --labels and --model")
    document = load_document(Path(labels_path), "labels")
    if labels is not None and settings.get("scenarios"):
        # A label pack other than the planned one: check it against the rendered course.
        place = course(resolve_scenario(Path(settings["scenarios"][0])))
        problems = check_labels(document, load_document(place.parts_file, "parts"), place)
        if problems:
            raise PackError(problems)
    return ClassMap(document.data, model), settings


# ------------------------------------------------------------------ masks -> YOLO geometry


def fill_holes(mask: Mask) -> Mask:
    cv2 = cv2_module()
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    filled = np.zeros(mask.shape, np.uint8)
    cv2.drawContours(filled, contours, -1, 1, thickness=cv2.FILLED)
    return np.asarray(filled, dtype=bool)


def polygons(mask: Mask) -> list[Points]:
    """External contours (pixel-index coordinates) simplified by approxPolyDP, left to right."""
    cv2 = cv2_module()
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    result: list[Points] = []
    for contour in contours:
        approximate = cv2.approxPolyDP(contour, POLYGON_EPSILON_PX, True)
        points: Points = np.asarray(approximate, dtype=np.float64).reshape(-1, 2)
        if len(points) < 3:
            # A 1 px wide piece collapses to a line: use its pixel-edge rectangle so seg labels
            # the same instances as bbox and obb.
            x0, y0 = points.min(axis=0)
            x1, y1 = points.max(axis=0) + 1
            points = np.array([[x0, y0], [x0, y1], [x1, y1], [x1, y0]], dtype=np.float64)
        result.append(points)
    result.sort(key=lambda points: (float(points[:, 0].min()), float(points[:, 1].min())))
    return result


def _nearest(a: Points, b: Points) -> tuple[int, int]:
    distances = ((a[:, None, :] - b[None, :, :]) ** 2).sum(-1)
    first, second = np.unravel_index(int(np.argmin(distances)), distances.shape)
    return int(first), int(second)


def merge_multi_segment(segments: list[Points]) -> Points:
    """One polygon through every segment, Ultralytics ``merge_multi_segment`` style.

    Neighbouring segments are joined at their closest points by a zero-width bridge walked out on
    a forward pass and back on a return pass. Unlike Ultralytics, a middle segment keeps its
    orientation (their reversed middle segments index the flipped array with unflipped indices).
    """
    if len(segments) == 1:
        return segments[0]
    links: list[list[int]] = [[] for _ in segments]
    for index in range(1, len(segments)):
        first, second = _nearest(segments[index - 1], segments[index])
        links[index - 1].append(first)
        links[index].append(second)
    last = len(segments) - 1
    forward: list[Points] = []
    backward: list[Points] = []
    for index, (segment, link) in enumerate(zip(segments, links)):
        entry = link[0]
        rolled = np.roll(segment, -entry, axis=0)
        closed = np.concatenate([rolled, rolled[:1]])
        if index in (0, last):
            forward.append(closed)
            continue
        exit_ = (link[1] - entry) % len(segment)
        forward.append(closed[: exit_ + 1])
        backward.append(closed[exit_:])
    return np.concatenate(forward + backward[::-1])


def _numbers(values: NDArray[np.float64]) -> str:
    return " ".join(f"{value:.6f}" for value in np.clip(values, 0.0, 1.0).reshape(-1))


def bbox_line(mask: Mask, cls: int) -> str | None:
    """``cls cx cy w h`` of the pixels' edges."""
    rows, columns = np.nonzero(mask)
    if len(rows) == 0:
        return None
    height, width = mask.shape
    x0, x1 = float(columns.min()), float(columns.max()) + 1
    y0, y1 = float(rows.min()), float(rows.max()) + 1
    box = np.array(
        [(x0 + x1) / 2 / width, (y0 + y1) / 2 / height, (x1 - x0) / width, (y1 - y0) / height]
    )
    return f"{cls} {_numbers(box)}"


def seg_line(mask: Mask, cls: int) -> str | None:
    """``cls x1 y1 ...``: contours in pixel-index coordinates (as Ultralytics' converters)."""
    found = polygons(mask)
    if not found:
        return None
    height, width = mask.shape
    points = merge_multi_segment(found) / np.array([width, height], dtype=np.float64)
    return f"{cls} {_numbers(points)}"


def _ordered_corners(corners: Points) -> Points:
    """Clockwise on screen (y down) from the top-most, then left-most corner."""
    centre = corners.mean(axis=0)
    angles = np.arctan2(corners[:, 1] - centre[1], corners[:, 0] - centre[0])
    corners = corners[np.argsort(angles, kind="stable")]
    rounded = np.round(corners, 6)
    start = min(range(4), key=lambda index: (rounded[index, 1], rounded[index, 0]))
    return np.roll(corners, -start, axis=0)


def obb_line(mask: Mask, cls: int) -> str | None:
    """``cls x1 y1 ... x4 y4``: minimum-area rectangle around the pixels' corners."""
    cv2 = cv2_module()
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    centres = np.concatenate([np.asarray(c, dtype=np.float32).reshape(-1, 2) for c in contours])
    offsets = np.array([[0, 0], [1, 0], [0, 1], [1, 1]], dtype=np.float32)
    points = (centres[:, None, :] + offsets[None, :, :]).reshape(-1, 2)
    box = np.asarray(cv2.boxPoints(cv2.minAreaRect(points)), dtype=np.float64)
    height, width = mask.shape
    corners = _ordered_corners(box) / np.array([width, height], dtype=np.float64)
    return f"{cls} {_numbers(corners)}"


LINES = {"yolo-seg": seg_line, "yolo-bbox": bbox_line, "yolo-obb": obb_line}


# ------------------------------------------------------------------ labelling one record


@dataclass
class Label:
    cls: int
    mask: Mask
    instance: dict[str, Any]


@dataclass
class Labelled:
    """One record's labels after the label pack, or why the image is skipped."""

    labels: list[Label] = field(default_factory=list)
    dropped_small: int = 0
    crumbs: int = 0  # pieces under min_fragment_px removed from labelled masks
    far: list[dict[str, Any]] = field(default_factory=list)  # labelled instances beyond range
    fragmented: list[dict[str, Any]] = field(default_factory=list)  # with fragments: reject

    @property
    def skipped(self) -> bool:
        return bool(self.far or self.fragmented)


def pieces(mask: Mask, min_px: int) -> tuple[list[Mask], int]:
    """8-connected pieces of at least ``min_px`` pixels (largest first), and the crumb count."""
    cv2 = cv2_module()
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    areas = np.asarray(stats, dtype=np.int64)[1:, cv2.CC_STAT_AREA]
    order = sorted(range(len(areas)), key=lambda index: -int(areas[index]))
    kept = [index + 1 for index in order if areas[index] >= min_px]
    components = np.asarray(labels)
    return [components == index for index in kept], int(count) - 1 - len(kept)


def label_record(record: dict[str, Any], ids: NDArray[np.uint16], classes: ClassMap) -> Labelled:
    sensor = record["camera"]["sensor"]
    if sensor != classes.camera:
        raise PackError(
            f"{record['name']}: rendered by camera '{sensor}', but model '{classes.model}' "
            f"is for camera '{classes.camera}'"
        )
    result = Labelled()
    for instance in record["instances"]:
        cls = classes.classify(instance["task"], instance["part"], instance.get("indicator"))
        if cls is None:
            continue
        if instance["depth_m"]["median"] > classes.max_range_m:
            result.far.append(instance)
            continue
        mask = ids == instance["id"]
        if int(np.count_nonzero(mask)) < classes.min_visible_px:
            result.dropped_small += 1
            continue
        kept, crumbs = pieces(mask, classes.min_fragment_px)
        result.crumbs += crumbs
        if not kept:  # nothing but crumbs
            result.dropped_small += 1
            continue
        if len(kept) >= 2 and classes.fragments == "reject":
            result.fragmented.append(instance)
            continue
        if len(kept) >= 2 and classes.fragments == "keep_largest":
            mask = kept[0]
        elif crumbs:
            mask = np.logical_or.reduce(kept)
        if classes.names[cls] in classes.outer:
            mask = fill_holes(mask)
        result.labels.append(Label(cls, mask, instance))
    return result


# ------------------------------------------------------------------ dataset layout


def split_of(name: str, fractions: dict[str, float]) -> str:
    """Stable split from the sample name's hash (the same name always lands in one split)."""
    digest = hashlib.sha256(name.encode("utf-8")).digest()
    position = int.from_bytes(digest[:8], "big") / 2.0**64
    total = 0.0
    chosen = "train"
    for split in SPLITS:
        fraction = float(fractions.get(split, 0.0))
        if fraction <= 0:
            continue
        chosen = split
        total += fraction
        if position < total:
            break
    return chosen


def place_file(source: Path, target: Path) -> None:
    """Hard link when the filesystem allows it, else copy."""
    target.unlink(missing_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def _prepare(out: Path) -> None:
    """Fresh images/ and labels/; refuse to write into an unrelated non-empty folder."""
    if out.exists() and any(out.iterdir()) and not (out / "data.yaml").is_file():
        raise PackError(f"{out}: exists, is not empty and is not a previous export")
    for folder in ("images", "labels"):
        shutil.rmtree(out / folder, ignore_errors=True)
    for split in SPLITS:
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)


def _data_yaml(names: list[str], fractions: dict[str, float], has_test: bool) -> str:
    lines = [
        # No absolute path: Ultralytics then uses this file's folder, so the export can move.
        "train: images/train",
        "val: images/val",
    ]
    if has_test or fractions.get("test", 0.0) > 0:
        lines.append("test: images/test")
    lines.append(f"nc: {len(names)}")
    lines.append(f"names: [{', '.join(json.dumps(name) for name in names)}]")
    return "\n".join(lines) + "\n"


@dataclass
class Summary:
    out: Path
    format: str
    names: list[str]
    images: dict[str, int]
    counts: list[int]
    backgrounds: int = 0
    skipped_far: list[str] = field(default_factory=list)
    skipped_fragmented: list[str] = field(default_factory=list)
    dropped_small: int = 0
    dropped_degenerate: int = 0
    crumbs: int = 0
    environments: dict[str, int] = field(default_factory=dict)  # images written per environment

    def unbalanced(self) -> list[str]:
        """Classes more than 20 % from the mean instance count."""
        if not self.counts:
            return []
        mean = sum(self.counts) / len(self.counts)
        return [
            name
            for name, count in zip(self.names, self.counts)
            if abs(count - mean) > BALANCE_TOLERANCE * mean
        ]

    def report(self) -> list[str]:
        written = sum(self.images.values())
        splits = ", ".join(f"{split} {count}" for split, count in self.images.items() if count)
        lines = [f"{self.format}: {written} images ({splits or 'none'}) -> {self.out}"]
        lines.append(f"  backgrounds (no labels): {self.backgrounds}")
        if self.skipped_far:
            lines.append(f"  skipped, labelled instance beyond range: {len(self.skipped_far)}")
        if self.skipped_fragmented:
            # Renders made with the same label pack never contain these.
            lines.append(f"  fragmented images skipped: {len(self.skipped_fragmented)}")
        if self.crumbs:
            lines.append(f"  crumbs removed (pieces under min_fragment_px): {self.crumbs}")
        if self.dropped_small:
            lines.append(f"  instances under min_visible_px: {self.dropped_small}")
        if self.dropped_degenerate:
            lines.append(f"  instances without a polygon (slivers): {self.dropped_degenerate}")
        if self.environments:
            lines.append(
                "  environments: "
                + ", ".join(f"{name} {count}" for name, count in sorted(self.environments.items()))
            )
        width = max((len(name) for name in self.names), default=0)
        for index, (name, count) in enumerate(zip(self.names, self.counts)):
            lines.append(f"  {index:>3} {name:<{width}} {count:>7}")
        unbalanced = self.unbalanced()
        if unbalanced:
            mean = sum(self.counts) / len(self.counts)
            lines.append(
                f"WARNING: class counts differ from the mean ({mean:.1f}) by more than "
                f"{BALANCE_TOLERANCE:.0%}: {', '.join(unbalanced)}"
            )
        return lines


def export(
    render: Path,
    fmt: str,
    out: Path,
    *,
    labels: Path | None = None,
    model: str | None = None,
    split: dict[str, float] | None = None,
) -> Summary:
    """Write one YOLO dataset from a render folder (records/, images/, ids/)."""
    if fmt not in LINES:
        raise PackError(f"unknown format '{fmt}' (choose from {', '.join(FORMATS)})")
    render = Path(render).resolve()
    out = Path(out).resolve()
    classes, settings = class_map(render, labels, model)
    fractions = split or settings.get("split") or {"train": 0.8, "val": 0.2, "test": 0.0}
    line_of = LINES[fmt]
    found = records(render)
    _prepare(out)
    summary = Summary(out, fmt, classes.names, dict.fromkeys(SPLITS, 0), [0] * len(classes.names))
    for record in found:
        name = record["name"]
        labelled = label_record(record, read_ids(render, record), classes)
        if labelled.far:
            summary.skipped_far.append(name)
            continue
        if labelled.fragmented:
            summary.skipped_fragmented.append(name)
            continue
        summary.dropped_small += labelled.dropped_small
        summary.crumbs += labelled.crumbs
        environment = record.get("environment")
        if environment is not None:
            summary.environments[environment] = summary.environments.get(environment, 0) + 1
        lines = []
        for label in labelled.labels:
            line = line_of(label.mask, label.cls)
            if line is None:
                summary.dropped_degenerate += 1
                continue
            lines.append(line)
            summary.counts[label.cls] += 1
        if not lines:
            summary.backgrounds += 1
        chosen = split_of(name, fractions)
        image = render / record["image"]
        if not image.is_file():
            raise PackError(f"{image}: missing image of record {name}")
        place_file(image, out / "images" / chosen / f"{name}{image.suffix}")
        text = "".join(f"{line}\n" for line in lines)
        (out / "labels" / chosen / f"{name}.txt").write_text(text, encoding="utf-8")
        summary.images[chosen] += 1
    (out / "data.yaml").write_text(
        _data_yaml(classes.names, fractions, summary.images["test"] > 0), encoding="utf-8"
    )
    return summary
