"""Environment comparison sheets: the same robot views under every environment at the low, middle
and high end of its ranges, with the values the renderer used printed beside each row.

Stage 1 renders a few views per task (near -> far) under one neutral environment. Stage 2 renders
each view again with a ``fixed`` sampler under every environment variant (a forced environment per
block), then one JPEG per task lays them out: rows = environment x variant, columns = views.
"""

from __future__ import annotations

import copy
import json
import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from nereus.packs import PackError

from . import _environments as environments
from ._documents import check_environment, load_document
from .export import class_map, cv2_module, label_record, read_ids, read_json, records
from .plan import Overrides, _randomize, _write_json, plan
from .preview import Image, overlay, read_image
from .render import render

VARIANTS = ("min", "mid", "max")
TILE_WIDTH = 640
SETTINGS_WIDTH = 560
RATIO = (110, 215, 255)  # BGR amber: multiples of the pool's calibrated values
HEADER_HEIGHT = 58
GAP = 6
LABEL_ALPHA = 0.25
BACKGROUND = (28, 28, 28)
INK = (235, 235, 235)
DIM = (150, 150, 150)


# ------------------------------------------------------------------ environment variants


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _end(value: Any, which: str) -> Any:
    """A range at its low end, middle or high end; per channel for a pair of vectors."""
    if isinstance(value, list) and len(value) == 2 and all(_number(item) for item in value):
        low, high = float(value[0]), float(value[1])
        return {"min": low, "mid": (low + high) / 2, "max": high}[which]
    pair_of_vectors = (
        isinstance(value, list)
        and len(value) == 2
        and all(isinstance(item, list) for item in value)
        and len(value[0]) == len(value[1])
    )
    if pair_of_vectors:
        return [_end([low, high], which) for low, high in zip(value[0], value[1])]
    return copy.deepcopy(value)


def variant(environment: dict[str, Any], which: str) -> dict[str, Any]:
    """Every range fixed at its ``min`` / ``mid`` / ``max``; fixed values kept; time_s at its mid."""
    if which not in VARIANTS:
        raise PackError(f"--variants: '{which}' is not one of {', '.join(VARIANTS)}")
    result = copy.deepcopy(environment)
    result["id"] = f"{environment['id']}:{which}"
    result["weight"] = 1.0
    for group in environments.GROUPS:
        result[group] = {key: _end(value, which) for key, value in result.get(group, {}).items()}
    result["time_s"] = _end(result.get("time_s", environments.DEFAULTS["time_s"]), "mid")
    return result


def parse_setting(text: str) -> tuple[str, Any]:
    """``group.key=VALUE``: VALUE is JSON (number, [a, b], [r, g, b]) or a bare word."""
    key, separator, raw = text.partition("=")
    key = key.strip()
    group, _, name = key.partition(".")
    if not separator or not (key == "time_s" or (group in environments.GROUPS and name)):
        raise PackError(f"--set: '{text}' is not group.key=VALUE (groups: water, lighting, image)")
    try:
        value: Any = json.loads(raw)
    except ValueError:
        value = raw.strip()
    return key, value


def apply_settings(environment: dict[str, Any], settings: list[tuple[str, Any]]) -> dict[str, Any]:
    """``--set`` values laid over a merged environment (absolute and scale forms replace each other)."""
    result = copy.deepcopy(environment)
    for key, value in settings:
        if key == "time_s":
            environments._apply(result, {"time_s": value})
        else:
            group, _, name = key.partition(".")
            environments._apply(result, {group: {name: value}})
    problems = check_environment(result, f"--set [{result['id']}]")
    if problems:
        raise PackError(problems)
    return result


# ------------------------------------------------------------------ stage 1: views


def split_range(value: list[float], count: int) -> list[list[float]]:
    low, high = float(value[0]), float(value[1])
    step = (high - low) / count
    return [[low + index * step, low + (index + 1) * step] for index in range(count)]


_SPLIT_KEY = {"approach": "range_m", "overhead": "altitude_m"}


def view_blocks(job: dict[str, Any], views: int) -> list[dict[str, Any]]:
    """Each task's block as ``views`` count-1 blocks over equal slices of its range (near -> far)."""
    blocks = []
    for block in job["samples"]:
        if block["task"] is None:
            continue
        key = _SPLIT_KEY.get(block["sampler"]["type"])
        for index in range(views):
            sampler = copy.deepcopy(block["sampler"])
            if key is not None:
                sampler[key] = split_range(block["sampler"][key], views)[index]
            blocks.append({"task": block["task"], "count": 1, "sampler": sampler, "view": index})
    return blocks


def _calm(randomize: dict[str, Any]) -> dict[str, Any]:
    """No placement jitter and no latched indicators: only the appearance changes."""
    result = copy.deepcopy(randomize)
    result["placement"] = dict(result["placement"], task_yaw_deg=0, task_offset_m=0)
    result["indicators"] = dict(result["indicators"], latched_probability=0)
    return result


def _shard_logs(folder: Path) -> dict[str, dict[str, Any]]:
    """Shard log lines by sample name (the last line per name wins)."""
    found: dict[str, dict[str, Any]] = {}
    for path in sorted((folder / "logs").glob("*.jsonl")):
        for line in path.read_text("utf-8").splitlines():
            if line.strip():
                item = json.loads(line)
                if "name" in item:
                    found[item["name"]] = item
    return found


def _reasons(log: dict[str, Any] | None) -> str:
    if log is None:
        return "not rendered"
    reasons = log.get("reasons") or {}
    return ", ".join(f"{name} x{count}" for name, count in reasons.items()) or str(
        log.get("status")
    )


@dataclass
class View:
    task: str
    index: int
    sampler: dict[str, Any]
    record: dict[str, Any]

    @property
    def pose(self) -> dict[str, Any]:
        pose: dict[str, Any] = self.record["robot"]["world_from_root"]
        return pose

    @property
    def target_frame(self) -> str | None:
        frame = self.record.get("target_frame")
        if frame is None:
            frame = self.record.get("randomization", {}).get("target_frame")
        return frame if isinstance(frame, str) and frame else None

    @property
    def heading(self) -> str:
        """Column header: which slice of the sampler's range this view came from."""
        sampler = self.sampler
        key = _SPLIT_KEY.get(sampler["type"])
        what = {"range_m": "range", "altitude_m": "altitude"}.get(key or "", "")
        span = f"{what} {sampler[key][0]:.2f}-{sampler[key][1]:.2f} m" if key else sampler["type"]
        frame = f"  @ {self.target_frame}" if self.target_frame else ""
        return f"view {self.index + 1}: {span}{frame}"


# ------------------------------------------------------------------ stage 2: grid


def grid_job(
    base: dict[str, Any],
    views: list[View],
    variants: list[dict[str, Any]],
    scenario: int,
    output: Path,
) -> tuple[dict[str, Any], list[tuple[View, int]]]:
    """One block per (view, environment variant) for the views of one scenario.

    Returns the job and, per sample index, the (view, variant index) it renders.
    """
    job = copy.deepcopy(base)
    job["output"] = str(output)
    job["scenarios"] = [base["scenarios"][scenario]]
    randomize = _calm(base["randomize"])
    randomize["environments"] = variants
    randomize["environment_mode"] = "weighted"
    job["randomize"] = randomize
    samples = []
    layout = []
    for view in views:
        if view.record.get("scenario_index", 0) != scenario:
            continue
        sampler: dict[str, Any] = {"type": "fixed", "world_from_root": view.pose}
        if view.target_frame is not None:
            sampler["target_frame"] = view.target_frame
        for index in range(len(variants)):
            samples.append(
                {"task": view.task, "count": 1, "sampler": sampler, "environment": index}
            )
            layout.append((view, index))
    job["samples"] = samples
    return job, layout


# ------------------------------------------------------------------ sheets

_FONT_SCALE = 0.58
_LINE = 24


def _put(image: Image, text: str, x: int, y: int, ink: tuple[int, int, int], scale: float) -> None:
    cv2 = cv2_module()
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, ink, 1, cv2.LINE_AA)


def _numbers(values: Any, digits: int = 3) -> str:
    if isinstance(values, list):
        return " ".join(f"{float(value):.{digits}f}" for value in values)
    if isinstance(values, (int, float)):
        return f"{float(values):.{digits}f}"
    return "-"


def _ratio(value: Any, pool: Any) -> str:
    """`` x0.85``: the value as a multiple of the pool's (one number when every channel agrees)."""
    values = value if isinstance(value, list) else [value]
    bases = pool if isinstance(pool, list) else [pool]
    if not values or len(values) != len(bases) or not all(_number(v) for v in values + bases):
        return ""
    ratios = [float(v) / float(b) for v, b in zip(values, bases) if float(b) != 0]
    if not ratios:
        return ""
    if max(ratios) - min(ratios) < 0.005:
        return f"  x{ratios[0]:.2f}"
    return "  x" + "/".join(f"{ratio:.2f}" for ratio in ratios)


def settings_lines(
    randomization: dict[str, Any], pool: dict[str, Any] | None = None
) -> list[tuple[str, str]]:
    """(label, value) rows of the appearance a record was rendered with; water and lights also as
    multiples of the pool pack's calibrated values when ``pool`` is given."""
    water = randomization.get("water", {})
    lighting = randomization.get("lighting", {})
    image = randomization.get("image", {})
    optics = (pool or {}).get("water_optics", {})
    lights = (pool or {}).get("lighting", {})

    def scaled(value: Any, base: Any, digits: int = 3) -> str:
        return _numbers(value, digits) + (_ratio(value, base) if base is not None else "")

    return [
        ("water", ""),
        ("scattering", scaled(water.get("scattering"), optics.get("scattering"))),
        (
            "absorption /m",
            scaled(water.get("absorption_per_m_rgb"), optics.get("absorption_per_m_rgb")),
        ),
        ("tint rgb", scaled(water.get("tint_rgb"), optics.get("tint_rgb"))),
        ("distance scale", scaled(water.get("distance_scale"), optics.get("distance_scale"), 2)),
        ("lighting", str(lighting.get("profile", ""))),
        ("exposure", _numbers(lighting.get("exposure"), 2)),
        ("caustics", _numbers(lighting.get("caustics"), 2)),
        ("direct light", scaled(lighting.get("direct_light"), lights.get("direct_light"), 2)),
        ("ambient light", scaled(lighting.get("ambient_light"), lights.get("ambient_light"), 2)),
        (
            "sun az / el",
            f"{_numbers(lighting.get('sun_azimuth_deg'), 0)} / "
            f"{_numbers(lighting.get('sun_elevation_deg'), 0)} deg",
        ),
        ("glare", _numbers(lighting.get("glare"), 2)),
        ("image", ""),
        ("noise sigma", _numbers(image.get("noise_sigma"), 2)),
        ("blur px", _numbers(image.get("blur_px"), 2)),
    ]


def settings_panel(title: str, lines: list[tuple[str, str]], height: int) -> Image:
    panel = np.full((height, SETTINGS_WIDTH, 3), BACKGROUND, dtype=np.uint8)
    _put(panel, title, 12, 30, INK, 0.8)
    y = 30 + _LINE + 4
    for label, value in lines:
        if not value or label in ("water", "image") or (label == "lighting"):
            _put(
                panel, label.upper() + (f"  {value}" if value else ""), 12, y, (120, 200, 255), 0.55
            )
        else:
            _put(panel, label, 22, y, DIM, _FONT_SCALE)
            number, _, ratio = value.partition("  x")
            _put(panel, number, 190, y, INK, _FONT_SCALE)
            if ratio:
                _put(panel, f"x{ratio}", 400, y, RATIO, _FONT_SCALE)
        y += _LINE
    return panel


def _blank(height: int, text: str) -> Image:
    tile = np.full((height, TILE_WIDTH, 3), (45, 45, 60), dtype=np.uint8)
    for index, line in enumerate(_wrap(text, 52)):
        _put(tile, line, 16, height // 2 - 10 + 26 * index, (120, 160, 255), 0.65)
    return tile


def _wrap(text: str, width: int) -> list[str]:
    words, lines, line = text.split(" "), [], ""
    for word in words:
        if line and len(line) + 1 + len(word) > width:
            lines.append(line)
            line = word
        else:
            line = f"{line} {word}".strip()
    return lines + [line] if line else lines


def _caption(tile: Image, text: str) -> None:
    cv2 = cv2_module()
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    y = tile.shape[0] - 8
    cv2.rectangle(tile, (0, y - height - 8), (width + 16, tile.shape[0]), (20, 20, 20), cv2.FILLED)
    _put(tile, text, 8, y - 2, INK, 0.6)


@dataclass
class Cell:
    folder: Path
    name: str
    record: dict[str, Any] | None
    log: dict[str, Any] | None


@dataclass
class Row:
    environment: str
    variant: str
    cells: list[Cell] = field(default_factory=list)


def sheet(
    task: str,
    views: list[View],
    rows: list[Row],
    classes: Any,
    labels: bool,
    pools: dict[int, dict[str, Any]] | None = None,
) -> tuple[Image, list[dict[str, Any]]]:
    """One task's grid; also the settings each row used (for settings.json)."""
    tile_height = round(TILE_WIDTH * 600 / 960)
    for row in rows:
        for cell in row.cells:
            if cell.record is not None:
                camera = cell.record["camera"]
                tile_height = round(TILE_WIDTH * camera["height"] / camera["width"])
                break
    columns = len(views)
    width = SETTINGS_WIDTH + columns * (TILE_WIDTH + GAP) + GAP
    blocks: list[Image] = []
    header = np.full((HEADER_HEIGHT, width, 3), BACKGROUND, dtype=np.uint8)
    _put(header, task, 12, 38, INK, 1.0)
    for column, view in enumerate(views):
        x = SETTINGS_WIDTH + GAP + column * (TILE_WIDTH + GAP)
        _put(header, view.heading, x + 4, 38, INK, 0.62)
    blocks.append(header)
    pools = pools or {}
    summary = []
    previous = None
    for row in rows:
        if previous is not None and previous != row.environment:
            blocks.append(np.full((GAP * 2, width, 3), (90, 90, 90), dtype=np.uint8))
        previous = row.environment
        used = next((cell.record for cell in row.cells if cell.record is not None), None)
        pool = pools.get(used.get("scenario_index", 0)) if used else None
        lines = settings_lines(used.get("randomization", {}), pool) if used else []
        line = np.full((tile_height, width, 3), BACKGROUND, dtype=np.uint8)
        title = f"{row.environment}  [{row.variant}]"
        line[:, :SETTINGS_WIDTH] = settings_panel(title, lines, tile_height)
        for column, cell in enumerate(row.cells):
            x = SETTINGS_WIDTH + GAP + column * (TILE_WIDTH + GAP)
            line[:, x : x + TILE_WIDTH] = _cell_tile(cell, row, classes, labels, tile_height)
        blocks.append(line)
        blocks.append(np.full((GAP, width, 3), BACKGROUND, dtype=np.uint8))
        summary.append(
            {
                "environment": row.environment,
                "variant": row.variant,
                "settings": used.get("randomization", {}) if used else None,
                "cells": [cell.name if cell.record is not None else None for cell in row.cells],
            }
        )
    return np.concatenate(blocks), summary


def _cell_tile(cell: Cell, row: Row, classes: Any, labels: bool, height: int) -> Image:
    cv2 = cv2_module()
    if cell.record is None:
        return _blank(height, f"{row.environment}:{row.variant} rejected: {_reasons(cell.log)}")
    record = cell.record
    image = read_image(cell.folder, record)
    labelled = label_record(record, read_ids(cell.folder, record), classes)
    if labels:
        tile = overlay(image, labelled, classes.names, TILE_WIDTH, alpha=LABEL_ALPHA, boxes=False)
    else:
        tile = np.asarray(
            cv2.resize(image, (TILE_WIDTH, height), interpolation=cv2.INTER_AREA), dtype=np.uint8
        )
    tile = np.ascontiguousarray(tile[:height])
    if tile.shape[0] < height:
        tile = np.concatenate([tile, np.zeros((height - tile.shape[0], TILE_WIDTH, 3), np.uint8)])
    count = len(labelled.labels)
    _caption(tile, f"{row.environment}:{row.variant}   {count} label{'s' if count != 1 else ''}")
    return tile


# ------------------------------------------------------------------ the command


@dataclass
class Options:
    tasks: list[str] | None = None
    views: int = 3
    environments: list[str] | None = None
    variants: list[str] = field(default_factory=lambda: list(VARIANTS))
    settings: list[str] = field(default_factory=list)
    labels: bool = True
    resolution: str = "960x600"
    supersample: int | None = None
    renderer: Path | None = None
    workers: int = 2
    force: bool = False


def _prepare(out: Path, dataset_file: Path, force: bool) -> None:
    """Fresh views/ and grid/; never clobber a folder made for another spec."""
    marker = out / "settings.json"
    if out.exists() and any(out.iterdir()) and not force:
        previous = read_json(marker).get("dataset_file") if marker.is_file() else None
        if previous != str(dataset_file):
            raise PackError(
                f"{out}: holds something other than this spec's comparison; "
                "choose another --out or pass --force"
            )
    for folder in ("views", "grid"):
        shutil.rmtree(out / folder, ignore_errors=True)
    out.mkdir(parents=True, exist_ok=True)
    _write_json(marker, {"dataset_file": str(dataset_file)})


def compare(dataset: Path, out: Path, options: Options) -> list[Path]:
    """Render the views and the grid; write one JPEG per task and settings.json."""
    cv2 = cv2_module()
    out = Path(out).resolve()
    spec = load_document(dataset, "dataset")
    tasks = options.tasks or list(spec.data.get("tasks", {}))
    unknown = [task for task in tasks if task not in spec.data.get("tasks", {})]
    if unknown or not tasks:
        raise PackError(f"--task: {', '.join(unknown) or 'none'} not a task of {spec.path}")
    if options.views < 1:
        raise PackError("--views: at least 1")
    variants_wanted = list(dict.fromkeys(options.variants))
    for which in variants_wanted:
        variant({"id": "x"}, which)  # validates the name
    settings = [parse_setting(item) for item in options.settings]

    # The environments to compare, merged as the planner merges them, then --set.
    picked = _randomize(
        spec.data.get("randomize", {}), Overrides(environments=options.environments)
    )
    chosen = [apply_settings(item, settings) for item in picked["environments"]]
    variants = [variant(item, which) for item in chosen for which in variants_wanted]
    _prepare(out, spec.path, options.force)

    # Stage 1: views under the first environment at its middle.
    views_dir = out / "views"
    overrides = Overrides(
        tasks=tasks, count=1, resolution=options.resolution, supersample=options.supersample
    )
    planned = plan(spec.path, views_dir, overrides)
    job = planned.job
    blocks = view_blocks(job, options.views)
    job["samples"] = [
        {key: value for key, value in block.items() if key != "view"} for block in blocks
    ]
    job["randomize"] = _calm(job["randomize"])
    job["randomize"]["environments"] = [variant(chosen[0], "mid")]
    _write_json(planned.job_path, job)
    print(f"stage 1: {len(blocks)} views of {', '.join(tasks)} -> {views_dir}")
    render(views_dir, workers=options.workers, renderer=options.renderer)
    by_name = {record["name"]: record for record in records(views_dir)}
    logs = _shard_logs(views_dir)
    views: list[View] = []
    for k, block in enumerate(blocks):
        name = f"{block['task']}_{k:06d}"
        if name in by_name:
            views.append(View(block["task"], block["view"], block["sampler"], by_name[name]))
        else:
            print(
                f"  {block['task']} view {block['view'] + 1}: no view ({_reasons(logs.get(name))})"
            )
    if not views:
        raise PackError("stage 1 produced no views: widen the samplers or check the shard logs")

    # Stage 2: every view under every environment variant, one job per scenario.
    classes, _ = class_map(views_dir, None, None)
    pools = {
        index: read_json(Path(item["resolved"]))["pool"]
        for index, item in enumerate(job["scenarios"])
    }
    cells: dict[tuple[str, int, int], Cell] = {}
    scenarios = sorted({view.record.get("scenario_index", 0) for view in views})
    for scenario in scenarios:
        folder = out / "grid" if len(scenarios) == 1 else out / "grid" / f"scenario_{scenario}"
        folder.mkdir(parents=True, exist_ok=True)
        grid, layout = grid_job(job, views, variants, scenario, folder)
        _write_json(folder / "job.json", grid)
        print(f"stage 2: {len(layout)} renders ({len(variants)} variants) -> {folder}")
        render(folder, workers=options.workers, renderer=options.renderer)
        found = {record["name"]: record for record in records(folder)}
        logs = _shard_logs(folder)
        for k, (view, index) in enumerate(layout):
            name = f"{view.task}_{k:06d}"
            cells[(view.task, view.index, index)] = Cell(
                folder, name, found.get(name), logs.get(name)
            )

    sheets = []
    report: dict[str, Any] = {"dataset_file": str(spec.path), "tasks": {}}
    for task in tasks:
        task_views = sorted((view for view in views if view.task == task), key=lambda v: v.index)
        if not task_views:
            continue
        rows = []
        for index, item in enumerate(variants):
            environment, _, which = item["id"].rpartition(":")
            row = Row(environment, which)
            row.cells = [cells[(task, view.index, index)] for view in task_views]
            rows.append(row)
        image, summary = sheet(task, task_views, rows, classes, options.labels, pools)
        path = out / f"{task}.jpg"
        if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            raise PackError(f"{path}: cannot write the sheet")
        sheets.append(path)
        report["tasks"][task] = {
            "sheet": str(path),
            "views": [
                {
                    "view": view.index + 1,
                    "heading": view.heading,
                    "pose": view.pose,
                    "target_frame": view.target_frame,
                    "scenario_index": view.record.get("scenario_index", 0),
                }
                for view in task_views
            ],
            "rows": summary,
        }
    report["environments"] = variants
    _write_json(out / "settings.json", report)
    return sheets
