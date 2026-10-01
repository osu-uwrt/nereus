"""Dataset spec + label pack + parts + resolved scenarios -> renderer job (``job.json``).

The job is everything ``nereus-dataset-render`` needs with absolute paths; ``job/export.json``
keeps what the exporter needs (label pack, model, split) next to it.
"""

from __future__ import annotations

import copy
import json
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from nereus.packs import PackError, ResolvedScenario, resolve_scenario

from . import _environments as environments
from ._documents import (
    Course,
    Document,
    canonical,
    check_data,
    check_dataset,
    check_labels,
    check_model,
    check_parts,
    course,
    label_warnings,
    load_document,
    native_resolution,
    parts_by_task,
)
from ._mapping import ClassMap

JOB_FORMAT = "nereus.dataset_job.v1"
EXPORT_FORMAT = "nereus.dataset_export.v1"
BACKGROUND = "background"

IMAGE_DEFAULTS: dict[str, Any] = {
    "resolution_px": "native",
    "crop": "center",
    "format": "jpg",
    "jpeg_quality": 92,
    # Samples per pixel along each axis, averaged like a sensor pixel (anti-aliasing); 1 = off.
    "supersample": 2,
}
SPLIT_DEFAULTS = {"train": 0.8, "val": 0.2, "test": 0.0}
ACCEPTANCE_DEFAULTS: dict[str, Any] = {
    "min_target_px": 150,
    "near_m": 0.2,
    "max_near_fraction": 0.02,
    "max_attempts": 200,
    "background_max_labelled_px": 0,
}
# Randomization kept at the top of the job; water / lighting / image / time_s live in environments.
RANDOMIZE_DEFAULTS: dict[str, Any] = {
    "placement": {"task_yaw_deg": 0, "task_offset_m": 0, "groups": []},
    "indicators": {"latched_probability": 0.2},
}
# Sampler keys a CLI override sets, and the sampler types that take each.
OVERRIDE_SAMPLERS = {
    "range_m": "approach",
    "bearing_deg": "approach",
    "elevation_deg": "approach",
    "altitude_m": "overhead",
}


@dataclass
class Overrides:
    """Command-line changes applied to the spec before planning."""

    tasks: list[str] | None = None  # task ids and/or "background"; None = every block
    count: int | None = None  # per selected block
    range_m: list[float] | None = None
    bearing_deg: float | None = None
    elevation_deg: list[float] | None = None
    altitude_m: list[float] | None = None
    resolution: str | None = None  # "native" or "WxH"
    supersample: int | None = None
    seed: int | None = None
    environments: list[str] | None = None  # globs over expanded environment ids
    environment_mode: str | None = None  # weighted | sweep


@dataclass
class Plan:
    job: dict[str, Any]
    job_path: Path
    export: dict[str, Any]
    warnings: list[str]


def parse_resolution(text: str) -> str | list[int]:
    if text == "native":
        return "native"
    width, separator, height = text.lower().partition("x")
    try:
        size = [int(width), int(height)]
    except ValueError:
        size = []
    if not separator or len(size) != 2 or min(size) < 1:
        raise PackError(f"--resolution: '{text}' is not 'native' or WIDTHxHEIGHT")
    return size


def merged(defaults: dict[str, Any], given: dict[str, Any]) -> dict[str, Any]:
    """Defaults with ``given`` laid over them, mapping by mapping (lists replace)."""
    result = copy.deepcopy(defaults)
    for key, value in given.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merged(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def apply_overrides(dataset: dict[str, Any], overrides: Overrides) -> dict[str, Any]:
    """The spec with CLI overrides applied: only selected blocks remain."""
    data = copy.deepcopy(dataset)
    tasks: dict[str, Any] = data.get("tasks", {})
    if overrides.tasks is not None:
        known = [*tasks, *([BACKGROUND] if "background" in data else [])]
        unknown = [name for name in overrides.tasks if name not in known]
        if unknown:
            raise PackError(
                f"--task: {', '.join(unknown)} not in the dataset (has: {', '.join(known)})"
            )
        data["tasks"] = {name: block for name, block in tasks.items() if name in overrides.tasks}
        if BACKGROUND not in overrides.tasks:
            data.pop("background", None)
    blocks = [
        *data.get("tasks", {}).values(),
        *([data["background"]] if "background" in data else []),
    ]
    if overrides.count is not None:
        for block in blocks:
            block["count"] = overrides.count
    for key, sampler_type in OVERRIDE_SAMPLERS.items():
        value = getattr(overrides, key)
        if value is None:
            continue
        targets = [block["sampler"] for block in blocks if block["sampler"]["type"] == sampler_type]
        if not targets:
            option = "--" + key.replace("_", "-")
            raise PackError(f"{option}: no selected task has a sampler of type {sampler_type}")
        for sampler in targets:
            sampler[key] = value
    if overrides.resolution is not None:
        data.setdefault("image", {})["resolution_px"] = parse_resolution(overrides.resolution)
    if overrides.supersample is not None:
        if not 1 <= overrides.supersample <= 4:
            raise PackError(f"--supersample: {overrides.supersample} is not 1..4")
        data.setdefault("image", {})["supersample"] = overrides.supersample
    if overrides.seed is not None:
        data["seed"] = overrides.seed
    return data


def _write_json(path: Path, document: Any) -> None:
    text = json.dumps(document, indent=2, allow_nan=False) + "\n"
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def _resolved_document(resolved: ResolvedScenario) -> dict[str, Any]:
    """What ``python -m nereus.packs resolve`` writes."""
    if resolved.changed_sources():
        raise PackError(f"{resolved.path}: pack sources changed while resolving")
    document = resolved.manifest()
    document["asset_paths"] = resolved.asset_paths()
    return document


def _refuse_mixing(out: Path, job: dict[str, Any], documents: list[dict[str, Any]]) -> None:
    """The renderer resumes into existing records: never let a different job resume them."""
    records = out / "records"
    if not records.is_dir() or not any(records.iterdir()):
        return
    try:
        same = json.loads((out / "job.json").read_text("utf-8")) == job and all(
            json.loads((out / "job" / f"scenario_{index}.json").read_text("utf-8")) == document
            for index, document in enumerate(documents)
        )
    except (OSError, ValueError):
        same = False
    if not same:
        raise PackError(
            f"{out}: holds renders of a different job or pack version; "
            "plan into a new folder or delete it"
        )


def _randomize(randomize: dict[str, Any], overrides: Overrides) -> dict[str, Any]:
    """Job randomization: placement and indicators, plus every environment fully merged."""
    found, mode = environments.expand(randomize)
    if overrides.environments is not None:
        known = ", ".join(item["id"] for item in found)
        found = environments.select(found, overrides.environments)
        if not found:
            raise PackError(
                f"--environment: {', '.join(overrides.environments)} match none of {known}"
            )
    if overrides.environment_mode is not None:
        if overrides.environment_mode not in environments.MODES:
            raise PackError(
                f"--environment-mode: '{overrides.environment_mode}' is not weighted or sweep"
            )
        mode = overrides.environment_mode
    # Weight 0 switches an environment off, in sweep mode too.
    found = [item for item in found if item["weight"] > 0]
    if not found:
        raise PackError("randomize.environments: every selected environment has weight 0")
    kept = {key: value for key, value in randomize.items() if key in RANDOMIZE_DEFAULTS}
    result = merged(RANDOMIZE_DEFAULTS, kept)
    result["environments"] = found
    result["environment_mode"] = mode
    return result


def _job_parts(parts: Document, place: Course) -> dict[str, Any]:
    textures = []
    for item in parts.data.get("textures", []):
        textures.append(
            {
                # Path.resolve() is std::filesystem::weakly_canonical: the renderer matches on it.
                "texture": str(place.assets[item["texture"]].resolve()),
                "mask": str((place.folder / item["mask"]).resolve()),
                "task": item.get("task"),
                "values": {str(part["value"]): part["part"] for part in item["parts"]},
            }
        )
    visuals = []
    for item in parts.data.get("visuals", []):
        visuals.append(
            {
                "task": item["task"],
                "asset": item["asset"],
                "prop": item.get("prop"),
                "frame": item.get("frame"),
                "part": item.get("part"),
                "materials": item.get("materials"),
                "split": item.get("split", "none"),
                "indicator": item.get("indicator"),
            }
        )
    return {"textures": textures, "visuals": visuals}


def plan(dataset_path: Path, out: Path, overrides: Overrides | None = None) -> Plan:
    """Validate everything the spec selects and write ``out/job.json`` (+ ``out/job/``)."""
    dataset = load_document(dataset_path, "dataset")
    data = apply_overrides(dataset.data, overrides or Overrides())
    problems = check_data("dataset", data)
    if problems:
        raise PackError([f"{dataset.path} (with overrides):{problem}" for problem in problems])
    labels = load_document(canonical(dataset.root / data["labels"], "labels"), "labels")
    model = data["model"]

    resolved: list[ResolvedScenario] = []
    for entry in data["scenarios"]:
        resolved.append(resolve_scenario(dataset.root / entry))
    places = [course(item) for item in resolved]
    if len({place.folder for place in places}) != 1:
        folders = ", ".join(sorted({str(place.folder) for place in places}))
        raise PackError(
            f"{dataset.path}:/scenarios: scenarios must share one tasks pack ({folders})"
        )
    place = places[0]
    if not place.parts_file.is_file():
        raise PackError(f"{place.folder}: no parts.yaml next to tasks.yaml")
    parts = load_document(place.parts_file, "parts")

    problems = check_parts(parts, place)
    problems += check_labels(labels, parts, place)
    for item in resolved:
        problems += check_model(labels, model, item)
    problems += check_dataset(Document(dataset.path, "dataset", data), place)
    if problems:
        raise PackError(problems)
    classes = ClassMap(labels.data, model)

    image = merged(IMAGE_DEFAULTS, data.get("image", {}))
    if image["resolution_px"] == "native":
        sizes = {tuple(native_resolution(item, classes.camera)) for item in resolved}
        if len(sizes) != 1:
            raise PackError(f"{dataset.path}: scenario cameras differ in native resolution")
        image["resolution_px"] = list(sizes.pop())

    samples = [
        {"task": task, "count": block["count"], "sampler": block["sampler"]}
        for task, block in data.get("tasks", {}).items()
    ]
    if "background" in data:
        background = data["background"]
        samples.append(
            {"task": None, "count": background["count"], "sampler": background["sampler"]}
        )
    if sum(item["count"] for item in samples) == 0:
        raise PackError(f"{dataset.path}: no samples selected")

    out = Path(out).resolve()
    documents = [_resolved_document(item) for item in resolved]
    scenarios = [
        {"id": item.scenario["id"], "resolved": str(out / "job" / f"scenario_{index}.json")}
        for index, item in enumerate(resolved)
    ]

    job = {
        "format": JOB_FORMAT,
        "dataset": data["id"],
        "seed": data["seed"],
        "output": str(out),
        "scenarios": scenarios,
        "camera": {
            "sensor": classes.camera,
            "resolution_px": image["resolution_px"],
            "crop": image["crop"],
            "format": image["format"],
            "jpeg_quality": image["jpeg_quality"],
            "supersample": image["supersample"],
            "robot_visuals": data.get("robot_visuals", True),
        },
        "parts": _job_parts(parts, place),
        "labelled": classes.labelled(parts_by_task(parts, place)),
        "acceptance": {
            "max_range_m": classes.max_range_m,
            **merged(ACCEPTANCE_DEFAULTS, data.get("acceptance", {})),
            # The label pack's fragment rule, enforced at render time when it rejects.
            "fragments": "reject" if classes.fragments == "reject" else "allow",
            "min_fragment_px": classes.min_fragment_px,
            "min_visible_px": classes.min_visible_px,
        },
        "samples": samples,
        "randomize": _randomize(data.get("randomize", {}), overrides or Overrides()),
    }
    export = {
        "format": EXPORT_FORMAT,
        "dataset": data["id"],
        "dataset_file": str(dataset.path),
        "labels": str(labels.path),
        "model": model,
        "scenarios": [str(item.path) for item in resolved],
        "split": merged(SPLIT_DEFAULTS, data.get("split", {})),
    }
    job_path = out / "job.json"
    _refuse_mixing(out, job, documents)
    (out / "job").mkdir(parents=True, exist_ok=True)
    for scenario, document in zip(scenarios, documents):
        _write_json(Path(scenario["resolved"]), document)
    _write_json(out / "job" / "export.json", export)
    _write_json(job_path, job)
    return Plan(job, job_path, export, label_warnings(labels))


def describe(job: dict[str, Any]) -> list[str]:
    """One line per sample block, with the global sample index ranges."""
    lines = []
    start = 0
    for block in job["samples"]:
        name = block["task"] or BACKGROUND
        end = start + block["count"]
        frames = block["sampler"].get("frame", "")
        where = f" @ {frames}" if frames else ""
        indices = f"k={start}..{end - 1}" if end > start else "-"
        lines.append(
            f"  {name:<12} {block['count']:>6}  {indices}  {block['sampler']['type']}{where}"
        )
        start = end
    randomize = job["randomize"]
    found = randomize["environments"]
    if randomize["environment_mode"] == "sweep":
        names = ", ".join(item["id"] for item in found)
        lines.append(f"  environments (sweep, cycled evenly in each block): {names}")
    else:
        total = sum(item["weight"] for item in found)
        shares = ", ".join(f"{item['id']} {item['weight'] / total:.0%}" for item in found)
        lines.append(f"  environments (weighted): {shares}")
    return lines
