"""Parts, labels and dataset documents: strict loading, schemas and cross-checks.

Mirrors ``nereus.packs``: YAML through the same strict reader, JSON Schemas in ``definitions/``,
then semantic checks; every problem found is reported at once in a ``PackError``.
"""

from __future__ import annotations

import fnmatch
import json
import re
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from functools import lru_cache
from importlib import resources
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from nereus.packs import PackError, ResolvedScenario
from nereus.packs._document import plain, read_yaml

from . import _environments as environments

DATASET_KINDS = ("parts", "labels", "dataset")
_DIALECT = "https://json-schema.org/draft/2020-12/schema"


# ------------------------------------------------------------------ schemas


def _load(name: str) -> dict[str, Any]:
    text = (
        resources.files(__package__)
        .joinpath("definitions")
        .joinpath(f"{name}.json")
        .read_text("utf-8")
    )
    result: dict[str, Any] = json.loads(text)
    return result


def _rewrite(node: Any) -> Any:
    """Point ``common.json#/$defs/x`` references at the bundled copy."""
    if isinstance(node, dict):
        result = {key: _rewrite(value) for key, value in node.items()}
        reference = result.get("$ref")
        if isinstance(reference, str) and reference.startswith("common.json#"):
            result["$ref"] = reference.removeprefix("common.json")
        return result
    if isinstance(node, list):
        return [_rewrite(item) for item in node]
    return node


@lru_cache(maxsize=None)
def _schema(kind: str) -> str:
    if kind not in DATASET_KINDS:
        raise KeyError(kind)
    document = _rewrite(_load(kind))
    defs = dict(_load("common")["$defs"])
    for name, definition in document.pop("$defs", {}).items():
        if name in defs:
            raise AssertionError(f"definition name collision: {name}")
        defs[name] = definition
    document.pop("$schema", None)
    bundle = {"$schema": _DIALECT, "$id": f"urn:nereus:dataset:{kind}:1", **document}
    bundle["$defs"] = defs
    Draft202012Validator.check_schema(bundle)
    return json.dumps(bundle)


def schema(kind: str) -> dict[str, Any]:
    """Self-contained JSON Schema (draft 2020-12) of a parts, labels or dataset document."""
    result: dict[str, Any] = json.loads(_schema(kind))
    return result


@lru_cache(maxsize=None)
def _validator(kind: str) -> Draft202012Validator:
    return Draft202012Validator(json.loads(_schema(kind)))


def schema_problems(kind: str, data: dict[str, Any]) -> list[str]:
    problems = []
    errors = _validator(kind).iter_errors(data)
    for error in sorted(errors, key=lambda item: [str(part) for part in item.path]):
        location = "/" + "/".join(str(part) for part in error.path)
        problems.append(f"{location}: {error.message[:300]}")
    return problems


# ------------------------------------------------------------------ loading


@dataclass
class Document:
    """One validated document as detached plain data."""

    path: Path
    kind: str
    data: dict[str, Any]

    @property
    def root(self) -> Path:
        return self.path.parent


def canonical(path: Path, kind: str) -> Path:
    """The document file: ``path`` itself, or ``<kind>.yaml`` in a folder."""
    path = Path(path)
    if path.is_dir():
        path = path / f"{kind}.yaml"
    if not path.is_file():
        raise PackError(f"{path}: no such {kind} file or folder")
    return path.resolve()


def _ordered(value: Any, where: str, problems: list[str]) -> None:
    """Every two-number range [low, high] in a document must have low <= high."""
    if isinstance(value, dict):
        for key, item in value.items():
            _ordered(item, f"{where}/{key}", problems)
    elif isinstance(value, list):
        is_range = len(value) == 2 and all(
            isinstance(item, (int, float)) and not isinstance(item, bool) for item in value
        )
        parent, _, name = where.rpartition("/")
        values_list = parent.endswith("/sweep")  # a sweep key's values, not a range
        if is_range and not values_list and name not in ("seed_px", "resolution_px"):
            if value[0] > value[1]:
                problems.append(f"{where}: range low {value[0]} exceeds high {value[1]}")
        for index, item in enumerate(value):
            _ordered(item, f"{where}/{index}", problems)


def duplicates(values: Iterable[str], what: str, where: str, problems: list[str]) -> None:
    seen: set[str] = set()
    for value in values:
        if value in seen:
            problems.append(f"{where}: duplicate {what} '{value}'")
        seen.add(value)


def _parts_semantics(data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    keys = [f"{item['texture']}@{item.get('task', '*')}" for item in data.get("textures", [])]
    duplicates(keys, "texture entry", "/textures", problems)
    for index, item in enumerate(data.get("textures", [])):
        values = [str(part["value"]) for part in item["parts"]]
        duplicates(values, "value", f"/textures/{index}/parts", problems)
    return problems


def _labels_semantics(data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    for name, model in data["models"].items():
        duplicates(model["classes"], "class", f"/models/{name}/classes", problems)
    return problems


def _dataset_semantics(data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    _ordered(data, "", problems)
    split = data.get("split")
    if split is not None:
        total = split["train"] + split["val"] + split.get("test", 0.0)
        if abs(total - 1.0) > 1e-9:
            problems.append(f"/split: fractions sum to {total:g}, not 1")
    if "background" in data.get("tasks", {}):
        problems.append("/tasks/background: 'background' names the background block, not a task")
    if not data.get("tasks") and "background" not in data:
        problems.append("/: no tasks and no background block: nothing to generate")
    if not problems:
        problems += environment_problems(data.get("randomize", {}))
    return problems


@lru_cache(maxsize=None)
def _environment_validator() -> Draft202012Validator:
    bundle = schema("dataset")
    return Draft202012Validator({"$ref": "#/$defs/environment", "$defs": bundle["$defs"]})


def environment_problems(randomize: dict[str, Any]) -> list[str]:
    """Expanded environments: unique ids, valid after sweep values, one form per quantity."""
    problems: list[str] = []
    entries = randomize.get("environments", {}).get("list", [])
    duplicates(
        (item["id"] for item in entries), "environment", "/randomize/environments/list", problems
    )
    found, _ = environments.expand(randomize)
    for item in found:
        where = f"/randomize/environments[{item['id']}]"
        candidate = {key: value for key, value in item.items() if key != "weight"}
        candidate["id"] = "x"  # expanded ids carry sweep values; the entry id was checked
        for error in _environment_validator().iter_errors(candidate):
            location = "/".join(str(part) for part in error.path)
            problems.append(f"{where}/{location}: {error.message[:300]}")
        _ordered(candidate, where, problems)
        for group, pairs in environments.EXCLUSIVE.items():
            for absolute, scale in pairs.items():
                if absolute in item[group] and scale in item[group]:
                    problems.append(f"{where}/{group}: set {absolute} or {scale}, not both")
    return problems


_SEMANTICS = {
    "parts": _parts_semantics,
    "labels": _labels_semantics,
    "dataset": _dataset_semantics,
}


def check_data(kind: str, data: dict[str, Any]) -> list[str]:
    """Schema, then single-document semantics (unprefixed problem list)."""
    problems = schema_problems(kind, data)
    return problems if problems else _SEMANTICS[kind](data)


def load_document(path: Path, kind: str) -> Document:
    """Load and validate one parts, labels or dataset document (folder or file)."""
    file = canonical(path, kind)
    data: dict[str, Any] = plain(read_yaml(file))
    found = data.get("kind")
    if found != kind:
        raise PackError(f"{file}: expected kind '{kind}', found {found!r}")
    problems = check_data(kind, data)
    if problems:
        raise PackError([f"{file}:{problem}" for problem in problems])
    return Document(file, kind, data)


# ------------------------------------------------------------------ the course a scenario selects

_INIT_FROM = re.compile(r"<init_from>\s*([^<]*?)\s*</init_from>")


def _references(node: Any) -> Iterator[tuple[str, str]]:
    """(key, value) for every asset reference in a task definition."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key in ("asset", "visual_asset", "texture") and isinstance(value, str):
                yield key, value
            else:
                yield from _references(value)
    elif isinstance(node, list):
        for item in node:
            yield from _references(item)


def _dae_textures(path: Path) -> set[Path]:
    try:
        text = path.read_text("utf-8", errors="replace")
    except OSError:
        return set()
    return {(path.parent / name).resolve() for name in _INIT_FROM.findall(text) if name}


@dataclass
class Course:
    """What the parts and label documents are checked against: one scenario's tasks pack."""

    tasks_id: str
    folder: Path
    assets: dict[str, Path]  # tasks-pack asset id -> absolute path
    frames: dict[str, set[str]]  # task id -> frame ids (including "task")
    regions: dict[str, dict[str, set[str]]]  # task id -> region id -> indicator colour names
    texture_tasks: dict[str, set[str]] = field(default_factory=dict)  # texture id -> task ids
    task_assets: dict[str, set[str]] = field(default_factory=dict)  # task id -> asset ids

    @property
    def parts_file(self) -> Path:
        return self.folder / "parts.yaml"


def course(resolved: ResolvedScenario) -> Course:
    folder = (resolved.path.parent / resolved.scenario["tasks"]).resolve()
    folder = folder if folder.is_dir() else folder.parent
    assets = {key: Path(value) for key, value in resolved.asset_paths()["tasks"].items()}
    by_path = {path: key for key, path in assets.items()}
    frames: dict[str, set[str]] = {}
    regions: dict[str, dict[str, set[str]]] = {}
    texture_tasks: dict[str, set[str]] = {}
    task_assets: dict[str, set[str]] = {}
    for task in resolved.task_definitions:
        name = task["id"]
        frames[name] = {"task", *(item["id"] for item in task.get("frames", []))}
        regions[name] = {}
        for region in task.get("regions", []):
            indicator = region.get("parameters", {}).get("indicator") or {}
            colours = {value for value in indicator.values() if isinstance(value, str)}
            regions[name][region["id"]] = colours
        used = {value for _, value in _references(task.get("props", [])) if value in assets}
        task_assets[name] = used
        for asset in used:
            if assets[asset].suffix.lower() == ".png":
                texture_tasks.setdefault(asset, set()).add(name)
            elif assets[asset].suffix.lower() == ".dae":
                for texture in _dae_textures(assets[asset]):
                    if texture in by_path:
                        texture_tasks.setdefault(by_path[texture], set()).add(name)
    return Course(
        tasks_id=resolved.tasks["id"],
        folder=folder,
        assets=assets,
        frames=frames,
        regions=regions,
        texture_tasks=texture_tasks,
        task_assets=task_assets,
    )


# ------------------------------------------------------------------ cross-checks


def check_parts(parts: Document, place: Course) -> list[str]:
    """parts.yaml against its tasks pack: pack id, asset ids, masks, task/frame/region ids."""
    data = parts.data
    problems: list[str] = []
    if data["tasks"] != place.tasks_id:
        problems.append(f"/tasks: '{data['tasks']}' is not the tasks pack id '{place.tasks_id}'")
    for index, item in enumerate(data.get("textures", [])):
        where = f"/textures/{index}"
        texture = place.assets.get(item["texture"])
        if texture is None:
            problems.append(f"{where}/texture: no tasks-pack asset '{item['texture']}'")
        elif texture.suffix.lower() != ".png":
            problems.append(f"{where}/texture: asset '{item['texture']}' is not a .png")
        mask = (place.folder / item["mask"]).resolve()
        if place.folder not in mask.parents:
            problems.append(f"{where}/mask: '{item['mask']}' escapes the tasks pack")
        elif not mask.is_file():
            problems.append(f"{where}/mask: {item['mask']} is not a file")
        if "task" in item and item["task"] not in place.frames:
            problems.append(f"{where}/task: no task '{item['task']}'")
    for index, item in enumerate(data.get("visuals", [])):
        where = f"/visuals/{index}"
        task = item["task"]
        if task not in place.frames:
            problems.append(f"{where}/task: no task '{task}'")
            continue
        if item["asset"] not in place.assets:
            problems.append(f"{where}/asset: no tasks-pack asset '{item['asset']}'")
        elif item["asset"] not in place.task_assets.get(task, set()):
            problems.append(f"{where}/asset: task '{task}' draws no visual '{item['asset']}'")
        if "frame" in item and item["frame"] not in place.frames[task]:
            problems.append(f"{where}/frame: task '{task}' has no frame '{item['frame']}'")
        if "indicator" in item and item["indicator"] not in place.regions[task]:
            problems.append(f"{where}/indicator: task '{task}' has no region '{item['indicator']}'")
    return [f"{parts.path}:{problem}" for problem in problems]


def parts_by_task(parts: Document, place: Course) -> dict[str, set[str]]:
    """Part names each task can show: its visuals' parts and the parts of textures it draws."""
    result: dict[str, set[str]] = {task: set() for task in place.frames}
    for item in parts.data.get("textures", []):
        names = {part["part"] for part in item["parts"]}
        if "task" in item:
            tasks = {item["task"]}
        else:
            tasks = place.texture_tasks.get(item["texture"], set())
        for task in tasks:
            result.setdefault(task, set()).update(names)
    for item in parts.data.get("visuals", []):
        names = {item["part"]} if "part" in item else set(item["materials"].values())
        result.setdefault(item["task"], set()).update(names)
    return result


def indicator_colours(parts: Document, place: Course) -> dict[str, set[str]]:
    """Task id -> colour names the indicator regions its parts carry can show."""
    result: dict[str, set[str]] = {}
    for item in parts.data.get("visuals", []):
        region = item.get("indicator")
        if region is not None:
            colours = place.regions.get(item["task"], {}).get(region, set())
            result.setdefault(item["task"], set()).update(colours)
    return result


def mapping_parts(mapping: list[str] | dict[str, Any]) -> list[str]:
    return list(mapping) if isinstance(mapping, list) else list(mapping["parts"])


def _model_classes(data: dict[str, Any]) -> set[str]:
    return {name for model in data["models"].values() for name in model["classes"]}


def unmodelled_classes(data: dict[str, Any]) -> list[str]:
    """Classes mapped or configured but in no model: a typo would silently label nothing."""
    known = _model_classes(data)
    problems = []
    for task, classes in data["tasks"].items():
        for name in classes:
            if name not in known:
                problems.append(f"/tasks/{task}/{name}: class '{name}' is in no model's classes")
    for name in data.get("classes", {}):
        if name not in known:
            problems.append(f"/classes/{name}: class '{name}' is in no model's classes")
    return problems


def label_warnings(labels: Document) -> list[str]:
    """Model classes no task maps: that class never gets a label."""
    mapped = {name for classes in labels.data["tasks"].values() for name in classes}
    return [
        f"{labels.path}:/models/{model}: class '{name}' has no mapping in any task"
        for model, item in labels.data["models"].items()
        for name in item["classes"]
        if name not in mapped
    ]


def check_labels(labels: Document, parts: Document, place: Course) -> list[str]:
    """Label pack against a tasks pack: pack id, task ids, part patterns, indicator colours."""
    data = labels.data
    problems: list[str] = []
    if data["tasks_pack"] != place.tasks_id:
        problems.append(
            f"/tasks_pack: '{data['tasks_pack']}' but the scenario selects '{place.tasks_id}'"
        )
    problems += unmodelled_classes(data)
    available = parts_by_task(parts, place)
    colours = indicator_colours(parts, place)
    for task, classes in data["tasks"].items():
        if task not in place.frames:
            problems.append(f"/tasks/{task}: no task '{task}' in tasks pack '{place.tasks_id}'")
            continue
        names = available.get(task, set())
        unconditional: dict[str, str] = {}
        for name, mapping in classes.items():
            where = f"/tasks/{task}/{name}"
            for pattern in mapping_parts(mapping):
                matched = fnmatch.filter(sorted(names), pattern)
                if not matched:
                    problems.append(f"{where}: '{pattern}' matches no part of task '{task}'")
                if isinstance(mapping, dict) and "when" in mapping:
                    continue
                for part in matched:
                    other = unconditional.setdefault(part, name)
                    if other != name:
                        problems.append(f"{where}: part '{part}' is already class '{other}'")
            when = mapping.get("when", {}) if isinstance(mapping, dict) else {}
            colour = when.get("indicator")
            if colour is not None and colour not in colours.get(task, set()):
                shown = ", ".join(sorted(colours.get(task, set()))) or "none"
                problems.append(f"{where}/when/indicator: '{colour}' never shown (task: {shown})")
    return [f"{labels.path}:{problem}" for problem in problems]


def check_model(labels: Document, model: str, resolved: ResolvedScenario) -> list[str]:
    """The model exists and its camera is a camera sensor of the scenario's robot."""
    models = labels.data["models"]
    if model not in models:
        return [f"{labels.path}:/models: no model '{model}' (has: {', '.join(models)})"]
    camera = models[model]["camera"]
    for sensor in resolved.robot.get("sensors", []):
        if sensor["id"] == camera:
            if "resolution_px" not in sensor.get("parameters", {}):
                return [f"{resolved.path}: robot sensor '{camera}' is not a camera"]
            return []
    return [f"{resolved.path}: robot has no camera sensor '{camera}' (model '{model}')"]


def native_resolution(resolved: ResolvedScenario, camera: str) -> list[int]:
    for sensor in resolved.robot.get("sensors", []):
        if sensor["id"] == camera:
            return [int(value) for value in sensor["parameters"]["resolution_px"]]
    raise PackError(f"{resolved.path}: robot has no camera sensor '{camera}'")


def check_dataset(dataset: Document, place: Course) -> list[str]:
    """Dataset task ids and sampler frames against a tasks pack."""
    problems: list[str] = []
    for task, block in dataset.data.get("tasks", {}).items():
        where = f"/tasks/{task}"
        if task not in place.frames:
            problems.append(f"{where}: no task '{task}' in tasks pack '{place.tasks_id}'")
            continue
        frames = block["sampler"].get("frame", "task")
        for frame in [frames] if isinstance(frames, str) else frames:
            if frame not in place.frames[task]:
                problems.append(f"{where}/sampler/frame: task '{task}' has no frame '{frame}'")
    groups = dataset.data.get("randomize", {}).get("placement", {}).get("groups", [])
    grouped: set[str] = set()
    for index, group in enumerate(groups):
        where = f"/randomize/placement/groups/{index}"
        for task in group:
            if task not in place.frames:
                problems.append(f"{where}: no task '{task}' in tasks pack '{place.tasks_id}'")
            elif task in grouped:
                problems.append(f"{where}: task '{task}' is already in a group")
            grouped.add(task)
    return [f"{dataset.path}:{problem}" for problem in problems]
