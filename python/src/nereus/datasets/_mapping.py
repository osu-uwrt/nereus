"""A label-pack model as a lookup: (task, part, indicator) -> class id, and per-class options."""

from __future__ import annotations

import ast
import fnmatch
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from nereus.packs import PackError
from nereus.packs._document import plain, read_yaml

from ._documents import mapping_parts

DEFAULT_MIN_VISIBLE_PX = 25


@dataclass(frozen=True)
class Rule:
    name: str
    patterns: tuple[str, ...]
    indicator: str | None

    def matches_part(self, part: str) -> bool:
        return any(fnmatch.fnmatchcase(part, pattern) for pattern in self.patterns)


class ClassMap:
    """Class mapping of one label-pack model. Classes the model lacks are never exported."""

    def __init__(self, labels: dict[str, Any], model: str) -> None:
        models = labels["models"]
        if model not in models:
            raise PackError(f"label pack '{labels['id']}' has no model '{model}'")
        self.model = model
        self.camera: str = models[model]["camera"]
        self.names: list[str] = list(models[model]["classes"])
        self.max_range_m = float(models[model]["max_range_m"])
        self.min_visible_px = int(
            labels.get("export", {}).get("min_visible_px", DEFAULT_MIN_VISIBLE_PX)
        )
        options = labels.get("classes", {})
        self.outer = {name for name, item in options.items() if item.get("shape") == "outer"}
        self.rules: dict[str, list[Rule]] = {}
        for task, classes in labels["tasks"].items():
            rules = []
            for name, mapping in classes.items():
                when = mapping.get("when", {}) if isinstance(mapping, dict) else {}
                rules.append(Rule(name, tuple(mapping_parts(mapping)), when.get("indicator")))
            self.rules[task] = rules

    def classify(self, task: str | None, part: str, indicator: str | None) -> int | None:
        """Class id of an instance in this model, or None if it is not labelled."""
        for rule in self.rules.get(task or "", []):
            # Classes this model lacks are skipped, as in ``labelled``.
            if rule.name in self.names and rule.matches_part(part):
                if rule.indicator in (None, indicator):
                    return self.names.index(rule.name)
        return None

    def labelled(self, parts: dict[str, set[str]]) -> list[dict[str, str]]:
        """(task, part) pairs mapped to a class of this model, any indicator state."""
        pairs = []
        for task, rules in self.rules.items():
            for part in sorted(parts.get(task, set())):
                for rule in rules:
                    if rule.name in self.names and rule.matches_part(part):
                        pairs.append({"task": task, "part": part})
                        break
        return pairs


# ------------------------------------------------------------------ deployed class maps


def yolo_class_maps(path: Path) -> dict[str, list[str]]:
    """``<camera>_class_id_map`` strings of a ROS parameter file, as class lists in id order."""
    data = plain(read_yaml(path))
    found: dict[str, list[str]] = {}

    def visit(node: Any) -> None:
        if isinstance(node, dict):
            for key, value in node.items():
                if key.endswith("_class_id_map") and isinstance(value, str):
                    found[key.removesuffix("_class_id_map")] = _id_map(path, key, value)
                else:
                    visit(value)

    visit(data)
    return found


def _id_map(path: Path, key: str, text: str) -> list[str]:
    try:
        value = ast.literal_eval(text.strip())
    except (SyntaxError, ValueError):
        raise PackError(f"{path}: {key} is not a Python dict literal") from None
    if not isinstance(value, dict) or not all(
        isinstance(k, int) and isinstance(v, str) for k, v in value.items()
    ):
        raise PackError(f"{path}: {key} must map integer ids to class names")
    if sorted(value) != list(range(len(value))):
        raise PackError(f"{path}: {key} ids must be 0..{len(value) - 1}")
    return [value[index] for index in range(len(value))]


def class_order_problems(labels: dict[str, Any], config: Path) -> tuple[list[str], list[str]]:
    """Compare every model's classes with the config's map for its camera: (problems, notes)."""
    maps = yolo_class_maps(config)
    problems: list[str] = []
    notes: list[str] = []
    for name, model in labels["models"].items():
        camera = model["camera"]
        deployed = maps.get(camera, maps.get(name))
        if deployed is None:
            notes.append(f"model '{name}': {config} has no {camera}_class_id_map")
            continue
        if deployed == model["classes"]:
            notes.append(f"model '{name}': {len(deployed)} classes match {camera}_class_id_map")
            continue
        problems.append(f"model '{name}': classes differ from {camera}_class_id_map")
        for index in range(max(len(deployed), len(model["classes"]))):
            ours = model["classes"][index] if index < len(model["classes"]) else "-"
            theirs = deployed[index] if index < len(deployed) else "-"
            if ours != theirs:
                problems.append(f"  id {index}: label pack '{ours}', config '{theirs}'")
    return problems, notes
