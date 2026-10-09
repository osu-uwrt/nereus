"""Editing a scenario's course in place: task placements, loose objects and fixed run options.

``set_course`` is what a course editor (the viewer's Edit map > Sim course) saves through. The change is written
with the round-trip emitter, so comments, key order and the untouched entries stay as authored, and only after
the edited scenario resolved: an edit that would not load is never written.
"""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from typing import Any

from ruamel.yaml.comments import CommentedMap, CommentedSeq

from ._document import PackError
from ._resolve import load_pack, resolve_scenario

DIGITS = 6  # written poses: micrometers and microdegrees


def _flow(pairs: list[tuple[str, Any]]) -> CommentedMap:
    """A one-line YAML mapping (as the scenarios write placements) of these keys, in order."""
    result = CommentedMap(pairs)
    result.fa.set_flow_style()
    return result


def _position(values: Any) -> CommentedSeq:
    position = CommentedSeq([round(float(value), DIGITS) for value in values])
    position.fa.set_flow_style()
    return position


def _yaw(value: Any) -> float:
    """Degrees wrapped to (-180, 180], rounded."""
    wrapped = math.fmod(math.fmod(float(value) + 180.0, 360.0) + 360.0, 360.0) - 180.0
    return round(180.0 if wrapped == -180.0 else wrapped, DIGITS)


def _check(course: dict[str, Any]) -> list[str]:
    """Shape problems of a course edit (the resolver checks the values)."""
    problems: list[str] = []
    for key in course:
        if key not in ("task_placements", "task_frames", "run_options"):
            problems.append(f"course: unknown key '{key}'")
    for key, fields in (("task_placements", ("task",)), ("task_frames", ("task", "frame"))):
        for index, entry in enumerate(course.get(key, [])):
            for name in (*fields, "position_m", "yaw_deg"):
                if not isinstance(entry, dict) or name not in entry:
                    problems.append(f"course /{key}/{index}: needs {name}")
    if not isinstance(course.get("run_options", {}), dict):
        problems.append("course /run_options: must be a mapping")
    return problems


def set_course(path: Path, course: dict[str, Any]) -> Path:
    """Apply a course edit to the scenario at `path` (a pack folder or file) and save it; returns the file.

    ``course`` holds any of: ``task_placements`` (world placements replacing the named tasks' own),
    ``task_frames`` (loose-object poses, replacing an entry for the same task and frame) and ``run_options``
    (values for the tasks pack's run options). Raises PackError, leaving the file untouched, when the edit is
    malformed or the edited scenario does not resolve.
    """
    problems = _check(course)
    if problems:
        raise PackError(problems)
    document = load_pack(Path(path))
    if document.kind != "scenario":
        raise PackError(f"{document.path}: expected a scenario, found kind '{document.kind}'")
    data = document.data

    # Placements: each named task's entry replaced in place (in the world, so relative_to and rpy go)
    placements = data["task_placements"]
    for entry in course.get("task_placements", []):
        index = next(
            (i for i, item in enumerate(placements) if item["task"] == entry["task"]), None
        )
        if index is None:
            raise PackError(
                f"{document.path}: /task_placements: task '{entry['task']}' has no placement"
            )
        placements[index] = _flow(
            [
                ("task", entry["task"]),
                ("position_m", _position(entry["position_m"])),
                ("yaw_deg", _yaw(entry["yaw_deg"])),
            ]
        )

    # Loose objects: task_frames (added after task_placements when the scenario has none yet)
    for entry in course.get("task_frames", []):
        if "task_frames" not in data:
            data.insert(list(data).index("task_placements") + 1, "task_frames", CommentedSeq())
        frames = data["task_frames"]
        item = _flow(
            [
                ("task", entry["task"]),
                ("frame", entry["frame"]),
                ("position_m", _position(entry["position_m"])),
                ("yaw_deg", _yaw(entry["yaw_deg"])),
            ]
        )
        index = next(
            (
                i
                for i, old in enumerate(frames)
                if old["task"] == entry["task"] and old["frame"] == entry["frame"]
            ),
            None,
        )
        if index is None:
            frames.append(item)
        else:
            frames[index] = item

    for key, value in course.get("run_options", {}).items():
        data["run"]["options"][key] = value

    # Resolve the edited scenario from a copy beside it (its relative pack paths hold), then replace the file
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{document.path.name}.", suffix=".yaml", dir=document.path.parent
    )
    os.close(descriptor)
    try:
        document.save(Path(temporary))
        resolve_scenario(Path(temporary))
    except PackError as error:  # name the scenario, not the copy
        raise PackError(
            [
                problem.replace(str(Path(temporary).resolve()), str(document.path))
                for problem in error.problems
            ]
        ) from None
    finally:
        Path(temporary).unlink(missing_ok=True)
    return document.save()
