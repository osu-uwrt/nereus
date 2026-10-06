"""Pack loading and scenario resolution into one reproducible manifest (no runtime, no ROS)."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import tempfile
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import _semantics as semantics
from ._definitions import schema_problems
from ._document import (
    DOCUMENT_KINDS,
    PackDocument,
    PackError,
    canonical_file,
    read_source,
    read_yaml,
)
from ._frames import express_in_body
from ._placements import resolve_placements

# Identity of the resolved manifest written by ``ResolvedScenario.dump`` and read by the runtime
FORMAT = "nereus.resolved_scenario"
FORMAT_VERSION = 1


# Prefix problems ("/json/pointer: message") with the file they were found in
def _prefixed(path: Path, problems: list[str]) -> list[str]:
    return [f"{path}:{problem}" for problem in problems]


def _includes(document: PackDocument, data: dict[str, Any], hashes: dict[Path, str]) -> list[str]:
    """Load and check every task include of a tasks pack."""
    problems: list[str] = []
    asset_ids = {item["id"] for item in data.get("assets", [])}
    for relative in data["tasks"]:
        target = semantics.inside(document.root, relative)
        if target is None:
            continue  # reported by semantics.tasks
        if not target.is_file():
            problems.append(f"{document.path}:/tasks: include '{relative}' does not exist")
            continue
        try:
            include = _open(target, hashes)
        except PackError as error:
            problems += error.problems
            continue
        if include.kind != "task":
            problems.append(f"{include.path}: task include must declare kind 'task'")
            continue
        problems += _prefixed(include.path, semantics.task(include.plain(), asset_ids))
        document.includes.append(include)

    semantics.duplicates((item.plain()["id"] for item in document.includes), "task", problems)
    return problems


def _check(document: PackDocument, hashes: dict[Path, str]) -> list[str]:
    """Schema and single-pack semantics; records verified file hashes."""
    data = document.plain()
    problems = schema_problems(document.kind, data)
    if problems:
        return _prefixed(document.path, problems)

    # Semantics run only on schema-valid data; each kind has its own checks
    kind = document.kind
    if kind not in ("task", "scenario"):
        problems += semantics.assets(data, document.root, hashes)
    if kind == "robot":
        problems += semantics.robot(data)
    elif kind == "pool":
        problems += semantics.pool(data)
    elif kind == "tasks":
        problems += semantics.tasks(data, document.root)
    elif kind == "task":
        problems += semantics.task(data, None)
    elif kind == "bridge":
        problems += semantics.bridge(data)
    elif kind == "equipment":
        problems += semantics.equipment(data)
    problems = _prefixed(document.path, problems)

    # A tasks pack's includes are loaded only when the pack itself is clean
    if kind == "tasks" and not problems:
        problems += _includes(document, data, hashes)
    return problems


def _open(path: Path, hashes: dict[Path, str]) -> PackDocument:
    """Read, kind-check and validate one document, recording its digest in ``hashes``."""
    given = Path(path)
    file = canonical_file(given).resolve()
    source = read_source(file)
    data = read_yaml(file, source)
    kind = data.get("kind")
    if not isinstance(kind, str) or kind not in DOCUMENT_KINDS:
        raise PackError(f"{file}: unknown or missing kind {kind!r}")
    if given.is_dir() and file.stem != kind:
        raise PackError(f"{file}: folder canonical file declares kind '{kind}'")

    document = PackDocument(file, kind, data, source=source)
    hashes[file] = source.sha256
    problems = _check(document, hashes)
    if problems:
        raise PackError(problems)
    return document


def load_pack(path: Path) -> PackDocument:
    """Load and validate one pack (folder with canonical <kind>.yaml, or a specific file).

    Raises PackError for invalid syntax, duplicate or non-string keys, non-finite numbers,
    unknown active fields, bad references, invalid physics or paths escaping the pack. A tasks
    pack also validates its task includes.
    """
    return _open(path, {})


@dataclass
class ResolvedScenario:
    """Validated scenario composition. Pack contents are detached plain data.

    ``source_sha256`` holds the digest of the exact bytes each document and asset had when
    resolved.
    """

    path: Path
    scenario: dict[str, Any]
    robot: dict[str, Any]
    pool: dict[str, Any]
    tasks: dict[str, Any]
    bridge: dict[str, Any] | None
    task_definitions: list[dict[str, Any]]
    run_options: dict[str, Any]
    sources: list[Path]
    source_sha256: dict[Path, str] = field(default_factory=dict)
    equipment: dict[str, Any] | None = None

    def changed_sources(self) -> list[Path]:
        """Sources whose current bytes differ from those resolved (or that disappeared)."""
        changed = []
        for source in self.sources:
            try:
                if semantics.sha256(source) != self.source_sha256[source]:
                    changed.append(source)
            except OSError:
                changed.append(source)
        return changed

    def asset_paths(self) -> dict[str, dict[str, str]]:
        """Absolute path of every declared, present asset, per selected pack (robot/pool/tasks/equipment)."""
        result: dict[str, dict[str, str]] = {}
        roles = [("robot", self.robot), ("pool", self.pool), ("tasks", self.tasks)]
        if self.equipment is not None:
            roles.append(("equipment", self.equipment))
        for role, document in roles:
            root = (self.path.parent / self.scenario[role]).resolve()
            root = root if root.is_dir() else root.parent
            result[role] = {
                item["id"]: str((root / item["path"]).resolve())
                for item in document.get("assets", [])
            }
        return result

    def manifest(self) -> dict[str, Any]:
        """The runtime document, with a ``content_sha256`` over its canonical JSON."""
        base = self.path.parent
        body = {
            "format": FORMAT,
            "version": FORMAT_VERSION,
            "scenario_file": self.path.name,
            "scenario": self.scenario,
            "robot": self.robot,
            "pool": self.pool,
            "tasks": self.tasks,
            "task_definitions": self.task_definitions,
            "bridge": self.bridge,
            **({"equipment": self.equipment} if self.equipment is not None else {}),
            "run_options": self.run_options,
            "sources": [
                {
                    "path": Path(os.path.relpath(source, base)).as_posix(),
                    "sha256": self.source_sha256[source],
                }
                for source in self.sources
            ],
        }

        # Digest of the body in canonical form (sorted keys, no whitespace), before it is added
        canonical = json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False)
        body["content_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
        return body

    def dump(self, path: Path) -> Path:
        """Write the resolved manifest as JSON; identical inputs give identical bytes.

        Refuses to write if any source changed since resolution (never a mixed snapshot).
        """
        changed = self.changed_sources()
        if changed:
            raise PackError(
                [f"{source}: changed since the scenario was resolved" for source in changed]
            )

        # Atomic write: temp file in the target folder, then rename over the target
        target = Path(path)
        text = json.dumps(self.manifest(), indent=2, allow_nan=False) + "\n"
        descriptor, temporary = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(text)
            os.replace(temporary, target)
        except BaseException:
            Path(temporary).unlink(missing_ok=True)
            raise
        return target


# Seconds to the nearest whole nanosecond
def _nanoseconds(seconds: float) -> int:
    return round(seconds * 1e9)


def _add_runtime_times(scenario: dict[str, Any], robot: dict[str, Any]) -> None:
    """Add the whole-nanosecond times the runtime steps on next to the authored seconds and rates."""
    scenario["timestep_ns"] = _nanoseconds(scenario["timestep_s"])
    for sensor in robot["sensors"]:
        sensor["period_ns"] = _nanoseconds(1 / sensor["rate_hz"])
        sensor["latency_ns"] = _nanoseconds(sensor.get("latency_s", 0))


def _override_thrusters(data: dict[str, Any], robot: dict[str, Any]) -> list[str]:
    """Apply the scenario's ``thruster_overrides`` to the plain robot in place; problems of the result.

    Each override selects thrusters (default all). A different ``type`` replaces their parameters; the same
    type merges over them. The overridden robot must still pass the robot schema and semantics.
    """
    overrides = data.get("thruster_overrides", [])
    if not overrides:
        return []

    problems: list[str] = []
    ids = [item["id"] for item in robot["thrusters"]]
    for index, override in enumerate(overrides):
        selected = override.get("thrusters", ids)
        problems += [
            f"/thruster_overrides/{index}/thrusters: unknown robot thruster '{name}'"
            for name in selected
            if name not in ids
        ]

        # Later overrides apply on top of earlier ones
        for item in robot["thrusters"]:
            if item["id"] not in selected:
                continue
            parameters = copy.deepcopy(override["parameters"])
            kind = override.get("type", item["type"])
            if kind != item["type"]:
                item["type"] = kind
                item["parameters"] = parameters
            else:
                item["parameters"] = {**item["parameters"], **parameters}
    if problems:
        return problems

    # Semantics assume a schema-valid robot, so they only run when the schema passes
    result = schema_problems("robot", robot) or semantics.robot(robot)
    return [f"/thruster_overrides: overridden robot {problem}" for problem in result]


def resolve_scenario(path: Path) -> ResolvedScenario:
    """Load a scenario and every selected pack and validate their cross-references."""
    hashes: dict[Path, str] = {}
    scenario = _open(path, hashes)
    if scenario.kind != "scenario":
        raise PackError(f"{scenario.path}: expected a scenario, found kind '{scenario.kind}'")
    data = scenario.plain()

    # Open every pack the scenario selects, each under its role; bridge and equipment are optional
    problems: list[str] = []
    documents: dict[str, PackDocument] = {}
    for role in ("robot", "pool", "tasks", "bridge", "equipment"):
        if role not in data:
            continue
        try:
            document = _open(scenario.root / data[role], hashes)
        except PackError as error:
            problems += error.problems
            continue
        if document.kind != role:
            problems.append(f"{scenario.path}:/{role}: selects a '{document.kind}' pack")
            continue
        documents[role] = document
    if problems:
        raise PackError(problems)

    # Plain copies of the selected packs; the robot is rewritten in place from here on
    tasks_document = documents["tasks"]
    tasks_data = tasks_document.plain()
    definitions = [include.plain() for include in tasks_document.includes]
    robot = documents["robot"].plain()
    problems += _prefixed(scenario.path, _override_thrusters(data, robot))
    if problems:
        raise PackError(problems)

    # Runtime-ready robot geometry and times, then cross-pack checks against that robot
    express_in_body(robot)
    _add_runtime_times(data, robot)
    bridge = documents["bridge"].plain() if "bridge" in documents else None
    if bridge is not None:
        problems += _prefixed(documents["bridge"].path, semantics.bridge_binding(bridge, robot))
    mechanism_types = Counter(item["type"] for item in robot["mechanisms"])
    scenario_problems, options = semantics.scenario(
        data, robot, tasks_data, [item["id"] for item in definitions], mechanism_types
    )
    problems += _prefixed(scenario.path, scenario_problems)
    problems += _prefixed(scenario.path, semantics.scenario_pool(data, documents["pool"].plain()))
    if problems:
        raise PackError(problems)

    # Placements last: they need valid task ids and the equipment pack's items
    equipment = documents["equipment"].plain() if "equipment" in documents else None
    task_ids = [item["id"] for item in definitions]
    problems += _prefixed(scenario.path, resolve_placements(data, task_ids, equipment))
    if problems:
        raise PackError(problems)

    return ResolvedScenario(
        path=scenario.path,
        scenario=data,
        robot=robot,
        pool=documents["pool"].plain(),
        tasks=tasks_data,
        bridge=bridge,
        equipment=equipment,
        task_definitions=definitions,
        run_options=options,
        sources=list(hashes),
        source_sha256=hashes,
    )
