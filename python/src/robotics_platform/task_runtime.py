"""Observe pack task geometry and evaluate pure scoring hooks without stepping a plant."""

from __future__ import annotations

import hashlib
import itertools
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import asdict
from types import MappingProxyType
from typing import Any

import numpy as np

from . import _native as native
from .pack_runtime import _frames, _pose
from .packs import ResolvedScenario
from .packs._semantics import hook_module
from .task_projectiles import PerforatedPanel, vector3
from .task_regions import PortalEvent, PortalTracker


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


def _time(value: int) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("time_ns must be a nonnegative integer")
    return value


def _placement(config: Mapping[str, Any]) -> native.Pose:
    yaw = math.radians(config["yaw_deg"]) / 2
    return _pose(config["position_m"], [math.cos(yaw), 0, 0, math.sin(yaw)])


def _envelope(robot: dict[str, Any]) -> np.ndarray:
    frames = _frames(robot["frames"])
    reference_from_root = frames.from_root(robot["reference_frame"]).inverse()
    boxes = {box["id"]: box for box in robot["collision_boxes"]}
    vertices = []
    for identifier in robot["scoring_envelope"]["collision_boxes"]:
        box = boxes[identifier]
        pose = reference_from_root.compose(
            _pose(box["center_m"], box["orientation_wxyz"])
        )
        half = np.asarray(box["size_m"]) / 2
        vertices.extend(pose.apply(half * signs) for signs in itertools.product((-1, 1), repeat=3))
    for point in robot["scoring_envelope"]["points"]:
        pose = reference_from_root.compose(frames.from_root(point["frame"]))
        vertices.append(pose.apply(point["position_m"]))
    if not vertices:
        raise ValueError("task traversal requires a nonempty robot scoring envelope")
    return np.asarray(vertices)


class TaskRuntime:
    """Explicit task observer with detached, recursively read-only snapshots.

    Implemented geometry includes portals, projectile panels and robot/prop contact-entry
    events and event-points rules. Unsupported selected types fail at construction; use
    task_ids explicitly for a partial scripted acceptance run. Contact pairs are supplied
    by the physical contact owner, not inferred from a scoring region. Task hooks are
    trusted pack Python, not a security sandbox; only state, events and parameters are
    passed to them. Hook failures stop observation until reset.
    """

    def __init__(self, resolved: ResolvedScenario, *, task_ids: Sequence[str] | None = None):
        changed = resolved.changed_sources()
        if changed:
            raise ValueError(f"resolved sources changed: {changed}")
        definitions = {task["id"]: task for task in resolved.task_definitions}
        selected = list(definitions) if task_ids is None else list(task_ids)
        if len(set(selected)) != len(selected) or not selected or set(selected) - definitions.keys():
            raise ValueError("task_ids must select unique existing tasks")
        # Copy config so edits to the caller's resolved data cannot alter a running judge.
        self._tasks = json.loads(json.dumps({key: definitions[key] for key in selected}))
        self._options = dict(resolved.run_options)
        self._auto_start = resolved.scenario["run"]["auto_start"]
        self._seed = resolved.scenario["seed"]
        surface = (resolved.pool["parameters"]["water_level_m"]
                   + resolved.scenario["pool_placement"]["position_m"][2])
        self._environment = {"surface_z_m": surface,
                             "floor_z_m": surface - resolved.pool["parameters"]["depth_m"]}
        envelope = _envelope(resolved.robot)
        placements = {item["task"]: _placement(item)
                      for item in resolved.scenario["task_placements"]}
        self._portals: dict[tuple[str, str], PortalTracker] = {}
        self._panels: dict[tuple[str, str], PerforatedPanel] = {}
        self._contacts: dict[tuple[str, str], list[dict[str, Any]]] = {}
        self._rules: list[tuple[str, dict[str, Any]]] = []
        for identifier, task in self._tasks.items():
            for region in task["regions"]:
                if region["type"] == "perforated_panel":
                    self._panels[identifier, region["id"]] = PerforatedPanel(
                        region["parameters"], placements[identifier])
                    continue
                if region["type"] != "rectangular_portal":
                    raise ValueError(f"task {identifier}: unsupported region {region['type']!r}")
                self._portals[identifier, region["id"]] = PortalTracker(
                    region["parameters"], placements[identifier], envelope,
                    self._environment["floor_z_m"],
                )
            for event in task["events"]:
                if event["type"] == "contact" and event["parameters"]["with"] == "robot":
                    self._contacts.setdefault((identifier, event["parameters"]["prop"]), []).append(event)
                elif event["type"] not in ("pass_through", "hit"):
                    raise ValueError(f"task {identifier}: unsupported event {event['type']!r}")
            for rule in task["scoring"]:
                if rule["type"] != "event_points":
                    raise ValueError(f"task {identifier}: unsupported scoring {rule['type']!r}")
                self._rules.append((identifier, rule))
        self._hook_sources = []
        pack_path = (resolved.path.parent / resolved.scenario["tasks"]).resolve()
        root = pack_path if pack_path.is_dir() else pack_path.parent
        for hook in resolved.tasks["scoring_hooks"]:
            path = hook_module(root, hook["module"])
            if path is None:
                raise ValueError("task hook escapes its pack")
            source = path.read_bytes()
            if hashlib.sha256(source).hexdigest() != resolved.source_sha256.get(path):
                raise ValueError(f"task hook is not the resolved source: {path}")
            self._hook_sources.append((path, source, hook["function"], _freeze(hook["parameters"])))
        self.reset()

    def reset(self) -> None:
        """Clear this observer and scoring state; the owner resets other components."""
        self._hooks = []
        for path, source, function, parameters in self._hook_sources:
            namespace = {"__name__": f"pack_hook_{hashlib.sha256(source).hexdigest()}",
                         "__file__": str(path)}
            exec(compile(source, str(path), "exec"), namespace)
            callback = namespace.get(function)
            if not callable(callback):
                raise ValueError(f"task hook {path}:{function} is not callable")
            self._hooks.append((callback, parameters))
        for portal in self._portals.values():
            portal.reset()
        self._scores: dict[str, int] = {}
        self._history: list[dict[str, Any]] = []
        self._award_counts: dict[tuple[str, str], int] = {}
        self._active_contacts: set[tuple[str, str]] = set()
        self._projectiles: dict[int, tuple[str, float]] = {}
        self._last_time = 0
        self._failed = False
        self._run = {"running": self._auto_start, "ended": False, "started_ns": 0,
                     "options": dict(self._options), "seed": self._seed}

    def snapshot(self) -> Mapping[str, Any]:
        return _freeze({"run": self._run, "scores": self._scores, "history": self._history,
                        "tasks": {key: {"frames": {f["id"]: f for f in task["frames"]}}
                                  for key, task in self._tasks.items()},
                        "environment": self._environment})

    def start(self, time_ns: int) -> None:
        """Start a fresh scoring run at the caller's current simulation time."""
        self._check_time(time_ns)
        self.reset()
        self._last_time = time_ns
        self._run.update(running=True, started_ns=time_ns)

    def stop(self, time_ns: int) -> tuple[Mapping[str, Any], ...]:
        self._check_time(time_ns)
        events = []
        for (task, region), portal in self._portals.items():
            for event in portal.finish_attempt(time_ns):
                events.extend(self._portal_events(task, region, event))
        try:
            result = self._evaluate(events)
        except Exception:
            self._failed = True
            raise
        self._last_time = time_ns
        self._run["running"] = False
        return result

    def _check_time(self, time_ns: int) -> None:
        _time(time_ns)
        if self._failed:
            raise RuntimeError("task observer failed; reset before continuing")
        if time_ns < self._last_time:
            raise ValueError("task time cannot go backwards without reset")

    def observe(self, time_ns: int, world_reference: native.Pose, *,
                contacts: Sequence[tuple[str, str]] = ()) -> tuple[Mapping[str, Any], ...]:
        """Observe a robot reference-frame pose, never a COM pose by implication."""
        self._check_time(time_ns)
        active = set(contacts)
        if active - self._contacts.keys():
            raise ValueError("contacts must identify selected task/prop contact bindings")
        world_reference = native.Pose().compose(world_reference)
        try:
            events = []
            for (task, region), portal in self._portals.items():
                for event in portal.observe(time_ns, world_reference):
                    events.extend(self._portal_events(task, region, event))
            for pair in sorted(active - self._active_contacts):
                for binding in self._contacts[pair]:
                    events.append({"id": binding["id"], "type": "contact", "task": pair[0],
                                   "region": "", "time_ns": time_ns,
                                   "data": {"prop": pair[1], "with": "robot"}})
            result = self._evaluate(events)
            self._active_contacts = active
            self._last_time = time_ns
            return result
        except Exception:
            self._failed = True
            raise

    def _portal_events(self, task: str, region: str, event: PortalEvent) -> list[dict[str, Any]]:
        data = asdict(event)
        del data["kind"], data["time_ns"]
        base = {"type": event.kind, "task": task, "region": region,
                "time_ns": event.time_ns, "data": data}
        if event.kind == "attempt_finished":
            return [{"id": f"{region}:attempt_finished", **base}]
        return [{"id": binding["id"], **base} for binding in self._tasks[task]["events"]
                if binding["type"] == event.kind
                and binding["parameters"]["region"] == region
                and binding["parameters"]["from_side"] == event.from_side
                and binding["parameters"]["to_side"] == event.to_side]

    def release_projectile(self, time_ns: int, identifier: int, mechanism_type: str,
                           tip_world: Sequence[float], radius_m: float
                           ) -> tuple[Mapping[str, Any], ...]:
        """Observe a successful physical release; the mechanism owner supplies its tip."""
        self._check_time(time_ns)
        if type(identifier) is not int or identifier < 0 or identifier in self._projectiles:
            raise ValueError("projectile identifiers must be unique nonnegative integers")
        if mechanism_type not in ("launcher", "dropper"):
            raise ValueError("unsupported projectile mechanism type")
        tip = vector3(tip_world, "projectile tip")
        if isinstance(radius_m, bool) or not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError("projectile radius must be positive and finite")
        events = []
        for (task, region), panel in self._panels.items():
            bindings = self._tasks[task]["events"]
            if any(b["type"] == "hit" and b["parameters"]["region"] == region
                   and b["parameters"]["projectile_mechanism_type"] == mechanism_type
                   for b in bindings):
                events.append({"id": "payload_released", "type": "payload_released",
                               "task": task, "region": region, "time_ns": time_ns,
                               "data": {"projectile_id": identifier,
                                        "mechanism_type": mechanism_type,
                                        "release_distance_m": panel.release_distance(tip)}})
        try:
            result = self._evaluate(events)
        except Exception:
            self._failed = True
            raise
        self._projectiles[identifier] = (mechanism_type, float(radius_m))
        self._last_time = time_ns
        return result

    def observe_projectile(self, time_ns: int, identifier: int,
                           start_center_world: Sequence[float], end_center_world: Sequence[float],
                           axis_world: Sequence[float]) -> tuple[Mapping[str, Any], ...]:
        """Judge swept mesh-center motion; hit data tells the physical owner to stop.

        The original panel contact uses mesh centers; only release-distance scoring uses
        the projectile tip. Keeping these distinct preserves near-panel outcomes.
        """
        self._check_time(time_ns)
        if type(identifier) is not int or identifier < 0 or identifier not in self._projectiles:
            raise ValueError("projectile must be released before observing its motion")
        start = vector3(start_center_world, "projectile start center")
        end = vector3(end_center_world, "projectile end center")
        axis = vector3(axis_world, "projectile axis")
        if abs(float(np.linalg.norm(axis)) - 1) > 1e-6:
            raise ValueError("projectile axis must be unit length")
        mechanism, radius = self._projectiles[identifier]
        events = []
        for (task, region), panel in self._panels.items():
            hit = panel.intersect(start, end, axis, radius)
            if hit is None:
                continue
            for binding in self._tasks[task]["events"]:
                p = binding["parameters"]
                if (binding["type"] == "hit" and p["region"] == region
                        and p["projectile_mechanism_type"] == mechanism
                        and p["outcome"] == hit.outcome):
                    events.append({"id": binding["id"], "type": "hit", "task": task,
                                   "region": region, "time_ns": time_ns,
                                   "data": {"projectile_id": identifier,
                                            "mechanism_type": mechanism, "outcome": hit.outcome,
                                            "hole_id": hit.hole_id, "hole_class": hit.hole_class,
                                            "hole_size": hit.hole_size,
                                            "hit_point_local": hit.point_local,
                                            "stop_projectile": p["stop_projectile"]}})
        try:
            result = self._evaluate(events)
        except Exception:
            self._failed = True
            raise
        self._last_time = time_ns
        return result

    def _evaluate(self, events: list[dict[str, Any]]) -> tuple[Mapping[str, Any], ...]:
        if not events:
            return ()
        scores = dict(self._scores)
        counts = dict(self._award_counts)
        if self._run["running"] and not self._run["ended"]:
            for task, rule in self._rules:
                key, parameters = (task, rule["id"]), rule["parameters"]
                for event in events:
                    if (event["task"] == task and event["id"] == parameters["event"]
                            and counts.get(key, 0) < parameters.get("max_awards", math.inf)):
                        row = f"{task}/{rule['id']}"
                        scores[row] = scores.get(row, 0) + parameters["points"]
                        counts[key] = counts.get(key, 0) + 1
        emitted = []
        for hook, parameters in self._hooks:
            state = dict(self.snapshot())
            state["scores"] = scores
            result = hook(_freeze(state), _freeze(events), parameters)
            # Own all output and reject non-data/NaN values before changing the ledger.
            result = json.loads(json.dumps(result, allow_nan=False))
            if not isinstance(result, dict) or set(result) != {"scores", "events"}:
                raise ValueError("hook must return scores and events")
            if not isinstance(result["scores"], list) or not isinstance(result["events"], list):
                raise ValueError("hook scores and events must be lists")
            for change in result["scores"]:
                if (not isinstance(change, dict) or set(change) != {"row", "points"}
                        or not isinstance(change["row"], str) or not change["row"]
                        or type(change["points"]) is not int):
                    raise ValueError("hook score changes require a row and integer points")
                scores[change["row"]] = change["points"]
            for event in result["events"]:
                self._validate_event(event, events)
                emitted.append(event)
        self._scores, self._award_counts = scores, counts
        self._history.extend([*events, *emitted])
        return _freeze([*events, *emitted])

    def _validate_event(self, event: Any, inputs: list[dict[str, Any]]) -> None:
        if (not isinstance(event, dict)
                or set(event) != {"id", "type", "task", "region", "time_ns", "data"}
                or any(not isinstance(event[key], str) for key in ("id", "type", "task", "region"))
                or not event["id"] or not event["type"] or event["task"] not in self._tasks
                or not isinstance(event["data"], dict)):
            raise ValueError("hook emitted a malformed task event")
        _time(event["time_ns"])
        if not min(item["time_ns"] for item in inputs) <= event["time_ns"] <= max(
                item["time_ns"] for item in inputs):
            raise ValueError("hook event time must belong to the input interval")
