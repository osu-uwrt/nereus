"""Observe pack task geometry and evaluate pure scoring hooks without stepping a plant."""

from __future__ import annotations

import hashlib
import itertools
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, field
from types import MappingProxyType
from typing import Any

import numpy as np

from . import _native as native
from .pack_runtime import _frames, _pose
from .packs import ResolvedScenario
from .packs._semantics import hook_module
from .task_projectiles import PerforatedPanel, vector3
from .task_regions import PortalEvent, PortalTracker
from .task_zones import Fact, OpenCrate, ProximityTarget, SurfaceTracker, TurnTracker


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


def _probe_point(robot: dict[str, Any], mechanism_type: str) -> np.ndarray:
    """Tip of the robot's single mechanism of this type, in the robot reference frame."""
    found = [item for item in robot["mechanisms"] if item["type"] == mechanism_type]
    if len(found) != 1:
        raise ValueError(f"task probe needs exactly one {mechanism_type!r} mechanism; "
                         f"robot has {len(found)}")
    frames = _frames(robot["frames"])
    pose = frames.from_root(robot["reference_frame"]).inverse().compose(
        frames.from_root(found[0]["frame"]))
    return np.asarray(pose.apply(found[0]["parameters"]["tip_position_m"]))


@dataclass(frozen=True)
class ProjectileStep:
    """Judged swept payload step: events plus the physical correction its owner must apply.

    stop: the payload has ended (blocked, landed or missed) and must no longer advance;
    position_world / velocity_world replace the owner's end-of-step centre and velocity, or
    are None when unchanged.
    """

    events: tuple[Mapping[str, Any], ...]
    stop: bool = False
    position_world: tuple[float, ...] | None = None
    velocity_world: tuple[float, ...] | None = None


@dataclass
class _Projectile:
    mechanism: str
    radius: float
    length: float
    released_ns: int
    tasks: set[str]
    entered: set[tuple[str, str]] = field(default_factory=set)
    active: bool = True
    scored: bool = False


_SURFACE_FACTS = ("surface_reached", "surface_lost", "facing_reached", "facing_lost", "breach")
_EVENT_TYPES = ("pass_through", "hit", "payload_landing", "drop_into", "activate", "rotation_judged", "attach",
                "detach", *_SURFACE_FACTS)
_MAX_PAYLOAD_AGE_S = 30.0


class TaskRuntime:
    """Explicit task observer with detached, recursively read-only snapshots.

    Implemented geometry: portals (optionally in a named task frame, with a depth band),
    projectile panels, open crates, proximity latches, octagon surfacing/facing, turn zones,
    and robot/prop contact-entry events. Unsupported selected types fail at construction; use
    task_ids explicitly for a partial scripted acceptance run. Contact pairs are supplied
    by the physical contact owner, not inferred from a scoring region. Task hooks are
    trusted pack Python, not a security sandbox; only state, events and parameters are
    passed to them. Hook failures stop observation until reset.

    Event sources: observe() samples the robot reference pose and emits region facts;
    release_projectile()/step_projectile() judge payload flight; observe_events() accepts
    events from a physical owner such as a prop world (attach, detach, drop_into on its own
    regions). Times are integer nanoseconds and must not decrease until reset.

    Projectile API (payload owner): release_projectile(time, id, mechanism_type, tip,
    radius[, length_m]) then, every tick, step_projectile(time, id, start_center, end_center,
    axis, velocity) -> ProjectileStep(events, stop, position_world, velocity_world). Crate walls,
    rim/floor landings, panel blocks and pool floor/wall/30 s misses are judged for every
    payload kind; the owner applies stop/position/velocity when set. Events: payload_released,
    hit (panels), payload_landing (crate landing: outcome inside | blocked) and miss (pool_floor |
    pool_wall_or_timeout, once per payload, emitted to each task that registered its release).
    A payload that already produced a panel or crate/miss result gets no further crate/miss
    events. observe_projectile() is the events-only view of the same step (velocity unknown).
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
        self._pool_size = (resolved.pool["parameters"]["length_m"],
                           resolved.pool["parameters"]["width_m"])
        self._pool_from_world = _placement(resolved.scenario["pool_placement"]).inverse()
        envelope = _envelope(resolved.robot)
        placements = {item["task"]: _placement(item)
                      for item in resolved.scenario["task_placements"]}
        self._portals: dict[tuple[str, str], PortalTracker] = {}
        self._panels: dict[tuple[str, str], PerforatedPanel] = {}
        self._crates: dict[tuple[str, str], OpenCrate] = {}
        self._targets: dict[tuple[str, str], ProximityTarget] = {}
        self._surfaces: dict[tuple[str, str], SurfaceTracker] = {}
        self._turns: dict[tuple[str, str], TurnTracker] = {}
        self._contacts: dict[tuple[str, str], list[dict[str, Any]]] = {}
        self._rules: list[tuple[str, dict[str, Any]]] = []
        for identifier, task in self._tasks.items():
            frames = {item["id"]: item for item in task["frames"]}

            def placed(parameters: Mapping[str, Any], *, task_id: str = identifier,
                       named: dict[str, Any] = frames) -> native.Pose:
                base = placements[task_id]
                if "frame" not in parameters:
                    return base
                frame = named[parameters["frame"]]
                return base.compose(_pose(frame["position_m"], frame["orientation_wxyz"]))

            for region in task["regions"]:
                key, parameters = (identifier, region["id"]), region["parameters"]
                kind = region["type"]
                if kind == "perforated_panel":
                    self._panels[key] = PerforatedPanel(parameters, placements[identifier])
                elif kind == "rectangular_portal":
                    self._portals[key] = PortalTracker(
                        parameters, placed(parameters), envelope,
                        self._environment["floor_z_m"])
                elif kind == "open_crate":
                    self._crates[key] = OpenCrate(parameters, placed(parameters))
                elif kind == "proximity_target":
                    self._targets[key] = ProximityTarget(
                        parameters, placed(parameters),
                        _probe_point(resolved.robot, parameters["probe_mechanism_type"]))
                    self._targets[key].indicator = dict(parameters.get("indicator", {}))
                elif kind == "surface":
                    positions = {name: placed({"frame": name}).translation
                                 for name in parameters["facing"]["targets"]}
                    self._surfaces[key] = SurfaceTracker(
                        parameters, placed(parameters), envelope, surface, positions)
                elif kind == "box":  # containment is judged by the rigid-body prop world
                    continue
                elif kind == "turn_zone":
                    self._turns[key] = TurnTracker(parameters, placed(parameters))
                else:
                    raise ValueError(f"task {identifier}: unsupported region {kind!r}")
            for event in task["events"]:
                if event["type"] == "contact" and event["parameters"]["with"] == "robot":
                    self._contacts.setdefault((identifier, event["parameters"]["prop"]), []).append(event)
                elif event["type"] not in _EVENT_TYPES:
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
            self._hook_sources.append((path, source, hook["function"], _freeze(hook["parameters"]),
                                       hook.get("status_function"), hook.get("feed_function")))
        self.reset()

    def reset(self, time_ns: int = 0) -> None:
        """Clear this observer and scoring state; the owner resets other components.

        time_ns is the owner's current simulation time: a scenario that auto-starts its run
        starts it now.
        """
        self._hooks = []
        self._status_hooks: list[tuple[Any, Any]] = []
        self._feed_hooks: list[tuple[Any, Any]] = []
        for path, source, function, parameters, status, feed in self._hook_sources:
            namespace = {"__name__": f"pack_hook_{hashlib.sha256(source).hexdigest()}",
                         "__file__": str(path)}
            exec(compile(source, str(path), "exec"), namespace)
            callback = namespace.get(function)
            if not callable(callback):
                raise ValueError(f"task hook {path}:{function} is not callable")
            self._hooks.append((callback, parameters))
            for name, table in ((status, self._status_hooks), (feed, self._feed_hooks)):
                if name is not None:
                    extra = namespace.get(name)
                    if not callable(extra):
                        raise ValueError(f"task hook {path}:{name} is not callable")
                    table.append((extra, parameters))
        for tracker in (*self._portals.values(), *self._targets.values(),
                        *self._surfaces.values(), *self._turns.values()):
            tracker.reset()
        self._scores: dict[str, int] = {}
        self._history: list[dict[str, Any]] = []
        self._award_counts: dict[tuple[str, str], int] = {}
        self._active_contacts: set[tuple[str, str]] = set()
        self._projectiles: dict[int, _Projectile] = {}
        self._last_time = _time(time_ns)
        self._failed = False
        self._run = {"running": self._auto_start, "ended": False, "started_ns": time_ns,
                     "stopped_ns": None, "options": dict(self._options), "seed": self._seed}

    def snapshot(self) -> Mapping[str, Any]:
        return _freeze({"run": self._run, "scores": self._scores, "history": self._history,
                        "tasks": {key: {"frames": {f["id"]: f for f in task["frames"]}}
                                  for key, task in self._tasks.items()},
                        "environment": self._environment,
                        "latched": {f"{task}/{region}": target.latched
                                    for (task, region), target in self._targets.items()}})

    def start(self, time_ns: int, options: Mapping[str, Any] | None = None) -> None:
        """Start a fresh scoring run at the caller's current simulation time.

        options override the scenario's run options for this run only; every key must be a
        declared option and the owner validates values (see Session.run_start).
        """
        self._check_time(time_ns)
        unknown = set(options or ()) - self._options.keys()
        if unknown:
            raise ValueError(f"unknown run options {sorted(unknown)}")
        self.reset(time_ns)
        self._run["options"].update(options or {})
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
        if self._run["running"]:
            self._run.update(running=False, stopped_ns=time_ns)
        return result

    def describe(self) -> dict[str, Any]:
        """Extra scalar run-score fields declared by the pack hooks (status_function)."""
        fields: dict[str, Any] = {}
        state = self.snapshot()
        for callback, parameters in self._status_hooks:
            fields.update(json.loads(json.dumps(callback(state, parameters), allow_nan=False)))
        return fields

    def feed(self, events: Sequence[Mapping[str, Any]], context: Mapping[str, Any]
             ) -> list[dict[str, Any]]:
        """Operator event-feed items (pack feed_function) for events already committed."""
        items: list[dict[str, Any]] = []
        state = self.snapshot()
        for callback, parameters in self._feed_hooks:
            items += json.loads(json.dumps(
                callback(state, _freeze(list(events)), _freeze(context), parameters),
                allow_nan=False))
        return items

    def indicators(self) -> list[dict[str, Any]]:
        """Latched proximity targets with their world face pose, for lights and viewers."""
        return [{"task": task, "region": region,
                 "position_m": tuple(float(v) for v in target.face_world.translation),
                 "orientation_wxyz": tuple(float(v) for v in target.face_world.orientation_wxyz),
                 "latched": bool(target.latched), "colors": dict(target.indicator)}
                for (task, region), target in self._targets.items()]

    def record(self, time_ns: int, events: Sequence[Mapping[str, Any]]
               ) -> tuple[Mapping[str, Any], ...]:
        """Score events produced by another physical owner (e.g. a prop contact world)."""
        self._check_time(time_ns)
        inputs = []
        for event in events:
            item = {"id": event["id"], "type": event["type"], "task": event["task"],
                    "region": event.get("region") or "", "time_ns": event["time_ns"],
                    "data": dict(event["data"])}
            if item["task"] not in self._tasks or item["time_ns"] != time_ns:
                raise ValueError("recorded events must belong to a selected task and this time")
            inputs.append(item)
        try:
            result = self._evaluate(inputs)
        except Exception:
            self._failed = True
            raise
        self._last_time = time_ns
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
            for trackers in (self._targets, self._surfaces, self._turns):
                for (task, region), tracker in trackers.items():
                    events.extend(self._fact_events(
                        task, region, time_ns, tracker.observe(time_ns, world_reference)))
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

    def _fact_events(self, task: str, region: str, time_ns: int, facts: Sequence[Fact]
                     ) -> list[dict[str, Any]]:
        return [{"id": binding["id"], "type": kind, "task": task, "region": region,
                 "time_ns": time_ns, "data": dict(data)}
                for kind, data in facts for binding in self._tasks[task]["events"]
                if binding["type"] == kind and binding["parameters"]["region"] == region]

    def observe_events(self, time_ns: int, events: Sequence[Mapping[str, Any]]
                       ) -> tuple[Mapping[str, Any], ...]:
        """Score events from a physical owner (for example a prop world) at this time.

        Each event is {id, type, task, region, data}; the task must be selected and the
        data plain JSON. The runtime stamps time_ns and copies the data before any hook
        sees it. Declared-event matching is not required: the pack hook owns the contract.
        """
        self._check_time(time_ns)
        built = []
        for event in events:
            if (not isinstance(event, Mapping)
                    or set(event) != {"id", "type", "task", "region", "data"}
                    or any(not isinstance(event[key], str) for key in ("id", "type", "task", "region"))
                    or not event["id"] or not event["type"] or event["task"] not in self._tasks
                    or not isinstance(event["data"], Mapping)):
                raise ValueError("external task events require id, type, task, region and data")
            built.append({**{key: event[key] for key in ("id", "type", "task", "region")},
                          "time_ns": time_ns,
                          "data": json.loads(json.dumps(event["data"], allow_nan=False))})
        try:
            result = self._evaluate(built)
        except Exception:
            self._failed = True
            raise
        self._last_time = time_ns
        return result

    def release_projectile(self, time_ns: int, identifier: int, mechanism_type: str,
                           tip_world: Sequence[float], radius_m: float,
                           length_m: float | None = None) -> tuple[Mapping[str, Any], ...]:
        """Observe a successful physical release; the mechanism owner supplies its tip.

        length_m is the payload's capsule length (default 2 * radius, a sphere); crate walls,
        floors and the pool floor use its oriented vertical extent.
        """
        self._check_time(time_ns)
        if type(identifier) is not int or identifier < 0 or identifier in self._projectiles:
            raise ValueError("projectile identifiers must be unique nonnegative integers")
        if mechanism_type not in ("launcher", "dropper"):
            raise ValueError("unsupported projectile mechanism type")
        tip = vector3(tip_world, "projectile tip")
        if isinstance(radius_m, bool) or not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError("projectile radius must be positive and finite")
        length = 2 * float(radius_m) if length_m is None else length_m
        if isinstance(length, bool) or not math.isfinite(length) or length <= 0:
            raise ValueError("projectile length must be positive and finite")
        events: list[dict[str, Any]] = []
        tasks: set[str] = set()
        for (task, region), panel in self._panels.items():
            bindings = self._tasks[task]["events"]
            if any(b["type"] == "hit" and b["parameters"]["region"] == region
                   and b["parameters"]["projectile_mechanism_type"] == mechanism_type
                   for b in bindings):
                tasks.add(task)
                events.append({"id": "payload_released", "type": "payload_released",
                               "task": task, "region": region, "time_ns": time_ns,
                               "data": {"projectile_id": identifier,
                                        "mechanism_type": mechanism_type,
                                        "release_distance_m": panel.release_distance(tip)}})
        for task, region in self._crates:
            if task not in tasks and any(
                    b["type"] == "payload_landing"
                    and mechanism_type in b["parameters"]["projectile_mechanism_types"]
                    and (task, b["parameters"]["region"]) in self._crates
                    for b in self._tasks[task]["events"]):
                tasks.add(task)
                events.append({"id": "payload_released", "type": "payload_released",
                               "task": task, "region": region, "time_ns": time_ns,
                               "data": {"projectile_id": identifier,
                                        "mechanism_type": mechanism_type}})
        try:
            result = self._evaluate(events)
        except Exception:
            self._failed = True
            raise
        self._projectiles[identifier] = _Projectile(mechanism_type, float(radius_m),
                                                    float(length), time_ns, tasks)
        self._last_time = time_ns
        return result

    def observe_projectile(self, time_ns: int, identifier: int,
                           start_center_world: Sequence[float], end_center_world: Sequence[float],
                           axis_world: Sequence[float]) -> tuple[Mapping[str, Any], ...]:
        """Events-only view of step_projectile with unknown velocity (wall damping skipped)."""
        return self.step_projectile(time_ns, identifier, start_center_world, end_center_world,
                                    axis_world).events

    def step_projectile(self, time_ns: int, identifier: int,
                        start_center_world: Sequence[float], end_center_world: Sequence[float],
                        axis_world: Sequence[float], velocity_world: Sequence[float] | None = None
                        ) -> ProjectileStep:
        """Judge swept mesh-center motion and return the physical correction to apply.

        The original panel contact uses mesh centers; only release-distance scoring uses
        the projectile tip. Keeping these distinct preserves near-panel outcomes. Order follows
        the original: panels, crates (walls, then landings), pool floor, pool walls/timeout.
        velocity_world is the pre-contact velocity that crate walls damp and project.
        """
        self._check_time(time_ns)
        if type(identifier) is not int or identifier < 0 or identifier not in self._projectiles:
            raise ValueError("projectile must be released before observing its motion")
        start = vector3(start_center_world, "projectile start center")
        end = vector3(end_center_world, "projectile end center")
        axis = vector3(axis_world, "projectile axis")
        if abs(float(np.linalg.norm(axis)) - 1) > 1e-6:
            raise ValueError("projectile axis must be unit length")
        velocity = (np.zeros(3) if velocity_world is None
                    else vector3(velocity_world, "projectile velocity"))
        state = self._projectiles[identifier]
        if not state.active:
            self._last_time = time_ns
            return ProjectileStep((), True)
        mechanism, radius = state.mechanism, state.radius
        events: list[dict[str, Any]] = []
        new, current, stop = end.copy(), velocity.copy(), False
        for (task, region), panel in self._panels.items():
            hit = panel.intersect(start, end, axis, radius)
            if hit is None:
                continue
            state.scored = True
            if hit.outcome == "blocked":
                new, current, stop = panel.world_point(hit.point_local), np.zeros(3), True
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
        for (task, region), crate in self._crates.items():
            step = crate.step(start, new, current, axis, radius, state.length,
                              (task, region) in state.entered)
            new, current = step.position, step.velocity
            if step.entered:
                state.entered.add((task, region))
            if step.outcome is None:
                continue
            stop = True
            if state.scored:
                continue
            state.scored = True
            for binding in self._tasks[task]["events"]:
                p = binding["parameters"]
                if (binding["type"] == "payload_landing" and p["region"] == region
                        and mechanism in p["projectile_mechanism_types"]
                        and p["outcome"] == step.outcome):
                    events.append({"id": binding["id"], "type": "payload_landing", "task": task,
                                   "region": region, "time_ns": time_ns,
                                   "data": {"projectile_id": identifier,
                                            "mechanism_type": mechanism, "outcome": step.outcome,
                                            "detail": step.detail,
                                            "region_class": crate.crate_class}})
        vertical = radius + max(0.0, state.length / 2 - radius) * abs(float(axis[2]))
        reason = ""
        floor_limit = self._environment["floor_z_m"] + vertical
        if new[2] < floor_limit:
            new = new.copy()
            new[2] = floor_limit
            current, stop, reason = np.zeros(3), True, "pool_floor"
        local = self._pool_from_world.apply(new)
        if (local[0] < 0 or local[0] > self._pool_size[0] or local[1] < 0
                or local[1] > self._pool_size[1]
                or (time_ns - state.released_ns) / 1e9 > _MAX_PAYLOAD_AGE_S):
            current, stop, reason = np.zeros(3), True, reason or "pool_wall_or_timeout"
        if reason and not state.scored:
            state.scored = True
            events.extend({"id": "payload_miss", "type": "miss", "task": task, "region": "",
                           "time_ns": time_ns,
                           "data": {"projectile_id": identifier, "mechanism_type": mechanism,
                                    "reason": reason}} for task in sorted(state.tasks))
        try:
            result = self._evaluate(events)
        except Exception:
            self._failed = True
            raise
        state.active = not stop
        self._last_time = time_ns
        moved = None if np.array_equal(new, end) else tuple(float(x) for x in new)
        damped = None if np.array_equal(current, velocity) else tuple(float(x) for x in current)
        return ProjectileStep(result, stop, moved, damped)

    def _evaluate(self, events: list[dict[str, Any]]) -> tuple[Mapping[str, Any], ...]:
        if not events:
            return ()
        for tracker in self._turns.values():
            if any((event["task"], event["id"]) in tracker.restart_events for event in events):
                tracker.restart()
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
