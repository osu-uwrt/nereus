"""Pack-driven rigid-body prop world: props, table/baskets, pool boundary and claw jaws.

Port of the original ``claw_world.py``. Bullet (PyBullet, CPU, one private ``DIRECT`` client per
instance) simulates the free props against static table/basket meshes and the pool boundary.
The Fossen plant stays authoritative for the robot: the world only kinematically places the two
jaw pads from the robot pose and the mechanism's joint positions. Opposing pad contact held for
``grasp_dwell_s`` creates a finite-force fixed constraint (``hold_force_n``); it is removed when
the jaws open, when the prop moves more than ``slip_distance_m`` from its held pose, or on reset.

Everything comes from resolved pack data: the task definition (frames, static bodies, rigid
bodies, ``box`` regions, events, ``contact_world`` settings), the scenario placement, the pool
pack and the robot claw mechanism (frame, gaps, speed, pads). No robot, prop or task name is
hardcoded. pybullet is an optional dependency imported only when a world is constructed.

Frames: ``robot_body_pose`` is the pose of the robot frame tree root (the physics COM in the Talos
pack) in world; ``linear_velocity_world`` is that origin's world velocity. The claw mount is the
mechanism frame relative to that root.

Claw input is the mechanism's joint positions (``ClawState.joint_positions_m``, mean of both jaws;
0 = closed at ``min_gap_m``). The world infers the commanded direction from their change: rising
means opening (releases a held prop at once), falling means closing (grasp allowed while the
jaws stall against a prop), constant with lagging physical jaws means blocked closing. It reports
the physical jaw position via ``jaw_position_m`` (the mechanism runs ahead when blocked).

Events (task id from the pack, default ``table``); each is a dict with ``task``, ``type``, ``id``,
``region``, ``time_ns`` and ``data``. Ids come from the pack; the table pack uses:

* ``{type: attach, id: grasp, region: None, data: {prop_id, mechanism_id}}``
* ``{type: detach, id: release, region: None, data: {prop_id, mechanism_id, reason}}`` where
  reason is ``released`` (jaws opened), ``slipped`` or ``reset``.
* ``{type: drop_into, id: basket_drop, region: <box region id>, data: {prop_id, basket,
  expected_basket}}`` when a prop rests (speed below ``rest_speed_m_s`` for ``rest_time_s``)
  inside a basket; basket == region; correct iff basket == expected_basket. Once per rest.
* ``{type: drop_into, id: object_dropped, region: "floor" | "table", data: {prop_id}}`` when a
  previously grasped prop rests in contact with the pool floor / anything else outside baskets.

Differences from the original: object drops are judged at rest (the original scored at release);
"miss" (floor rest) is object_dropped/floor; surfacing is not an event (a prop is held and above
the surface in ``props()``); pool walls/floor are the pool pack's boxes, not a plane.
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from . import _native as native
from .packs import ResolvedScenario

Matrix = NDArray[np.float64]

# Contact heuristics of the original claw_world.py (Bullet tuning priors, not pack data).
_PAD_SPINNING_FRICTION = 0.01
_PAD_MARGIN_M = 0.0005
_STALL_NORMAL_SPEED = -0.2  # m/s into a contact that stalls jaw travel
_STALL_PENETRATION_M = -0.0008
_GRASP_NORMAL_FORCE_N = 0.01
_GRASP_NORMAL_ALIGNMENT = 0.6  # only inward-facing pad faces can pinch
_DIRECTION_EPSILON_M = 1e-9


@dataclass(frozen=True)
class Water:
    """Flow relative to which props feel drag, and the fluid density."""

    velocity_world_m_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    density_kg_m3: float = 998.2


@dataclass(frozen=True)
class PropState:
    position_m: tuple[float, float, float]  # mesh origin in world
    orientation_wxyz: tuple[float, float, float, float]
    attached: bool
    mechanism_id: str | None
    basket: str | None  # box region currently containing the prop


def pool_water(resolved: ResolvedScenario, time_s: float) -> Water:
    """Pool current (with its sinusoidal oscillation) and density at ``time_s``."""
    p = resolved.pool["parameters"]
    phase = math.sin(2 * math.pi * p["current_oscillation_frequency_hz"] * time_s)
    velocity = tuple(
        float(c + a * phase)
        for c, a in zip(p["current_m_s"], p["current_oscillation_amplitude_m_s"], strict=True)
    )
    return Water((velocity[0], velocity[1], velocity[2]), float(p["water_density_kg_m3"]))


def _bullet() -> tuple[Any, Any]:
    try:
        import pybullet
        from pybullet_utils.bullet_client import BulletClient
    except ImportError as error:  # pragma: no cover - depends on the environment
        raise ImportError(
            "the prop world needs the optional pybullet package "
            "(pip install 'nereus[props]' or pip install pybullet)"
        ) from error
    return pybullet, BulletClient


def _matrix(position: ArrayLike, wxyz: ArrayLike) -> Matrix:
    w, x, y, z = (float(v) for v in np.asarray(wxyz))
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    t = np.eye(4)
    t[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    t[:3, 3] = position
    return t


def _pose_matrix(pose: native.Pose) -> Matrix:
    return _matrix(pose.translation, pose.orientation_wxyz)


def _xyzw(r: Matrix) -> Matrix:
    """Quaternion (x, y, z, w), stable at 180 degrees: largest component first."""
    trace = np.trace(r)
    candidates = np.array(
        [1 + trace, 1 + 2 * r[0, 0] - trace, 1 + 2 * r[1, 1] - trace, 1 + 2 * r[2, 2] - trace]
    )
    i = int(np.argmax(candidates))
    s = 2 * math.sqrt(max(candidates[i], 0))
    if i == 0:
        q = np.array(
            [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
        )
    else:
        a = i - 1
        b, c = (a + 1) % 3, (a + 2) % 3
        q = np.zeros(4)
        q[a] = s / 4
        q[b] = (r[b, a] + r[a, b]) / s
        q[c] = (r[c, a] + r[a, c]) / s
        q[3] = (r[c, b] - r[b, c]) / s
    return np.asarray(q / np.linalg.norm(q), dtype=float)


def _from_xyzw(position: Sequence[float], q: Sequence[float]) -> Matrix:
    return _matrix(position, (q[3], q[0], q[1], q[2]))


def _finite(values: Any, count: int, name: str) -> Matrix:
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {count} finite values")
    return result


def _verified(root: Path, assets: Mapping[str, Mapping[str, Any]], identifier: str) -> Path:
    item = assets.get(identifier)
    if item is None or item["status"] != "present":
        raise ValueError(f"asset '{identifier}' is not present in its pack")
    path = (root / item["path"]).resolve()
    if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
        raise ValueError(f"asset '{identifier}' does not match its declared sha256")
    return Path(path)


def _root(resolved: ResolvedScenario, key: str) -> Path:
    path = (resolved.path.parent / resolved.scenario[key]).resolve()
    return Path(path if path.is_dir() else path.parent)


class PropWorld:
    """Deterministic contact world for one task with a rigid-body prop set and one claw.

    Calls are serialized by the owner. ``step`` needs no wall time. Not thread safe.
    """

    def __init__(
        self,
        resolved: ResolvedScenario,
        *,
        task: str = "table",
        mechanism: str | None = None,
    ) -> None:
        definitions = {item["id"]: item for item in resolved.task_definitions}
        if task not in definitions:
            raise ValueError(f"unknown task '{task}'")
        self.task = task
        definition = definitions[task]
        placement = {item["task"]: item for item in resolved.scenario["task_placements"]}[task]
        half = math.radians(placement["yaw_deg"]) / 2
        self._world_from_task = _matrix(
            placement["position_m"], (math.cos(half), 0, 0, math.sin(half))
        )
        self._frames: dict[str, Matrix] = {"task": self._world_from_task}
        for frame in definition["frames"]:
            self._frames[frame["id"]] = self._world_from_task @ _matrix(
                frame["position_m"], frame["orientation_wxyz"]
            )

        by_type: dict[str, list[dict[str, Any]]] = {}
        for prop in definition["props"]:
            by_type.setdefault(prop["type"], []).append(prop)
        worlds = by_type.get("contact_world", [])
        if len(worlds) != 1 or not by_type.get("rigid_body"):
            raise ValueError("task needs one contact_world prop and at least one rigid_body")
        self._settings = worlds[0]["parameters"]
        self._statics = by_type.get("static_body", [])
        self._rigid = by_type["rigid_body"]
        self._baskets = [r for r in definition["regions"] if r["type"] == "box"]
        events: dict[str, dict[str, Any]] = {}
        for event in definition["events"]:
            key = event["type"]
            if key == "drop_into":
                key = f"drop_{event['parameters']['outcome']}"
            events.setdefault(key, event)
        try:
            self._events = {
                key: events[key] for key in ("attach", "detach", "drop_in_region", "drop_elsewhere")
            }
        except KeyError as error:
            raise ValueError(f"task lacks an event needed by the prop world: {error}") from error

        task_root = _root(resolved, "tasks")
        robot_root = _root(resolved, "robot")
        self._task_assets = {a["id"]: a for a in resolved.tasks.get("assets", [])}
        self._task_root = task_root
        claws = [
            m
            for m in resolved.robot["mechanisms"]
            if m["type"] == self._settings["claw_mechanism_type"]
            and (mechanism is None or m["id"] == mechanism)
        ]
        if len(claws) != 1:
            raise ValueError("expected exactly one matching claw mechanism")
        claw = claws[0]
        self.mechanism_id = claw["id"]
        self.claw = dict(claw["parameters"])
        for key in (
            "max_gap_m",
            "min_gap_m",
            "jaw_speed_m_s",
            "hold_force_n",
            "friction",
            "contact_margin_m",
            "grasp_dwell_s",
            "slip_distance_m",
        ):
            if not math.isfinite(self.claw[key]) or self.claw[key] <= 0:
                raise ValueError(f"Invalid claw {key}")
        if self.claw["max_gap_m"] <= self.claw["min_gap_m"]:
            raise ValueError("Invalid claw travel")
        self.travel = (self.claw["max_gap_m"] - self.claw["min_gap_m"]) / 2
        self._initial_q = self.travel if self.claw["initial_state"] == "open" else 0.0
        robot_assets = {a["id"]: a for a in resolved.robot.get("assets", [])}
        self._pad_paths = [
            _verified(robot_root, robot_assets, self.claw["pads"][side])
            for side in ("left", "right")
        ]
        edges = []
        for entry in resolved.robot["frames"]["transforms"]:
            edge = native.FixedFrame()
            edge.parent, edge.child = entry["parent"], entry["child"]
            pose = native.Pose()
            pose.translation = entry["position_m"]
            pose.orientation_wxyz = entry["orientation_wxyz"]
            edge.pose = pose
            edges.append(edge)
        frames = native.FixedFrames(resolved.robot["frames"]["root"], edges)
        self._mount = _pose_matrix(frames.from_root(claw["frame"]))

        pool = resolved.pool["parameters"]
        self._surface_z = float(
            pool["water_level_m"] + resolved.scenario["pool_placement"]["position_m"][2]
        )
        self._floor_z = self._surface_z - float(pool["depth_m"])
        yaw = math.radians(resolved.scenario["pool_placement"]["yaw_deg"]) / 2
        pool_from_world = _matrix(
            resolved.scenario["pool_placement"]["position_m"], (math.cos(yaw), 0, 0, math.sin(yaw))
        )
        self._pool_boxes = [
            (
                pool_from_world @ _matrix(b["center_m"], b["orientation_wxyz"]),
                np.asarray(b["size_m"], dtype=float) / 2,
                abs(
                    b["center_m"][2]
                    + b["size_m"][2] / 2
                    - (pool["water_level_m"] - pool["depth_m"])
                )
                < 1e-3,
            )
            for b in resolved.pool["collision_boxes"]
        ]
        self._time_ns: int | None = None
        self._closed = True
        self._build()

    # ------------------------------------------------------------------ construction

    def _obj_vertices(self, identifier: str) -> Matrix:
        path = _verified(self._task_root, self._task_assets, identifier)
        return np.array(
            [
                [float(x) for x in line.split()[1:4]]
                for line in path.read_text().splitlines()
                if line.startswith("v ")
            ]
        )

    def _build(self) -> None:
        pb, client = _bullet()
        self._pb = pb

        class _Headless(client):  # type: ignore[misc,valid-type]
            """Per-instance client without BulletClient's empty CLI options string."""

            def __init__(self) -> None:
                self._shapes: dict[Any, Any] = {}
                self._pid = __import__("os").getpid()
                self._client = -1
                self._client = pb.connect(pb.DIRECT)

        self.b = _Headless()
        self._closed = False
        s = self._settings
        self.b.setGravity(0, 0, -s["gravity_m_s2"])
        self.b.setPhysicsEngineParameter(
            numSolverIterations=s["solver_iterations"], deterministicOverlappingPairs=1
        )
        self.scenery: list[int] = []
        self.floors: list[int] = []
        for transform, half, is_floor in self._pool_boxes:
            shape = self.b.createCollisionShape(pb.GEOM_BOX, halfExtents=half)
            uid = self.b.createMultiBody(
                0,
                shape,
                basePosition=transform[:3, 3],
                baseOrientation=_xyzw(transform[:3, :3]),
            )
            self.scenery.append(uid)
            if is_floor:
                self.floors.append(uid)
        self.statics: dict[str, int] = {}
        for prop in self._statics:
            parameters = prop["parameters"]
            for mesh in parameters.get("collision_meshes", []):
                path = _verified(self._task_root, self._task_assets, mesh["asset"])
                shape = self.b.createCollisionShape(
                    pb.GEOM_MESH, fileName=str(path), flags=pb.GEOM_FORCE_CONCAVE_TRIMESH
                )
                t = self._frames[mesh["frame"]]
                uid = self.b.createMultiBody(
                    0, shape, basePosition=t[:3, 3], baseOrientation=_xyzw(t[:3, :3])
                )
                self.b.changeDynamics(
                    uid,
                    -1,
                    lateralFriction=mesh["lateral_friction"],
                    restitution=mesh["restitution"],
                )
                self.statics[mesh["id"]] = uid
                self.scenery.append(uid)
            for box in parameters["collision_boxes"]:
                t = self._world_from_task @ _matrix(box["center_m"], box["orientation_wxyz"])
                shape = self.b.createCollisionShape(
                    pb.GEOM_BOX, halfExtents=np.asarray(box["size_m"]) / 2
                )
                self.scenery.append(
                    self.b.createMultiBody(
                        0, shape, basePosition=t[:3, 3], baseOrientation=_xyzw(t[:3, :3])
                    )
                )
        self.props: dict[str, dict[str, Any]] = {}
        for prop in self._rigid:
            c = prop["parameters"]
            v = self._obj_vertices(c["collision_asset"])
            center = (v.min(0) + v.max(0)) / 2
            shape = self.b.createCollisionShape(pb.GEOM_MESH, vertices=(v - center).tolist())
            t = self._frames[c["frame"]]
            uid = self.b.createMultiBody(
                c["mass_kg"],
                shape,
                basePosition=t[:3, 3] + t[:3, :3] @ center,
                baseOrientation=_xyzw(t[:3, :3]),
            )
            self.b.changeDynamics(
                uid,
                -1,
                lateralFriction=c["lateral_friction"],
                spinningFriction=c["spinning_friction"],
                rollingFriction=c["rolling_friction"],
                restitution=c["restitution"],
                linearDamping=c["linear_damping"],
                angularDamping=c["angular_damping"],
                collisionMargin=c["collision_margin_m"],
                ccdSweptSphereRadius=c["ccd_swept_sphere_radius_m"],
                contactProcessingThreshold=c["contact_processing_threshold_m"],
            )
            self.props[prop["id"]] = dict(
                id=uid,
                center=center,
                half=(v.max(0) - v.min(0)) / 2,
                config=c,
                settled=0.0,
                scored=False,
                picked=False,
            )
        # Position-held rack drives: impacts cannot back-drive the jaws; the prop solver sees two
        # kinematic, commanded contact surfaces (vehicle contacts belong to the plant).
        self.claw_body = self.b.createMultiBody(0)
        self.pads: list[int] = []
        for path in self._pad_paths:
            shape = self.b.createCollisionShape(pb.GEOM_MESH, fileName=str(path))
            uid = self.b.createMultiBody(0, shape, basePosition=[0, 0, 10])
            self.b.changeDynamics(
                uid,
                -1,
                lateralFriction=self.claw["friction"],
                spinningFriction=_PAD_SPINNING_FRICTION,
                restitution=0,
                collisionMargin=_PAD_MARGIN_M,
            )
            self.pads.append(uid)
        self._basket = [
            (r["id"], np.linalg.inv(self._frames[r["parameters"]["frame"]]), r["parameters"])
            for r in self._baskets
        ]
        self._reset_state()

    def _reset_state(self) -> None:
        self.held: str | None = None
        self.constraint: int | None = None
        self.held_relative: Matrix = np.eye(4)
        self.q = self._initial_q
        self.direction = 0
        self._lockout = False
        self._last_command: float = self._initial_q
        self.contact_time: dict[str, float] = {}
        self._pending: list[dict[str, Any]] = []
        self._time_ns = None
        self._place_pads(np.eye(4), self.q, np.zeros(3), np.zeros(3), 0.0)
        for prop in self.props.values():
            self._restore(prop)

    def _restore(self, prop: dict[str, Any]) -> None:
        c = prop["config"]
        t = self._frames[c["frame"]]
        self.b.resetBasePositionAndOrientation(
            prop["id"], t[:3, 3] + t[:3, :3] @ prop["center"], _xyzw(t[:3, :3])
        )
        self.b.resetBaseVelocity(prop["id"], [0, 0, 0], [0, 0, 0])
        prop.update(settled=0.0, scored=False, picked=False)

    def close(self) -> None:
        """Disconnect this instance's Bullet client (idempotent)."""
        if not self._closed:
            self.b.disconnect()
            self._closed = True

    def __enter__(self) -> PropWorld:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def reset(self) -> None:
        """Recreate the contact world (clears broadphase caches, constraints and poses)."""
        self.close()
        self._build()

    # ------------------------------------------------------------------ queries

    @property
    def jaw_position_m(self) -> float:
        """Physical per-jaw opening (0 = closed); trails the mechanism while blocked/held."""
        return self.q

    @property
    def held_prop(self) -> str | None:
        return self.held

    def _pose(self, key: str) -> Matrix:
        p = self.props[key]
        position, q = self.b.getBasePositionAndOrientation(p["id"])
        t = _from_xyzw(position, q)
        t[:3, 3] -= t[:3, :3] @ p["center"]
        return t

    def props_state(self) -> Mapping[str, PropState]:
        """Read-only prop poses (mesh origin, world frame) and attachment for rendering."""
        result = {}
        for key in self.props:
            t = self._pose(key)
            w = _xyzw(t[:3, :3])
            result[key] = PropState(
                (float(t[0, 3]), float(t[1, 3]), float(t[2, 3])),
                (float(w[3]), float(w[0]), float(w[1]), float(w[2])),
                key == self.held,
                self.mechanism_id if key == self.held else None,
                self._destination(key),
            )
        return MappingProxyType(result)

    def basket_contents(self) -> Mapping[str, str]:
        """Props at rest (settled > rest_time) inside a basket -> basket region id."""
        return MappingProxyType(
            {
                key: basket
                for key, p in self.props.items()
                if p["settled"] > self._settings["rest_time_s"]
                and (basket := self._destination(key)) is not None
            }
        )

    def _destination(self, key: str, _resting: tuple[str, ...] = ()) -> str | None:
        p = self.props[key]
        if key == self.held:
            return None
        position, q = self.b.getBasePositionAndOrientation(p["id"])
        rotation = _from_xyzw((0, 0, 0), q)[:3, :3]
        for identifier, region_from_world, region in self._basket:
            local_t = region_from_world @ np.r_[position, 1]
            local = local_t[:3]
            extent = np.abs(region_from_world[:3, :3] @ rotation) @ p["half"]
            low, high = region["z_range_m"]
            if (
                np.all(np.abs(local[:2]) + extent[:2] < region["half_extents_xy_m"])
                and low < local[2] < high
            ):
                if self.b.getContactPoints(p["id"], self.statics[region["support_mesh"]]):
                    return str(identifier)
                # Stacked: resting on another prop that is itself in this basket.
                resting = (*_resting, key)
                for other, o in self.props.items():
                    if (
                        other not in resting
                        and self.b.getContactPoints(p["id"], o["id"])
                        and self._destination(other, resting) == identifier
                    ):
                        return str(identifier)
        return None

    # ------------------------------------------------------------------ dynamics

    def _place_pads(
        self, mount: Matrix, q: float, velocity: Matrix, angular: Matrix, dq: float
    ) -> None:
        for uid, sign in zip(self.pads, (1, -1), strict=True):
            offset = mount[:3, 1] * (sign * q)
            self.b.resetBasePositionAndOrientation(uid, mount[:3, 3] + offset, _xyzw(mount[:3, :3]))
            self.b.resetBaseVelocity(
                uid, velocity + np.cross(angular, offset) + mount[:3, 1] * (sign * dq), angular
            )

    def _drive_jaws(
        self, dt: float, mount: Matrix, velocity: Matrix, angular: Matrix, target: float
    ) -> None:
        old = self.q
        step = self.claw["jaw_speed_m_s"] * dt
        proposed = float(np.clip(target, old - step, old + step))
        direction = np.sign(proposed - old)
        self._place_pads(mount, proposed, velocity, angular, 0.0)
        if direction:
            # Stall commanded travel against solid contacts; scenery never moves the jaws.
            for uid, sign in zip(self.pads, (1, -1), strict=True):
                axis = mount[:3, 1] * sign
                for other in self.scenery + [p["id"] for p in self.props.values()]:
                    for c in self.b.getClosestPoints(uid, other, 0):
                        normal_speed = np.dot(c[7], axis) * direction
                        if normal_speed < _STALL_NORMAL_SPEED and c[8] < _STALL_PENETRATION_M:
                            proposed -= direction * min(
                                abs(proposed - old),
                                (_STALL_PENETRATION_M - c[8]) / (-normal_speed),
                            )
            self.q = proposed
        self._place_pads(mount, self.q, velocity, angular, (self.q - old) / dt)

    def _event(
        self, key: str, time_ns: int, region: str | None, data: dict[str, Any]
    ) -> dict[str, Any]:
        event = self._events[key]
        emits = event["parameters"].get("emits")
        if emits is not None:
            data = {k: v for k, v in data.items() if k in emits}
        return {
            "task": self.task,
            "type": event["type"],
            "id": event["id"],
            "region": region,
            "time_ns": time_ns,
            "data": data,
        }

    def _release(self, reason: str, time_ns: int) -> None:
        if self.constraint is not None:
            self.b.removeConstraint(self.constraint)
            self.constraint = None
        if self.held is not None:
            for uid in self.pads:
                self.b.setCollisionFilterPair(uid, self.props[self.held]["id"], -1, -1, 1)
            self._pending.append(
                self._event(
                    "detach",
                    time_ns,
                    None,
                    {"prop_id": self.held, "mechanism_id": self.mechanism_id, "reason": reason},
                )
            )
        self.held = None

    def _infer_direction(self, command: float, enabled: bool) -> None:
        delta = command - self._last_command
        self._last_command = command
        if not enabled:
            self.direction = 0
        elif delta > _DIRECTION_EPSILON_M:
            self.direction, self._lockout = 1, False
        elif delta < -_DIRECTION_EPSILON_M:
            self.direction = 0 if self._lockout else -1
        elif not (
            self.direction == -1 and self.q > command + _DIRECTION_EPSILON_M
        ):  # blocked closing continues; anything else is a stop
            self.direction = 0

    def step(
        self,
        dt_s: float,
        time_ns: int,
        robot_body_pose: native.Pose,
        linear_velocity_world: Sequence[float],
        angular_velocity_world: Sequence[float],
        claw_joint_positions: Sequence[float],
        water: Water,
        *,
        enabled: bool = True,
    ) -> list[dict[str, Any]]:
        """Advance ``dt_s`` seconds and return the events produced (ordered)."""
        if self._closed:
            raise RuntimeError("prop world is closed")
        if (
            type(time_ns) is not int
            or time_ns < 0
            or (self._time_ns is not None and time_ns < self._time_ns)
        ):
            raise ValueError("time_ns must be a nondecreasing nonnegative int")
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError("dt_s must be positive and finite")
        velocity = _finite(linear_velocity_world, 3, "linear_velocity_world")
        angular = _finite(angular_velocity_world, 3, "angular_velocity_world")
        joints = _finite(claw_joint_positions, 2, "claw_joint_positions")
        flow = _finite(water.velocity_world_m_s, 3, "water velocity")
        if not math.isfinite(water.density_kg_m3) or water.density_kg_m3 <= 0:
            raise ValueError("water density must be positive and finite")
        self._time_ns = time_ns
        body = _pose_matrix(robot_body_pose)
        mount = body @ self._mount
        mount_velocity = velocity + np.cross(angular, mount[:3, 3] - body[:3, 3])
        command = float(np.clip(joints.mean(), 0.0, self.travel))

        self.b.setTimeStep(dt_s)
        self._infer_direction(command, enabled)
        if self.direction > 0:
            self._release("released", time_ns)
        # Original command semantics: open -> travel, close -> 0, stop -> hold the current gap.
        target = {1: self.travel, -1: 0.0, 0: self.q}[self.direction]
        self.b.resetBasePositionAndOrientation(self.claw_body, mount[:3, 3], _xyzw(mount[:3, :3]))
        self.b.resetBaseVelocity(self.claw_body, mount_velocity, angular)
        self._drive_jaws(dt_s, mount, mount_velocity, angular, target)
        settings = self._settings
        for p in self.props.values():
            xyz, q = self.b.getBasePositionAndOrientation(p["id"])
            r = _from_xyzw((0, 0, 0), q)[:3, :3]
            vel, omega = self.b.getBaseVelocity(p["id"])
            rel = np.asarray(vel) - flow
            half_z = (np.abs(r) @ p["half"])[2]
            wet = float(np.clip((half_z - (xyz[2] - self._surface_z)) / (2 * half_z), 0, 1))
            local = r.T @ rel
            size = 2 * p["half"]
            area = np.array([size[1] * size[2], size[0] * size[2], size[0] * size[1]])
            force = (
                -settings["drag_coefficient"]
                * water.density_kg_m3
                * wet
                * (r @ (area * local * np.abs(local)))
            )
            force[2] += (
                water.density_kg_m3 * settings["gravity_m_s2"] * p["config"]["volume_m3"] * wet
            )
            self.b.applyExternalForce(p["id"], -1, force, xyz, self._pb.WORLD_FRAME)
            self.b.applyExternalTorque(
                p["id"],
                -1,
                -settings["angular_drag_n_m_s_per_rad"] * wet * np.asarray(omega),
                self._pb.WORLD_FRAME,
            )
        self.b.stepSimulation()
        if self.held is not None:
            position, _ = self.b.getBasePositionAndOrientation(self.props[self.held]["id"])
            expected = (mount @ self.held_relative)[:3, 3]
            if np.linalg.norm(np.asarray(position) - expected) > self.claw["slip_distance_m"]:
                self._release("slipped", time_ns)
        if self.held is None and self.direction < 0 and enabled:
            self._try_grasp(dt_s, mount, time_ns)
        self._settle(dt_s, time_ns)
        events, self._pending = self._pending, []
        return events

    def _try_grasp(self, dt: float, mount: Matrix, time_ns: int) -> None:
        pb = self._pb
        for key, p in self.props.items():
            contacts = [self.b.getContactPoints(uid, p["id"]) for uid in self.pads]
            # Only the inward pad faces can pinch; brushing both pad backs is not a grasp.
            touching = all(
                any(
                    c[8] < self.claw["contact_margin_m"]
                    and c[9] > _GRASP_NORMAL_FORCE_N
                    and np.dot(c[7], mount[:3, 1] * sign) > _GRASP_NORMAL_ALIGNMENT
                    for c in cs
                )
                for sign, cs in zip((1, -1), contacts, strict=True)
            )
            self.contact_time[key] = self.contact_time.get(key, 0) + dt if touching else 0
            if self.contact_time[key] < self.claw["grasp_dwell_s"]:
                continue
            position, q = self.b.getBasePositionAndOrientation(p["id"])
            relative = np.linalg.inv(mount) @ _from_xyzw(position, q)
            self.constraint = self.b.createConstraint(
                self.claw_body,
                -1,
                p["id"],
                -1,
                pb.JOINT_FIXED,
                [0, 0, 0],
                relative[:3, 3],
                [0, 0, 0],
                parentFrameOrientation=_xyzw(relative[:3, :3]),
                childFrameOrientation=[0, 0, 0, 1],
            )
            self.b.changeConstraint(self.constraint, maxForce=self.claw["hold_force_n"])
            # A grasped object and its pads form one assembly; internal contacts must not
            # fight the grasp constraint.
            for uid in self.pads:
                self.b.setCollisionFilterPair(uid, p["id"], -1, -1, 0)
            self.held, self.held_relative = key, relative
            p.update(picked=True, scored=False, settled=0.0)
            self.direction, self._lockout = 0, True
            self._pending.append(
                self._event(
                    "attach", time_ns, None, {"prop_id": key, "mechanism_id": self.mechanism_id}
                )
            )
            break

    def _settle(self, dt: float, time_ns: int) -> None:
        settings = self._settings
        for key, p in self.props.items():
            if key == self.held or p["scored"]:
                continue
            velocity, _ = self.b.getBaseVelocity(p["id"])
            destination = self._destination(key)
            label = destination
            if p["picked"]:
                if any(self.b.getContactPoints(p["id"], f) for f in self.floors):
                    label, destination = "floor", None
                elif destination is None and any(
                    c[2] not in self.pads for c in self.b.getContactPoints(p["id"])
                ):
                    label = "support"
            if label and np.linalg.norm(velocity) < settings["rest_speed_m_s"]:
                p["settled"] += dt
                if p["settled"] > settings["rest_time_s"]:
                    if destination is not None:
                        event = self._event(
                            "drop_in_region",
                            time_ns,
                            destination,
                            {
                                "prop_id": key,
                                "basket": destination,
                                "expected_basket": p["config"]["expected_region"],
                            },
                        )
                    else:
                        surfaces = self._events["drop_elsewhere"]["parameters"]["surfaces"]
                        event = self._event(
                            "drop_elsewhere",
                            time_ns,
                            surfaces["floor" if label == "floor" else "support"],
                            {"prop_id": key},
                        )
                    self._pending.append(event)
                    p["scored"] = True
            else:
                p["settled"] = 0.0
