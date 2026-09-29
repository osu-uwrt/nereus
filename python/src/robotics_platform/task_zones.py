"""Generic task geometry observers: open crates, proximity latches, surfacing and turn zones.

Every class is a pure sampled geometry judge. It owns no clock, ledger or competition
rule; each emits ``(kind, data)`` facts that the task runtime binds to declared events and
the pack scoring hook interprets. Times are nonnegative integer nanoseconds and never
decrease until ``reset``; dwell counters use exact integer nanoseconds.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

import numpy as np

from . import _native as native
from .task_projectiles import vector3
from .task_regions import _number, _owned_pose, _rotation

Fact = tuple[str, dict[str, Any]]


def _crossing(start: np.ndarray, end: np.ndarray, coordinate: int, plane: float
              ) -> np.ndarray | None:
    """Segment/plane crossing that ignores repeat hits from a start exactly on the plane."""
    a, b = start[coordinate] - plane, end[coordinate] - plane
    if a * b > 0 or a == b or a == 0:
        return None
    result: np.ndarray = start + (end - start) * (-a / (b - a))
    return result


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class CrateStep:
    """One swept payload step against a crate: corrected motion and an optional landing."""

    position: np.ndarray
    velocity: np.ndarray
    entered: bool
    outcome: str | None = None   # inside | blocked | None (still flying)
    detail: str = ""             # floor | rim for a blocked landing


class OpenCrate:
    """Open-topped crate: conservative wall proxies, top entry, and floor or rim landings.

    Local z = 0 is the interior floor; the port of the original payload-crate logic applies
    to every payload kind. Walls are solid from outside until the payload has entered through
    the top opening, after which the inner liner faces contain it.
    """

    def __init__(self, parameters: Mapping[str, Any], world_from_crate: native.Pose):
        self.crate_class = str(parameters["class"])
        self._outer = _number(parameters["outer_width_m"], "outer_width_m", positive=True) / 2
        inner = _number(parameters["inner_width_m"], "inner_width_m", positive=True)
        liner = _number(parameters["liner_thickness_m"], "liner_thickness_m")
        self._inner = inner / 2 - liner
        self._height = (_number(parameters["outer_height_m"], "outer_height_m", positive=True)
                        - _number(parameters["base_thickness_m"], "base_thickness_m"))
        if self._inner <= 0 or self._inner > self._outer or self._height <= 0:
            raise ValueError("invalid crate geometry")
        pose = _owned_pose(world_from_crate)
        self._rotation = _rotation(pose)
        self._origin = np.asarray(pose.translation, dtype=float)

    def step(self, old_world: np.ndarray, new_world: np.ndarray, velocity_world: np.ndarray,
             axis_world: np.ndarray, radius_m: float, length_m: float, entered: bool
             ) -> CrateStep:
        rotation, origin = self._rotation, self._origin
        new = new_world.copy()
        velocity = velocity_world.copy()
        a = rotation.T @ (old_world - origin)
        b = rotation.T @ (new - origin)
        extent = radius_m + max(0.0, length_m / 2 - radius_m) * np.abs(rotation.T @ axis_world)
        inner, outer, height = self._inner, self._outer, self._height
        # Top entry must clear the inner walls with the entire marker.
        top = _crossing(a, b, 2, height + extent[2])
        if top is not None and b[2] < a[2] and np.all(np.abs(top[:2]) + extent[:2] <= inner):
            entered = True
        # Walls are conservative solid contact proxies for the lattice.
        for axis in (0, 1):
            for sign in (-1, 1):
                surface = sign * (inner - extent[axis] if entered else outer + extent[axis])
                hit = _crossing(a, b, axis, surface)
                if (hit is not None and -extent[2] < hit[2] < height + extent[2]
                        and abs(hit[1 - axis]) < outer + extent[1 - axis]):
                    b = hit
                    velocity = velocity * 0.15
                    normal = rotation[:, axis]
                    velocity = velocity - normal * float(np.dot(velocity, normal))
                    new = rotation @ b + origin
        floor = _crossing(a, b, 2, extent[2])
        if floor is not None and b[2] < a[2] and max(abs(floor[0]), abs(floor[1])) <= outer:
            inside = bool(np.all(np.abs(floor[:2]) + extent[:2] <= inner))
            return CrateStep(rotation @ floor + origin, np.zeros(3), entered,
                             "inside" if entered and inside else "blocked", "floor")
        if (top is not None and b[2] < a[2]
                and not np.all(np.abs(top[:2]) + extent[:2] <= inner)
                and np.all(np.abs(top[:2]) <= outer + extent[:2])):
            return CrateStep(rotation @ top + origin, np.zeros(3), entered, "blocked", "rim")
        return CrateStep(new, velocity, entered)


class ProximityTarget:
    """Latched proximity sensor: a robot probe point within range for a continuous dwell.

    The sensor point is fixed in the task; dwell restarts whenever the probe leaves range.
    Once latched it stays latched until reset. The first observation contributes no time.
    """

    def __init__(self, parameters: Mapping[str, Any], world_from_frame: native.Pose,
                 probe_reference_m: Any):
        self._distance = _number(parameters["trigger_distance_m"], "trigger_distance_m",
                                 positive=True)
        self._dwell_ns = _number(parameters["dwell_s"], "dwell_s", positive=True) * 1e9
        face = native.Pose()
        face.orientation_wxyz = list(parameters["face_orientation_wxyz"])
        offset = vector3(parameters["sensor_offset_m"], "sensor_offset_m")
        self._sensor = _owned_pose(world_from_frame).compose(face).apply(offset)
        self._probe = vector3(probe_reference_m, "probe point")
        self.reset()

    def reset(self) -> None:
        self.latched = False
        self._time: int | None = None
        self._dwell = 0

    def observe(self, time_ns: int, world_reference: native.Pose) -> list[Fact]:
        dt = 0 if self._time is None else time_ns - self._time
        if dt < 0:
            raise ValueError("proximity time must not decrease without reset")
        self._time = time_ns
        if self.latched:
            return []
        tip = _owned_pose(world_reference).apply(self._probe)
        if float(np.linalg.norm(tip - self._sensor)) > self._distance:
            self._dwell = 0
            return []
        self._dwell += dt
        if self._dwell + 1e-3 >= self._dwell_ns:
            self.latched = True
            return [("activate", {"distance_m": float(np.linalg.norm(tip - self._sensor))})]
        return []


class SurfaceTracker:
    """Surfacing inside a regular octagon, breaches outside it, and facing-target dwell.

    The octagon is centred on the region frame with sides facing 0, 45, ... degrees. The
    robot counts as submerged once its envelope top is below the water by the margin, and
    breaches when the top later rises above the margin outside the pipe. Surfacing is
    inside, previously submerged, envelope top at or above the water, held for the dwell.
    Facing picks the target with the smallest planar heading error and needs the same target
    within tolerance for the facing dwell while surfaced (the first sighting restarts the
    counter). Facts are edge-triggered: surface_reached/lost, facing_reached/lost, breach.
    A breach latches; nothing is emitted afterwards until reset. A pose jump resets state.
    """

    def __init__(self, parameters: Mapping[str, Any], world_from_frame: native.Pose,
                 envelope_reference_vertices: Any, surface_z: float,
                 target_positions_world: Mapping[str, Any]):
        if parameters["shape"] != "regular_octagon":
            raise ValueError("surface regions currently support only regular_octagon")
        self._apothem = (_number(parameters["apothem_m"], "apothem_m", positive=True)
                         - _number(parameters["pipe_radius_m"], "pipe_radius_m"))
        self._margin = _number(parameters["breach_margin_m"], "breach_margin_m")
        self._dwell_ns = _number(parameters["dwell_s"], "dwell_s", positive=True) * 1e9
        self._max_step = _number(parameters["max_pose_step_m"], "max_pose_step_m", positive=True)
        facing = parameters["facing"]
        self._tolerance = math.radians(_number(facing["tolerance_deg"], "tolerance_deg"))
        self._facing_ns = _number(facing["dwell_s"], "facing dwell_s", positive=True) * 1e9
        self._targets = {name: np.asarray(target_positions_world[name], dtype=float)
                         for name in facing["targets"]}
        self._surface = _number(surface_z, "surface_z")
        self._vertices = np.array(envelope_reference_vertices, dtype=float, copy=True)
        if self._vertices.ndim != 2 or self._vertices.shape[1] != 3 or len(self._vertices) == 0:
            raise ValueError("surface envelope must be a nonempty N-by-3 array")
        pose = _owned_pose(world_from_frame).inverse()
        self._frame_from_world = pose
        self._frame_rotation = _rotation(pose)
        angles = np.arange(8) * math.pi / 4
        self._normals = np.c_[np.cos(angles), np.sin(angles)]
        self.reset()

    def reset(self) -> None:
        self._time: int | None = None
        self._previous: np.ndarray | None = None
        self._clear()
        self._breached = False

    def _clear(self) -> None:
        self._submerged = False
        self._dwell = 0
        self._facing_dwell = 0.0
        self._facing: str | None = None
        self._surfaced = False
        self._achieved: str | None = None

    def observe(self, time_ns: int, world_reference: native.Pose) -> list[Fact]:
        dt = 0 if self._time is None else time_ns - self._time
        if dt < 0:
            raise ValueError("surface time must not decrease without reset")
        self._time = time_ns
        if self._breached:
            return []
        pose = _owned_pose(world_reference)
        position, rotation = np.asarray(pose.translation, dtype=float), _rotation(pose)
        if self._previous is not None and np.linalg.norm(position - self._previous) > self._max_step:
            self._clear()
        self._previous = position
        facts: list[Fact] = []
        world = self._vertices @ rotation.T + position
        top = float(world[:, 2].max())
        if top < self._surface - self._margin:
            self._submerged = True
        local = (self._vertices @ (self._frame_rotation @ rotation).T
                 + self._frame_from_world.apply(position))
        inside = bool(np.all(local[:, :2] @ self._normals.T <= self._apothem))
        if self._submerged and top > self._surface + self._margin and not inside:
            self._breached = True
            return [("breach", {"envelope_top_world": top})]
        if self._submerged and inside and top >= self._surface:
            self._dwell += dt
            if self._dwell >= self._dwell_ns:
                if not self._surfaced:
                    self._surfaced = True
                    facts.append(("surface_reached", {"envelope_top_world": top}))
                facts.extend(self._facing_step(position, rotation, dt))
            return facts
        self._dwell, self._facing_dwell, self._facing = 0, 0.0, None
        if self._surfaced:
            self._surfaced = False
            facts.append(("surface_lost", {}))
        if self._achieved is not None:
            self._achieved = None
            facts.append(("facing_lost", {}))
        return facts

    def _facing_step(self, position: np.ndarray, rotation: np.ndarray, dt: int) -> list[Fact]:
        heading = rotation[:2, 0]
        candidates = []
        for name, target in self._targets.items():
            direction = target[:2] - position[:2]
            denominator = float(np.linalg.norm(direction) * np.linalg.norm(heading))
            angle = (math.acos(float(np.clip(np.dot(direction, heading) / denominator, -1, 1)))
                     if denominator > 1e-9 else math.pi)
            candidates.append((angle, name))
        angle, facing = min(candidates)
        if angle <= self._tolerance:
            self._facing_dwell = self._facing_dwell + dt if facing == self._facing else 0.0
            self._facing = facing
            achieved = facing if self._facing_dwell >= self._facing_ns else None
        else:
            self._facing_dwell, self._facing = 0.0, None
            achieved = None
        facts: list[Fact] = []
        if achieved != self._achieved:
            if self._achieved is not None:
                facts.append(("facing_lost", {}))
            if achieved is not None:
                facts.append(("facing_reached", {"target": achieved, "angle_rad": angle}))
            self._achieved = achieved
        return facts


class TurnTracker:
    """Signed yaw travel near a zone centre, judged in whole turns.

    While the robot is within radius_m of the zone frame origin the world yaw of the
    reference frame is unwrapped. One-direction travel runs from spin_start to its furthest
    heading; backing off that heading by more than reversal_deg judges the travel and restarts
    from the peak, holding the heading within settle_deg for dwell_s judges it once (the
    count continues), and leaving the zone judges it. Travel under min_travel_deg is not
    judged. ``turns = floor((travel + tolerance) / 2 pi)``. Listed restart events of other
    task state discard the travel so far, as the original did when the basket count changed.
    Facts: rotation_judged {reason: stopped | reversed | left, travel_rad, turns}.
    """

    def __init__(self, parameters: Mapping[str, Any], world_from_frame: native.Pose):
        self._radius = _number(parameters["radius_m"], "radius_m", positive=True)
        self._max_step = _number(parameters["max_pose_step_m"], "max_pose_step_m", positive=True)
        self._tolerance = math.radians(_number(parameters["turn_tolerance_deg"], "turn tolerance"))
        self._settle = math.radians(_number(parameters["settle_deg"], "settle_deg"))
        self._reversal = math.radians(_number(parameters["reversal_deg"], "reversal_deg"))
        self._minimum = math.radians(_number(parameters["min_travel_deg"], "min_travel_deg"))
        self._dwell_ns = _number(parameters["dwell_s"], "dwell_s", positive=True) * 1e9
        self._frame_from_world = _owned_pose(world_from_frame).inverse()
        self.restart_events = tuple((item["task"], item["id"])
                                    for item in parameters["restart_events"])
        self.reset()

    def reset(self) -> None:
        self._time: int | None = None
        self._previous: np.ndarray | None = None
        self._previous_yaw: float | None = None
        self._tracking = False
        self._restart = False
        self._clear()

    def _clear(self) -> None:
        self._yaw = self._settle_yaw = self._start = self._peak = 0.0
        self._dwell = 0
        self._judged = False

    def restart(self) -> None:
        """Discard travel so far; the next in-zone observation resumes from the current yaw."""
        self._restart = self._tracking

    def _judge(self, reason: str) -> list[Fact]:
        travel = abs(self._peak - self._start)
        if travel < self._minimum:
            return []
        turns = int((travel + self._tolerance) // (2 * math.pi))
        return [("rotation_judged", {"reason": reason, "travel_rad": travel, "turns": turns})]

    def observe(self, time_ns: int, world_reference: native.Pose) -> list[Fact]:
        dt = 0 if self._time is None else time_ns - self._time
        if dt < 0:
            raise ValueError("turn time must not decrease without reset")
        self._time = time_ns
        pose = _owned_pose(world_reference)
        position, rotation = np.asarray(pose.translation, dtype=float), _rotation(pose)
        if self._previous is not None and np.linalg.norm(position - self._previous) > self._max_step:
            self._clear()
            self._tracking = self._restart = False
            self._previous_yaw = None
        self._previous = position
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        facts: list[Fact] = []
        near = float(np.linalg.norm(self._frame_from_world.apply(position))) <= self._radius
        if near:
            if not self._tracking or self._restart:
                self._clear()
                self._restart = False
            elif self._previous_yaw is not None:
                self._yaw += _wrap(yaw - self._previous_yaw)
                if abs(self._yaw - self._start) > abs(self._peak - self._start):
                    self._peak = self._yaw
                elif abs(self._peak - self._yaw) > self._reversal:
                    facts.extend(self._judge("reversed"))
                    self._start, self._peak = self._peak, self._yaw
                if abs(self._yaw - self._settle_yaw) <= self._settle:
                    self._dwell += dt
                else:
                    self._settle_yaw, self._dwell = self._yaw, 0
                    self._judged = False
            self._tracking = True
            if self._dwell >= self._dwell_ns and not self._judged:
                self._judged = True
                facts.extend(self._judge("stopped"))
        elif self._tracking:
            facts.extend(self._judge("left"))
            self._clear()
            self._tracking = self._restart = False
        self._previous_yaw = yaw
        return facts
