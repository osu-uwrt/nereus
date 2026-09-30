"""Explicitly sampled task geometry; no clock, scoring, transport or simulation ownership."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from . import _native as native


@dataclass(frozen=True)
class PortalEvent:
    kind: str
    time_ns: int
    from_side: str
    to_side: str
    crossing_point_local: tuple[float, float, float] | None
    envelope_top_world: float
    rotation_vector_body: tuple[float, float, float]
    attempt_id: int
    depth_overlap: bool | None = None


@dataclass
class _Attempt:
    identifier: int
    turns: np.ndarray
    passages: dict[str, PortalEvent]


@dataclass(frozen=True)
class _Observation:
    position: np.ndarray
    rotation: np.ndarray
    local_position: np.ndarray
    local_rotation: np.ndarray
    top: float


def _number(value: Any, field: str, *, positive: bool = False) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise ValueError(f"{field} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{field} must be a finite number") from None
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{field} must be {'positive and ' if positive else ''}finite")
    return result


def _keys(value: Any, expected: set[str], field: str, optional: frozenset[str] = frozenset()) -> None:
    if (not isinstance(value, Mapping) or not expected <= set(value)
            or set(value) - expected - optional):
        raise ValueError(f"{field} requires exactly {sorted(expected)}"
                         + (f" plus optional {sorted(optional)}" if optional else ""))


def _owned_pose(value: native.Pose) -> native.Pose:
    # Compose validates and copies through the platform spatial implementation.
    result = native.Pose().compose(value)
    q = result.orientation_wxyz
    result.orientation_wxyz = q / np.linalg.norm(q)
    return result


def _rotation(value: native.Pose) -> np.ndarray:
    rotation = native.Pose()
    rotation.orientation_wxyz = value.orientation_wxyz
    return np.column_stack([rotation.apply(axis) for axis in np.eye(3)])


def _rotation_delta(before: np.ndarray, after: np.ndarray) -> np.ndarray:
    """Net successive rotation in body axes, including cancellation on reversal."""
    r = before.T @ after
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2
    magnitude = float(np.linalg.norm(v))
    angle = math.atan2(magnitude, float(np.clip((np.trace(r) - 1) / 2, -1, 1)))
    return v * (angle / magnitude) if magnitude > 1e-9 else v


class PortalTracker:
    """Track X-plane passages and motion within a spherical approach region.

    Inputs and events are owned values. The envelope is already in the observed reference
    frame; task placement uses native spatial transforms. Bounds are strict with no contact
    tolerance. At a reference-origin crossing, fit uses the completion observation's
    orientation, not an interpolated orientation. Initial side follows the origin even when
    the envelope straddles the plane; subsequent full-envelope sides require every vertex.

    Every completed fit emits pass_through, independently of score eligibility. An approach
    attempt records the last successful passage in each direction; closing it emits those
    attempt_finished events with its net body rotation. A pass outside an attempt has ID 0.
    The approach sphere is centred at (plane.offset_m, 0, 0) in task coordinates.
    Call finish_attempt on an explicit stop; reset silently discards all history.

    Calls must be serialized by the owner. Time is supplied in nonnegative integer
    nanoseconds and must not decrease until reset. Invalid calls leave history unchanged.
    """

    def __init__(self, parameters: Mapping[str, Any], world_from_task: native.Pose,
                 envelope_reference_vertices: Any, floor_z: float) -> None:
        _keys(parameters, {"plane", "bounds_local", "world_floor_clearance",
                           "crossing_reference", "fit_checks", "traversal",
                           "approach_radius_m", "max_pose_step_m"}, "portal parameters",
              frozenset({"frame", "depth_band_m"}))
        plane, bounds = parameters["plane"], parameters["bounds_local"]
        _keys(plane, {"axis", "offset_m"}, "plane")
        _keys(bounds, {"abs_y_lt_m", "z_lt_m"}, "bounds_local")
        if plane["axis"] != "x":
            raise ValueError("portal tracker currently supports only x planes")
        if parameters["crossing_reference"] != "robot_reference_origin":
            raise ValueError("unsupported portal crossing_reference")
        if parameters["traversal"] not in ("full_envelope", "reference_origin"):
            raise ValueError("unsupported portal traversal")
        checks = parameters["fit_checks"]
        supported = {"envelope_at_crossing_point_with_current_orientation",
                     "envelope_at_completion", "reference_origin_at_crossing"}
        if (not isinstance(checks, (list, tuple)) or not checks or
                any(not isinstance(check, str) or check not in supported for check in checks) or
                len(set(checks)) != len(checks)):
            raise ValueError("unsupported or repeated portal fit_checks")
        if not isinstance(parameters["world_floor_clearance"], bool):
            raise ValueError("world_floor_clearance must be boolean")
        self._floor = _number(floor_z, "floor_z")
        self._offset = _number(plane["offset_m"], "plane.offset_m")
        self._width = _number(bounds["abs_y_lt_m"], "bounds_local.abs_y_lt_m", positive=True)
        self._top = _number(bounds["z_lt_m"], "bounds_local.z_lt_m")
        self._radius = _number(parameters["approach_radius_m"], "approach_radius_m", positive=True)
        self._max_step = _number(parameters["max_pose_step_m"], "max_pose_step_m", positive=True)
        self._floor_check = parameters["world_floor_clearance"]
        band = parameters.get("depth_band_m")
        self._band: tuple[float, float] | None = None
        if band is not None:
            if not isinstance(band, (list, tuple)) or len(band) != 2:
                raise ValueError("depth_band_m must be [bottom, top]")
            self._band = (_number(band[0], "depth_band_m[0]"), _number(band[1], "depth_band_m[1]"))
            if self._band[0] > self._band[1]:
                raise ValueError("depth_band_m must be ordered [bottom, top]")
        self._traversal, self._checks = parameters["traversal"], frozenset(checks)
        self._vertices = np.array(envelope_reference_vertices, dtype=float, copy=True)
        if (self._vertices.ndim != 2 or self._vertices.shape[1] != 3 or
                len(self._vertices) == 0 or not np.isfinite(self._vertices).all()):
            raise ValueError("portal envelope must be a nonempty finite N-by-3 array")
        self._vertices.setflags(write=False)
        self._task_from_world = _owned_pose(world_from_task).inverse()
        self._task_rotation = _rotation(self._task_from_world)
        self.reset()

    def reset(self) -> None:
        self._time_ns: int | None = None
        self._previous: _Observation | None = None
        self._entry_side: int | None = None
        self._crossing: np.ndarray | None = None
        self._attempt: _Attempt | None = None
        self._next_attempt = 1

    def _time(self, time_ns: int) -> None:
        if isinstance(time_ns, bool) or not isinstance(time_ns, int) or time_ns < 0:
            raise ValueError("time_ns must be a nonnegative integer")
        if self._time_ns is not None and time_ns < self._time_ns:
            raise ValueError("portal time must not decrease without reset")

    @staticmethod
    def _finished(attempt: _Attempt | None, time_ns: int, top: float) -> tuple[PortalEvent, ...]:
        if attempt is None:
            return ()
        turns = tuple(float(x) for x in attempt.turns)
        return tuple(replace(event, kind="attempt_finished", time_ns=time_ns,
                             envelope_top_world=top, rotation_vector_body=turns)
                     for event in attempt.passages.values())

    def finish_attempt(self, time_ns: int) -> tuple[PortalEvent, ...]:
        self._time(time_ns)
        events = self._finished(self._attempt, time_ns,
                                self._previous.top if self._previous is not None else 0.0)
        self._attempt = None
        self._time_ns = time_ns
        return events

    def observe(self, time_ns: int, world_reference: native.Pose) -> tuple[PortalEvent, ...]:
        self._time(time_ns)
        pose = _owned_pose(world_reference)
        position, rotation = pose.translation, _rotation(pose)
        local_position = self._task_from_world.apply(position)
        local_rotation = self._task_rotation @ rotation
        # Everything below is computed against local state before committing the observation.
        with np.errstate(over="raise", invalid="raise"):
            try:
                world = self._vertices @ rotation.T + position
                local = self._vertices @ local_rotation.T + local_position
                current = _Observation(position, rotation, local_position, local_rotation,
                                       float(world[:, 2].max()))
                events, entry, crossing, attempt, next_id = self._advance(time_ns, current,
                                                                          world, local)
            except FloatingPointError as error:
                raise ValueError("portal geometry overflow") from error
        self._previous, self._entry_side, self._crossing = current, entry, crossing
        self._attempt, self._next_attempt, self._time_ns = attempt, next_id, time_ns
        return tuple(events)

    def _advance(self, time_ns: int, current: _Observation, world: np.ndarray,
                 local: np.ndarray) -> tuple[list[PortalEvent], int | None, np.ndarray | None,
                                              _Attempt | None, int]:
        previous, entry, crossing = self._previous, self._entry_side, self._crossing
        attempt = (None if self._attempt is None else
                   _Attempt(self._attempt.identifier, self._attempt.turns.copy(),
                            dict(self._attempt.passages)))
        next_id, events = self._next_attempt, []
        if previous is not None and np.linalg.norm(current.position - previous.position) > self._max_step:
            events.extend(self._finished(attempt, time_ns, previous.top))
            previous, entry, crossing, attempt = None, None, None, None
        near = np.linalg.norm(current.local_position - [self._offset, 0, 0]) <= self._radius
        if near and attempt is None:
            attempt = _Attempt(next_id, np.zeros(3), {})
            next_id += 1
        passage = None
        if previous is not None:
            a, b = previous.local_position[0] - self._offset, current.local_position[0] - self._offset
            if (a > 0 >= b) or (a < 0 <= b):
                crossing = previous.local_position + (a / (a - b)) * (
                    current.local_position - previous.local_position)
            x = (local[:, 0] - self._offset if self._traversal == "full_envelope" else
                 np.array([current.local_position[0] - self._offset]))
            side = 1 if x.min() > 0 else -1 if x.max() < 0 else 0
            if side and entry is not None and side != entry:
                if crossing is not None:
                    fits = not self._floor_check or world[:, 2].min() > self._floor
                    for check in self._checks:
                        if check == "reference_origin_at_crossing":
                            # Origin-only fit: the crossing must have a side (y != 0) and stay in width.
                            fits = fits and 0 < abs(float(crossing[1])) < self._width
                            continue
                        vertices = (local if check == "envelope_at_completion" else
                                    self._vertices @ current.local_rotation.T + crossing)
                        fits = (fits and np.max(np.abs(vertices[:, 1])) < self._width and
                                vertices[:, 2].max() < self._top)
                    if fits:
                        depth = None
                        if self._band is not None:
                            # Conservative box extent of the envelope in task axes at completion.
                            extent = np.abs(current.local_rotation) @ np.abs(self._vertices).max(axis=0)
                            depth = bool(crossing[2] + extent[2] >= self._band[0]
                                         and crossing[2] - extent[2] <= self._band[1])
                        passage = (entry, side, tuple(float(x) for x in crossing), depth)
                crossing = None
            if side:
                entry = side
            if near and attempt is not None:
                attempt.turns += _rotation_delta(previous.rotation, current.rotation)
        else:
            entry = 1 if current.local_position[0] > self._offset else -1
        if passage is not None:
            source, target, point, depth = passage
            event = PortalEvent(
                "pass_through", time_ns, "positive" if source > 0 else "negative",
                "positive" if target > 0 else "negative", point, current.top,
                tuple(float(x) for x in attempt.turns) if attempt is not None else (0., 0., 0.),
                attempt.identifier if attempt is not None else 0, depth)
            events.append(event)
            if attempt is not None:
                attempt.passages[event.from_side] = event
        if not near:
            events.extend(self._finished(attempt, time_ns, current.top))
            attempt = None
        return events, entry, crossing, attempt, next_id
