"""Swept projectile-center intersections with pack-defined perforated panels."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from . import _native as native


@dataclass(frozen=True)
class PanelHit:
    outcome: str
    point_local: tuple[float, float, float]
    hole_id: str = ''
    hole_class: str = ''
    hole_size: str = ''


def vector3(value: Sequence[float], name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f'{name} must have three finite coordinates')
    return vector


class PerforatedPanel:
    """The original segment-plane and oblique capsule-clearance test, without scoring."""

    def __init__(self, parameters: Mapping[str, Any], world_from_task: native.Pose):
        if parameters['plane']['axis'] != 'x':
            raise ValueError('perforated panels currently require an X plane')
        self._task_from_world = world_from_task.inverse()
        self._rotation = native.Pose()
        self._rotation.orientation_wxyz = self._task_from_world.orientation_wxyz
        self._offset = float(parameters['plane']['offset_m'])
        self._half = float(parameters['half_size_m'])
        clearance = parameters['projectile_clearance']
        if clearance['rule'] != 'radius_over_axis_cosine':
            raise ValueError('unsupported projectile clearance rule')
        self._min_cosine = float(clearance['min_cosine'])
        if (not all(math.isfinite(v) for v in (self._offset, self._half, self._min_cosine))
                or self._half <= 0 or not 0 < self._min_cosine <= 1):
            raise ValueError('invalid panel geometry')
        self._holes = []
        for hole in parameters['holes']:
            uv = np.asarray(hole['uv'], dtype=float)
            radius = float(hole['radius_uv']) * 2 * self._half
            if uv.shape != (2,) or not np.isfinite(uv).all() or not math.isfinite(radius) or radius <= 0:
                raise ValueError('invalid panel hole')
            self._holes.append((hole['id'], hole['class'], hole['size'],
                                (uv - .5) * 2 * self._half, radius))

    def release_distance(self, tip_world: Sequence[float]) -> float:
        return abs(float(self._task_from_world.apply(vector3(tip_world, 'tip'))[0]) - self._offset)

    def intersect(self, start_world: Sequence[float], end_world: Sequence[float],
                  axis_world: Sequence[float], radius_m: float) -> PanelHit | None:
        start = self._task_from_world.apply(vector3(start_world, 'start'))
        end = self._task_from_world.apply(vector3(end_world, 'end'))
        axis = vector3(axis_world, 'axis')
        norm = float(np.linalg.norm(axis))
        if abs(norm - 1) > 1e-6:
            raise ValueError('projectile axis must be unit length')
        if isinstance(radius_m, bool) or not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError('projectile radius must be positive and finite')
        a, b = start[0] - self._offset, end[0] - self._offset
        if a * b > 0 or a == b or a == 0:
            return None
        hit = start + (end - start) * (-a / (b - a))
        if max(abs(hit[1]), abs(hit[2])) > self._half:
            return None
        local_axis = self._rotation.apply(axis)
        clearance = radius_m / max(self._min_cosine, abs(float(local_axis[0])))
        point = tuple(float(v) for v in hit)
        for identifier, category, size, center, radius in self._holes:
            if np.linalg.norm(hit[1:] - center) + clearance <= radius:
                return PanelHit('pass', point, identifier, category, size)
        return PanelHit('blocked', point)
