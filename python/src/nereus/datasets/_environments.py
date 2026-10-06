"""Appearance environments: base randomization + named entries + an optional parameter sweep.

The planner merges everything; the renderer only picks an environment per sample. Water values
are relative to the pool pack (``*_scale``) unless an absolute key is given; within one
environment the highest layer that sets either form of a quantity wins.
"""

from __future__ import annotations

import copy
import fnmatch
import itertools
from typing import Any

# Parameter groups of an environment (plus the top-level ``time_s`` range)
GROUPS = ("water", "lighting", "image")
# absolute key -> its pool-relative scale key
EXCLUSIVE = {
    "water": {
        "tint_rgb": "tint_scale",
        "absorption_per_m_rgb": "absorption_scale",
        "scattering": "scattering_scale",
    }
}
# Base randomization ranges ([low, high]) under every spec
DEFAULTS: dict[str, Any] = {
    # Scales of the calibrated pool optics (robosub_2026: scattering 0.458), never absolute.
    "water": {
        "tint_scale": [0.85, 1.15],
        "absorption_scale": [0.7, 1.4],
        "scattering_scale": [0.8, 1.25],
    },
    "lighting": {
        "caustics": [0.0, 1.3],
        "exposure": [0.75, 1.25],
        "direct_light_scale": [0.8, 1.2],
        "ambient_light_scale": [0.8, 1.2],
        "sun_azimuth_deg": [0, 360],
        "sun_elevation_deg": [35, 80],
    },
    "image": {"noise_sigma": [0, 4], "blur_px": [0, 0.8]},
    "time_s": [0, 600],
}
# How the renderer picks an environment per sample: by weight, or cycling evenly
MODES = ("weighted", "sweep")


def _apply(environment: dict[str, Any], layer: dict[str, Any]) -> None:
    """Lay one layer over an environment; an absolute key and its scale replace each other."""
    for group in GROUPS:
        if group not in layer:
            continue
        values = layer[group]
        target = environment.setdefault(group, {})

        # Setting one form of an exclusive quantity drops the other from lower layers
        for absolute, scale in EXCLUSIVE.get(group, {}).items():
            if absolute in values and scale not in values:
                target.pop(scale, None)
            if scale in values and absolute not in values:
                target.pop(absolute, None)
        target.update(copy.deepcopy(values))
    if "time_s" in layer:
        environment["time_s"] = copy.deepcopy(layer["time_s"])


def _sweep_layer(key: str, value: Any) -> dict[str, Any]:
    """A sweep key ("group.name" or "time_s") and value as a layer for ``_apply``."""
    if key == "time_s":
        return {"time_s": value}
    group, _, name = key.partition(".")
    return {group: {name: value}}


def _format(value: Any) -> str:
    """Compact text of a sweep value for environment ids (lists as "[a;b]")."""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, (int, float)):
        return f"{value:g}"
    if isinstance(value, list):
        return "[" + ";".join(_format(item) for item in value) + "]"
    return str(value)


def base(randomize: dict[str, Any]) -> dict[str, Any]:
    """Defaults with the spec's top-level water / lighting / image / time_s over them."""
    environment: dict[str, Any] = {}
    _apply(environment, DEFAULTS)
    _apply(environment, randomize)
    return environment


def expand(randomize: dict[str, Any]) -> tuple[list[dict[str, Any]], str]:
    """Every environment, fully merged, in list order x sweep order; and the selection mode."""
    common = base(randomize)
    spec = randomize.get("environments")
    if not spec:
        return [_entry("default", 1.0, common)], "weighted"

    # Cartesian product of the sweep values; one empty point when there is no sweep
    entries = spec.get("list") or [{"id": "base"}]
    sweep = spec.get("sweep", {})
    points = list(
        itertools.product(*([(key, value) for value in values] for key, values in sweep.items()))
    )

    # Each entry (over the base) x each sweep point; the entry's weight is split over its points
    result = []
    for entry in entries:
        weight = float(entry.get("weight", 1.0)) / len(points)
        for point in points:
            environment = copy.deepcopy(common)
            _apply(environment, entry)
            for key, value in point:
                _apply(environment, _sweep_layer(key, value))
            suffix = ",".join(f"{key}={_format(value)}" for key, value in point)
            name = f"{entry['id']}/{suffix}" if suffix else entry["id"]
            result.append(_entry(name, weight, environment))
    return result, spec.get("mode", "weighted")


def _entry(name: str, weight: float, environment: dict[str, Any]) -> dict[str, Any]:
    """One environment as written to the job: id, weight and every group."""
    return {
        "id": name,
        "weight": weight,
        "water": environment.get("water", {}),
        "lighting": environment.get("lighting", {}),
        "image": environment.get("image", {}),
        "time_s": environment.get("time_s", DEFAULTS["time_s"]),
    }


def select(environments: list[dict[str, Any]], patterns: list[str]) -> list[dict[str, Any]]:
    """Environments whose id matches any glob."""
    return [
        item
        for item in environments
        if any(fnmatch.fnmatchcase(item["id"], pattern) for pattern in patterns)
    ]
