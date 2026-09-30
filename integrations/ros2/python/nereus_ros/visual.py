"""Generic viewer-facing state endpoints and the named encoders that turn them into messages.

Bridge streams declare ``native: 'state:<name>'`` and ``format: json | marker_array`` plus
``options``; everything here is driven by that data and by pack content (frames, assets,
mechanism types, indicator colors), never by robot, task or year names. Every stream is
validated against the runtime and ROS types when the bridge is constructed.

Native value trees handed to a writer/encoder are plain dictionaries:
``{"sim": {"time": ros_ns}, "json": text}`` for ``format: json`` and
``{"sim": {"time": ros_ns}, "items": [marker item, ...]}`` for ``format: marker_array``.
A marker item is ``{ns, id, type: sphere|mesh, frame_id, position_m, orientation_wxyz, scale,
color, mesh (file:// URI), embedded_materials}`` or ``{delete_all: True}``.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np
from nereus import _native as native

from . import mapping

if TYPE_CHECKING:
    from .core import BridgeCore

FORMATS = ("json", "marker_array")
# Endpoint -> (format it must use, published on a timer at rate_hz, or as an event).
ENDPOINTS: dict[str, tuple[str, str]] = {
    "state:run": ("json", "timed"),
    "state:task_score": ("json", "timed"),
    "state:props": ("marker_array", "timed"),
    "state:indicators": ("marker_array", "timed"),
    "state:payloads": ("marker_array", "timed"),
    "event:tasks.feed": ("json", "event"),
    "event:scenario.description": ("json", "event"),
}
_MARKER = "visualization_msgs/msg/Marker"
_MARKER_BASE = "visualization_msgs/Marker"
_MARKER_TYPES = {"sphere": "SPHERE", "mesh": "MESH_RESOURCE"}


class VisualError(ValueError):
    """A viewer-facing stream declaration cannot be applied to this scenario."""


def check_options(options: Mapping[str, Any], where: str, required: Sequence[str],
                  optional: Sequence[str] = ()) -> None:
    unknown = set(options) - set(required) - set(optional)
    missing = set(required) - set(options)
    if unknown or missing:
        raise VisualError(f"{where}/options: keys must be {sorted(required)} plus optional "
                          f"{sorted(optional)} (unknown {sorted(unknown)}, missing "
                          f"{sorted(missing)})")


def _numbers(value: Any, count: int, where: str) -> tuple[float, ...]:
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(isinstance(item, bool) or not isinstance(item, (int, float))
                   for item in value) or not np.all(np.isfinite(value))):
        raise VisualError(f"{where}: must be {count} finite numbers")
    return tuple(float(item) for item in value)


# ------------------------------------------------------------------ encoders


@dataclass(frozen=True)
class JsonEncoder:
    """std_msgs/String-like message whose ``data`` is the native ``json`` text."""

    cls: Any

    def __call__(self, values: Mapping[str, Any]) -> Any:
        message = self.cls()
        message.data = values["json"]
        return message


@dataclass(frozen=True)
class MarkerArrayEncoder:
    """visualization_msgs/MarkerArray from the native marker item list."""

    cls: Any
    marker: Any
    time: Any

    def __call__(self, values: Mapping[str, Any]) -> Any:
        message = self.cls()
        stamp = int(values["sim"]["time"])
        for item in values["items"]:
            marker = self.marker()
            if item.get("delete_all"):
                marker.action = self.marker.DELETEALL
                message.markers.append(marker)
                continue
            marker.header.frame_id = item["frame_id"]
            marker.header.stamp = self.time(sec=stamp // 1_000_000_000,
                                            nanosec=stamp % 1_000_000_000)
            marker.ns, marker.id = item["ns"], int(item["id"])
            marker.type = getattr(self.marker, _MARKER_TYPES[item["type"]])
            marker.action = self.marker.ADD
            pose = marker.pose
            pose.position.x, pose.position.y, pose.position.z = (
                float(v) for v in item["position_m"])
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = (
                float(v) for v in item["orientation_wxyz"])
            marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in item["scale"])
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                float(v) for v in item["color"])
            if item["type"] == "mesh":
                marker.mesh_resource = item["mesh"]
                marker.mesh_use_embedded_materials = bool(item["embedded_materials"])
            message.markers.append(marker)
        return message


# ------------------------------------------------------------------ marker item sources


def _indicator_items(core: BridgeCore, options: Mapping[str, Any], frame_id: str,
                     where: str) -> Callable[[], list[dict[str, Any]]]:
    check_options(options, where, ["shape", "scale_m", "colors"])
    if options["shape"] != "sphere":
        raise VisualError(f"{where}/options/shape: only 'sphere' is supported")
    scale = _numbers(options["scale_m"], 3, f"{where}/options/scale_m")
    colors = {name: _numbers(value, 4, f"{where}/options/colors/{name}")
              for name, value in options["colors"].items()}
    for item in core.session.indicators():
        missing = set(item["colors"].values()) - colors.keys()
        if missing:
            raise VisualError(f"{where}/options/colors: no color for {sorted(missing)} of "
                              f"indicator {item['region']!r}")

    def items() -> list[dict[str, Any]]:
        result = []
        for index, item in enumerate(core.session.indicators()):
            state = item["colors"]["latched" if item["latched"] else "initial"]
            result.append({"ns": item["region"], "id": index, "type": "sphere",
                           "frame_id": frame_id, "position_m": item["position_m"],
                           "orientation_wxyz": item["orientation_wxyz"], "scale": scale,
                           "color": colors[state]})
        return result
    return items


def _asset_uri(core: BridgeCore, role: str, asset: str, where: str) -> str:
    path = core.asset_paths.get(role, {}).get(asset)
    if path is None:
        raise VisualError(f"{where}: asset {asset!r} is not present in the {role} pack")
    return "file://" + path


def _prop_items(core: BridgeCore, options: Mapping[str, Any], frame_id: str,
                where: str) -> Callable[[], list[dict[str, Any]]]:
    check_options(options, where, ["held_frame_id"], ["scale_m", "color"])
    held_frame = options["held_frame_id"]
    if not isinstance(held_frame, str) or not held_frame:
        raise VisualError(f"{where}/options/held_frame_id: must be a ROS frame name")
    scale = _numbers(options.get("scale_m", [1, 1, 1]), 3, f"{where}/options/scale_m")
    color = _numbers(options.get("color", [0, 0, 0, 1]), 4, f"{where}/options/color")
    uris = {}
    for task in core.resolved.task_definitions:
        for prop in task["props"]:
            asset = prop["parameters"].get("visual_asset") if prop["type"] == "rigid_body" else None
            if asset is not None:
                uris[(task["id"], prop["id"])] = _asset_uri(core, "tasks", asset,
                                                            f"{where}: prop {prop['id']!r}")

    def items() -> list[dict[str, Any]]:
        result = []
        reference = None
        for index, prop in enumerate(core.session.prop_visuals()):
            uri = uris.get((prop["task"], prop["id"]))
            if uri is None:
                continue
            position, wxyz, frame = prop["position_m"], prop["orientation_wxyz"], frame_id
            if prop["held"]:
                if reference is None:
                    reference = core.reference_pose(core.session.last_step.snapshot.body)
                pose = native.Pose()
                pose.translation, pose.orientation_wxyz = np.asarray(position, float), np.asarray(
                    wxyz, float)
                relative = reference.inverse().compose(pose)
                position = tuple(float(v) for v in relative.translation)
                wxyz = tuple(float(v) for v in relative.orientation_wxyz)
                frame = held_frame
            result.append({"ns": prop["id"], "id": index, "type": "mesh", "frame_id": frame,
                           "position_m": position, "orientation_wxyz": wxyz, "scale": scale,
                           "color": color, "mesh": uri, "embedded_materials": True})
        return result
    return items


def _payload_items(core: BridgeCore, options: Mapping[str, Any], frame_id: str,
                   where: str) -> Callable[[], list[dict[str, Any]]]:
    check_options(options, where, ["namespaces", "loaded_suffix", "mesh_asset", "color"],
                  ["delete_all"])
    namespaces = dict(options["namespaces"])
    kinds = {item["type"] for item in core.robot.get("mechanisms", [])
             if item["type"] in ("launcher", "dropper")}
    if not kinds <= namespaces.keys():
        raise VisualError(f"{where}/options/namespaces: no namespace for mechanism types "
                          f"{sorted(kinds - namespaces.keys())}")
    suffix = options["loaded_suffix"]
    if not isinstance(suffix, str):
        raise VisualError(f"{where}/options/loaded_suffix: must be a string")
    color = _numbers(options["color"], 4, f"{where}/options/color")
    uri = _asset_uri(core, "robot", options["mesh_asset"], f"{where}/options/mesh_asset")
    delete_all = bool(options.get("delete_all", False))

    def items() -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = [{"delete_all": True}] if delete_all else []
        for item in core.session.payload_visuals():
            namespace = namespaces[item["mechanism_type"]] + (suffix if item["loaded"] else "")
            result.append({
                "ns": namespace, "id": item["id"], "type": "mesh", "frame_id": frame_id,
                "position_m": item["position_m"], "orientation_wxyz": item["orientation_wxyz"],
                "scale": (item["length_m"], 2 * item["radius_m"], 2 * item["radius_m"]),
                "color": color, "mesh": uri, "embedded_materials": False})
        return result
    return items


_ITEM_SOURCES = {"state:props": _prop_items, "state:indicators": _indicator_items,
                 "state:payloads": _payload_items}


# ------------------------------------------------------------------ json sources


def _dump(value: Any) -> str:
    return json.dumps(value, allow_nan=False)


def _json_values(core: BridgeCore, endpoint: str) -> Callable[[], str]:
    if endpoint == "state:run":
        return lambda: _dump(core.session.run_snapshot())
    if endpoint == "state:task_score":
        return lambda: _dump(core.session.task_counters)
    if endpoint == "event:scenario.description":
        return core.scenario_json
    raise VisualError(f"no JSON source for {endpoint!r}")  # event:tasks.feed has per-event values


def compile_format(core: BridgeCore, stream: Mapping[str, Any], cls: Any, where: str
                   ) -> tuple[Callable[[Any], Any], Callable[[], dict[str, Any]] | None]:
    """Validate one ``format`` stream and return its encoder and native-value producer.

    The producer is None for ``event:tasks.feed`` whose values come from queued events.
    """
    endpoint, encoding = stream["native"], stream["format"]
    known = ENDPOINTS.get(endpoint)
    if known is None or known[0] != encoding:
        raise VisualError(f"{where}: native {endpoint!r} cannot use format {encoding!r}")
    if stream["fields"]:
        raise VisualError(f"{where}: format {encoding!r} streams take no field map")
    options = stream.get("options", {})
    if stream["direction"] != "publish":
        raise VisualError(f"{where}: format streams publish")
    if encoding == "json":
        check_options(options, where, [])
        if stream["frame_id"]:
            raise VisualError(f"{where}: json streams are unstamped (frame_id '')")
        if mapping.ros_field(cls, "data").base != "string":
            raise VisualError(f"{where}: json needs a message with a string 'data' field")
        values = None if endpoint == "event:tasks.feed" else _json_values(core, endpoint)
        if values is None:
            return JsonEncoder(cls), None
        return JsonEncoder(cls), lambda: {"sim": {"time": core.clock_ns()}, "json": values()}
    if stream["frame_id"] != core.world_frame:
        raise VisualError(f"{where}: marker frame_id must be the world frame "
                          f"{core.world_frame!r}")
    if mapping.ros_field(cls, "markers").base != _MARKER_BASE:
        raise VisualError(f"{where}: marker_array needs a message with Marker[] 'markers'")
    marker = mapping.message_class(_MARKER)
    items = _ITEM_SOURCES[endpoint](core, options, stream["frame_id"], where)
    return (MarkerArrayEncoder(cls, marker, mapping.message_class("builtin_interfaces/msg/Time")),
            lambda: {"sim": {"time": core.clock_ns()}, "items": items()})
