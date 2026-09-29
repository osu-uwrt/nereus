"""Semantic checks JSON Schema cannot express: references, frames, geometry, paths, hooks.

Every function returns problem strings; nothing here imports task code or builds a runtime.
"""

from __future__ import annotations

import fnmatch
import hashlib
import math
import re
from collections import Counter
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import numpy as np

WORLD = "world"  # reserved reporting frame for world-referenced sensor products
TASK = "task"  # reserved name of a task's own local frame
# Native invariants (libraries/spatial frames.cpp, simulation plant.cpp, marine_dynamics.cpp).
QUATERNION_TOLERANCE = 1e-8
DIRECTION_TOLERANCE = 1e-9
SYMMETRY_TOLERANCE = 1e-10
SEMIDEFINITE_TOLERANCE = 1e-10
TRIANGLE_TOLERANCE = 1e-9
INACTIVE_KEYS = frozenset({"pending", "provenance", "metadata", "scoring_hooks"})


def duplicates(values: Iterable[str], what: str, problems: list[str]) -> None:
    for value, count in Counter(values).items():
        if count > 1:
            problems.append(f"duplicate {what} id '{value}'")


def _unit(vector: list[float], where: str, problems: list[str], tolerance: float) -> None:
    norm = math.sqrt(sum(value * value for value in vector))
    if not abs(norm - 1.0) < tolerance:
        problems.append(f"{where}: must have unit norm (got {norm:.12g})")


def _quaternions(node: Any, where: str, problems: list[str]) -> None:
    """Check every active orientation_wxyz; inactive metadata/pending/hook data is skipped."""
    if isinstance(node, dict):
        for key, value in node.items():
            if where == "" and key in INACTIVE_KEYS:
                continue
            if key == "orientation_wxyz":
                _unit(value, f"{where}/{key}", problems, QUATERNION_TOLERANCE)
            else:
                _quaternions(value, f"{where}/{key}", problems)
    elif isinstance(node, list):
        for index, value in enumerate(node):
            _quaternions(value, f"{where}/{index}", problems)


def _symmetric_semidefinite(matrix: Any, where: str, problems: list[str]) -> bool:
    """Eigen isApprox symmetry and eigenvalue floor, as the native plant validates."""
    array = np.asarray(matrix, float)
    difference = float(np.sum((array - array.T) ** 2))
    if difference > SYMMETRY_TOLERANCE**2 * float(np.sum(array**2)):
        problems.append(f"{where}: must be symmetric")
        return False
    if float(np.linalg.eigvalsh(array).min()) < -SEMIDEFINITE_TOLERANCE:
        problems.append(f"{where}: must be positive semidefinite")
        return False
    return True


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def inside(root: Path, relative: str) -> Path | None:
    """Resolve a pack-relative path; None if it escapes the pack (including via symlinks)."""
    base = root.resolve()
    target = (root / relative).resolve()
    return target if target == base or base in target.parents else None


def assets(
    data: dict[str, Any], root: Path, owner: str, present: dict[Path, str] | None = None
) -> tuple[list[str], list[dict[str, Any]]]:
    """Check declared assets; return (problems, unresolved missing assets).

    Verified present assets are recorded in ``present`` with the digest actually checked.
    """
    problems: list[str] = []
    unresolved: list[dict[str, Any]] = []
    declared = data.get("assets", [])
    duplicates((item["id"] for item in declared), "asset", problems)
    for item in declared:
        where = f"/assets/{item['id']}"
        target = inside(root, item["path"])
        if target is None:
            problems.append(f"{where}: path '{item['path']}' escapes the pack")
            continue
        if item["status"] == "missing":
            if target.exists():
                problems.append(f"{where}: marked missing but {item['path']} exists")
            unresolved.append({
                "kind": "asset", "pack": owner, "id": item["id"], "path": item["path"],
                "required_from_step": item["required_from_step"],
            })
        elif not target.is_file():
            problems.append(f"{where}: marked present but {item['path']} is not a file")
        else:
            digest = sha256(target)
            if digest != item["sha256"]:
                problems.append(f"{where}: sha256 mismatch for {item['path']}")
            elif present is not None:
                present[target] = digest
    return problems, unresolved


def pending(data: dict[str, Any], owner: str) -> list[dict[str, Any]]:
    return [
        {"kind": f"pending_{item['category']}", "pack": owner, "id": item["id"],
         "required_from_step": item["required_from_step"], "blocker": item["blocker"]}
        for item in data.get("pending", [])
    ]


# ------------------------------------------------------------------ robot

def robot_frames(robot: dict[str, Any]) -> set[str]:
    frames = robot["frames"]
    return {frames["root"], *(item["child"] for item in frames["transforms"])}


_RIGHT_OUTPUTS = frozenset({"rgb_right", "camera_info_right"})


def _stereo(item: dict[str, Any], parent_of: dict[str, str], transforms: dict[str, Any],
            where: str, problems: list[str]) -> None:
    """Depth range order and a rectified right eye one baseline along the left optical +X."""
    parameters = item["parameters"]
    depth = parameters["depth"]
    if depth["min_range_m"] >= depth["max_range_m"]:
        problems.append(f"{where}/parameters/depth: min_range_m must be < max_range_m")
    right = parameters.get("right_frame")
    if right is None:
        if _RIGHT_OUTPUTS.intersection(parameters["outputs"]):
            problems.append(f"{where}/parameters: right-eye outputs require right_frame")
        return
    if parent_of.get(right) != item["frame"]:
        problems.append(f"{where}/parameters/right_frame: '{right}' must be a child of "
                        f"sensor frame '{item['frame']}'")
        return
    transform = transforms[right]
    offset = np.subtract(transform["position_m"], [parameters["baseline_m"], 0, 0])
    rotation = np.abs(transform["orientation_wxyz"]) - [1, 0, 0, 0]
    if np.abs(offset).max() > DIRECTION_TOLERANCE or np.abs(rotation).max() > QUATERNION_TOLERANCE:
        problems.append(f"{where}/parameters/right_frame: '{right}' must be at [baseline_m, 0, 0] "
                        "with identity orientation")


def robot(data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    frames = data["frames"]
    root = frames["root"]
    children = [item["child"] for item in frames["transforms"]]
    duplicates(children, "frame", problems)
    if root in children:
        problems.append(f"/frames: root '{root}' cannot also be a child")
    parent_of = {item["child"]: item["parent"] for item in frames["transforms"]}
    transforms = {item["child"]: item for item in frames["transforms"]}
    names = robot_frames(data)
    for child, parent in parent_of.items():
        if parent not in names:
            problems.append(f"/frames/{child}: unknown parent '{parent}'")
    for start in parent_of:
        seen, node = set(), start
        while node in parent_of and node not in seen:
            seen.add(node)
            node = parent_of[node]
        if node in seen:
            problems.append(f"/frames/{start}: cycle in frame tree")
            break

    def frame(name: str, where: str, world: bool = False) -> None:
        if name not in names and not (world and name == WORLD):
            problems.append(f"{where}: unknown frame '{name}'")

    frame(data["reference_frame"], "/reference_frame")
    body = data["body"]["parameters"]
    where = "/body/parameters/inertia_matrix"
    if _symmetric_semidefinite(body["inertia_matrix"], where, problems):
        moments = np.linalg.eigvalsh(np.asarray(body["inertia_matrix"], float))
        if moments.min() <= 0 or moments.max() > moments.sum() / 2 + TRIANGLE_TOLERANCE:
            problems.append(f"{where}: must be positive definite and satisfy the "
                            "principal-moment triangle inequalities")
    for key in ("added_mass_matrix", "linear_damping_matrix"):
        _symmetric_semidefinite(body[key], f"/body/parameters/{key}", problems)

    boxes = [item["id"] for item in data["collision_boxes"]]
    duplicates(boxes, "collision box", problems)
    duplicates((item["id"] for item in data["thrusters"]), "thruster", problems)
    for item in data["thrusters"]:
        _unit(item["direction"], f"/thrusters/{item['id']}/direction", problems,
              DIRECTION_TOLERANCE)
        response = item["parameters"]
        if response["forward_limit_n"] <= 0 and response["reverse_limit_n"] <= 0:
            problems.append(f"/thrusters/{item['id']}: cannot produce force")

    duplicates((item["id"] for item in data["sensors"]), "sensor", problems)
    for item in data["sensors"]:
        where = f"/sensors/{item['id']}"
        frame(item["frame"], f"{where}/frame", world=True)
        frame(item["mount_frame"], f"{where}/mount_frame")
        parameters = item["parameters"]
        for index, axis in enumerate(parameters.get("axes", [])):
            if math.hypot(*axis) == 0:
                problems.append(f"{where}/parameters/axes/{index}: zero axis")
        if item["type"] == "stereo_camera":
            _stereo(item, parent_of, transforms, where, problems)

    asset_ids = {item["id"] for item in data.get("assets", [])}
    for index, visual in enumerate(data.get("visuals", [])):
        if visual["asset"] not in asset_ids:
            problems.append(f"/visuals/{index}/asset: unknown asset '{visual['asset']}'")
        frame(visual["frame"], f"/visuals/{index}/frame")
    mechanisms = {item["id"]: item for item in data["mechanisms"]}
    duplicates((item["id"] for item in data["mechanisms"]), "mechanism", problems)
    for identifier, item in mechanisms.items():
        where = f"/mechanisms/{identifier}"
        frame(item["frame"], f"{where}/frame")
        parameters = item["parameters"]
        if item["type"] in ("launcher", "dropper"):
            slots = parameters["slots"]
            duplicates((slot["id"] for slot in slots), f"{identifier} slot", problems)
            if parameters["capacity"] != len(slots):
                problems.append(f"{where}/parameters/capacity: must equal the number of slots")
        if item["type"] == "claw":
            if parameters["min_gap_m"] >= parameters["max_gap_m"]:
                problems.append(f"{where}/parameters: min_gap_m must be < max_gap_m")
            for side, asset in parameters["pads"].items():
                if asset not in asset_ids:
                    problems.append(f"{where}/parameters/pads/{side}: unknown asset '{asset}'")
    for name in data["safety"]["arming"]["applies_to"]:
        if name not in mechanisms:
            problems.append(f"/safety/arming/applies_to: unknown mechanism '{name}'")
    envelope = data["scoring_envelope"]
    for name in envelope["collision_boxes"]:
        if name not in boxes:
            problems.append(f"/scoring_envelope/collision_boxes: unknown box '{name}'")
    duplicates((point["id"] for point in envelope["points"]), "envelope point", problems)
    for point in envelope["points"]:
        frame(point["frame"], f"/scoring_envelope/points/{point['id']}/frame")
    _quaternions(data, "", problems)
    return problems


# ------------------------------------------------------------------ pool

def pool(data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    duplicates((item["id"] for item in data["collision_boxes"]), "collision box", problems)
    _quaternions(data, "", problems)
    return problems


# ------------------------------------------------------------------ tasks

_MODULE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def hook_module(root: Path, module: str) -> Path | None:
    """Pack-local file a hook module name resolves to (not imported), or None if it escapes."""
    parts = module.split(".")
    if not all(_MODULE.match(part) for part in parts):
        return None
    candidate = inside(root, "/".join(parts) + ".py")
    package = inside(root, "/".join(parts) + "/__init__.py")
    if candidate is None or package is None:
        return None
    return package if package.is_file() and not candidate.exists() else candidate


def tasks(data: dict[str, Any], root: Path) -> tuple[list[str], list[dict[str, Any]]]:
    problems: list[str] = []
    unresolved: list[dict[str, Any]] = []
    for relative in data["tasks"]:
        if inside(root, relative) is None:
            problems.append(f"/tasks: include '{relative}' escapes the pack")
    options = data["run_options"]
    duplicates((item["key"] for item in options), "run option", problems)
    for item in options:
        if item["type"] == "choice" and item["default"] not in item["choices"]:
            problems.append(f"/run_options/{item['key']}: default not among choices")
    for index, hook in enumerate(data["scoring_hooks"]):
        target = hook_module(root, hook["module"])
        if target is None:
            problems.append(f"/scoring_hooks/{index}: module '{hook['module']}' escapes the pack")
        elif not target.is_file():
            unresolved.append({
                "kind": "hook_module", "pack": "tasks", "id": f"{hook['module']}:{hook['function']}",
                "path": target.relative_to(root.resolve()).as_posix(), "required_from_step": None,
            })
    return problems, unresolved


_EVENT_REGION = {"pass_through": "rectangular_portal", "hit": "perforated_panel"}


def task(data: dict[str, Any], asset_ids: set[str] | None) -> list[str]:
    """Check one task include; asset references are checked when the owning pack is known."""
    problems: list[str] = []
    frames = {item["id"] for item in data["frames"]}
    duplicates((item["id"] for item in data["frames"]), "frame", problems)
    if TASK in frames:
        problems.append(f"/frames: '{TASK}' is reserved for the task-local frame")
    frames.add(TASK)
    props = {item["id"]: item for item in data["props"]}
    regions = {item["id"]: item for item in data["regions"]}
    events = {item["id"] for item in data["events"]}
    for what, items in (("prop", data["props"]), ("region", data["regions"]),
                        ("event", data["events"]), ("scoring", data["scoring"])):
        duplicates((item["id"] for item in items), what, problems)
    for identifier, item in props.items():
        parameters = item["parameters"]
        duplicates((box["id"] for box in parameters["collision_boxes"]), f"{identifier} box", problems)
        for visual in parameters.get("visuals", []):
            if asset_ids is not None and visual["asset"] not in asset_ids:
                problems.append(f"/props/{identifier}/visuals: unknown asset '{visual['asset']}'")
            if visual["frame"] not in frames:
                problems.append(f"/props/{identifier}/visuals: unknown frame '{visual['frame']}'")
        cutouts = parameters.get("cutouts")
        if cutouts is not None:
            region = regions.get(cutouts["region"])
            if region is None or region["type"] != "perforated_panel":
                problems.append(f"/props/{identifier}/cutouts: region must be a perforated_panel")
    for identifier, item in regions.items():
        if item["type"] == "perforated_panel":
            duplicates((hole["id"] for hole in item["parameters"]["holes"]), "hole", problems)
    for item in data["events"]:
        where = f"/events/{item['id']}"
        parameters = item["parameters"]
        expected = _EVENT_REGION.get(item["type"])
        if expected is not None:
            region = regions.get(parameters["region"])
            if region is None or region["type"] != expected:
                problems.append(f"{where}/parameters/region: must name a {expected} region")
        if item["type"] == "pass_through" and parameters["from_side"] == parameters["to_side"]:
            problems.append(f"{where}/parameters: from_side and to_side must differ")
        if item["type"] == "contact" and parameters["prop"] not in props:
            problems.append(f"{where}/parameters/prop: unknown prop '{parameters['prop']}'")
    for item in data["scoring"]:
        if item["parameters"]["event"] not in events:
            problems.append(f"/scoring/{item['id']}: unknown event '{item['parameters']['event']}'")
    _quaternions(data, "", problems)
    return problems


# ------------------------------------------------------------------ bridge

def bridge(data: dict[str, Any]) -> list[str]:
    """Internal bridge references; robot bindings are checked by ``bridge_binding``."""
    problems: list[str] = []
    streams = {item["id"]: item for item in data["streams"]}
    services = {item["id"] for item in data.get("services", [])}
    duplicates((item["id"] for item in data["streams"]), "stream", problems)
    duplicates((item["id"] for item in data.get("services", [])), "service", problems)
    converters = {item["name"] for item in data.get("converters", [])}
    duplicates((item["name"] for item in data.get("converters", [])), "converter", problems)

    def stream(name: str, where: str, direction: str | None = None) -> None:
        item = streams.get(name)
        if item is None:
            problems.append(f"{where}: unknown stream '{name}'")
        elif direction is not None and item["direction"] != direction:
            problems.append(f"{where}: stream '{name}' must be a {direction} stream")

    for item in [*data["streams"], *data.get("services", [])]:
        if "converter" in item and item["converter"] not in converters:
            problems.append(f"/{item['id']}/converter: unknown converter '{item['converter']}'")
    for item in data["streams"]:
        if "reply_stream" in item:
            stream(item["reply_stream"], f"/streams/{item['id']}/reply_stream", "publish")
    if "kill" in data:
        stream(data["kill"]["command_stream"], "/kill/command_stream", "subscribe")
        stream(data["kill"]["state_stream"], "/kill/state_stream", "publish")
    for key, value in data.get("reset", {}).items():
        if value not in services:
            problems.append(f"/reset/{key}: unknown service '{value}'")
    placement = data.get("placement", {})
    for key in ("set_service", "sync_service"):
        if key in placement and placement[key] not in services:
            problems.append(f"/placement/{key}: unknown service '{placement[key]}'")
    if "estimator_alignment" in placement:
        stream(placement["estimator_alignment"]["estimate_stream"],
               "/placement/estimator_alignment/estimate_stream", "subscribe")
    thrusters = data.get("thrusters")
    if thrusters is not None and len(thrusters["order"]) != len(thrusters["input_scales"]):
        problems.append("/thrusters: order and input_scales must have equal length")
    uses_thrusters = any(item["native"].startswith("command:thrusters") for item in data["streams"])
    if uses_thrusters and thrusters is None:
        problems.append("/streams: thruster command stream requires a thrusters block")
    tf = data.get("tf", {})
    published = [*tf.get("publish", []), *tf.get("static", [])]
    edges = [*published, *tf.get("lookup", [])]
    duplicates((edge["child"] for edge in edges), "TF child/owner", problems)
    parents = {edge["child"]: edge["parent"] for edge in edges}
    for edge in published:
        if any(fnmatch.fnmatchcase(edge["child"], pattern)
               for pattern in tf.get("never_publish", [])):
            problems.append(f"/tf: '{edge['child']}' is listed in never_publish")
    for child in parents:
        seen: set[str] = set()
        current = child
        while current in parents and current not in seen:
            seen.add(current)
            current = parents[current]
        if current in seen:
            problems.append(f"/tf: cycle at '{current}'")
            break
    return problems


def bridge_binding(data: dict[str, Any], robot_data: dict[str, Any]) -> list[str]:
    problems: list[str] = []
    if "requires_robot" in data and data["requires_robot"] != robot_data["id"]:
        problems.append(f"/requires_robot: bridge requires '{data['requires_robot']}', "
                        f"scenario selects '{robot_data['id']}'")
    thrusters = data.get("thrusters")
    robot_thrusters = [item["id"] for item in robot_data["thrusters"]]
    if thrusters is not None and sorted(thrusters["order"]) != sorted(robot_thrusters):
        problems.append("/thrusters/order: must be a permutation of the robot thruster ids")
    frames = robot_frames(robot_data) | {WORLD}
    for name in data.get("frame_names", {}):
        if name not in frames:
            problems.append(f"/frame_names: unknown robot frame '{name}'")
    names = data.get("frame_names", {})
    for index, edge in enumerate(data.get("tf", {}).get("static", [])):
        for native_name, ros_name in (("from_frame", "parent"), ("to_frame", "child")):
            frame = edge[native_name]
            if frame not in frames or frame == WORLD:
                problems.append(f"/tf/static/{index}/{native_name}: unknown robot frame '{frame}'")
            elif frame in names and names[frame] != edge[ros_name]:
                problems.append(f"/tf/static/{index}/{ros_name}: differs from frame_names['{frame}']")
    sensors = {item["id"]: item for item in robot_data["sensors"]}
    mechanisms = {item["id"] for item in robot_data["mechanisms"]}
    for item in [*data["streams"], *data.get("services", [])]:
        endpoint = item.get("native", item.get("action", ""))
        category, _, path = endpoint.partition(":")
        parts = path.split(".")
        where = f"/{item['id']}"
        if category == "sensor":
            sensor = sensors.get(parts[0])
            if sensor is None:
                problems.append(f"{where}: unknown robot sensor '{parts[0]}'")
            elif len(parts) > 1 and parts[1] not in sensor["parameters"].get("outputs", []):
                problems.append(f"{where}: sensor '{parts[0]}' has no output '{parts[1]}'")
        if category == "command" and parts[0] == "mechanisms" and len(parts) > 2:
            if parts[1] not in mechanisms:
                problems.append(f"{where}: unknown robot mechanism '{parts[1]}'")
    return problems


# ------------------------------------------------------------------ scenario

def scenario(data: dict[str, Any], robot_data: dict[str, Any], tasks_data: dict[str, Any],
             task_ids: list[str], mechanism_types: Counter[str]) -> tuple[list[str], dict[str, Any]]:
    """Check scenario bindings and return resolved run options (defaults applied)."""
    problems: list[str] = []
    placed = [item["task"] for item in data["task_placements"]]
    duplicates(placed, "task placement", problems)
    for name in placed:
        if name not in task_ids:
            problems.append(f"/task_placements: unknown task '{name}'")
    for name in task_ids:
        if name not in placed:
            problems.append(f"/task_placements: task '{name}' has no placement")
    if data["initial"]["frame"] not in robot_frames(robot_data):
        problems.append(f"/initial/frame: unknown robot frame '{data['initial']['frame']}'")
    for sensor in robot_data["sensors"]:
        if sensor.get("enabled", True) and sensor["period_ns"] < data["timestep_ns"]:
            problems.append(f"robot sensor '{sensor['id']}': period_ns {sensor['period_ns']} is "
                            f"shorter than timestep_ns {data['timestep_ns']}")
    for requirement in tasks_data["requires"]:
        if requirement["task"] not in task_ids:
            problems.append(f"tasks /requires: unknown task '{requirement['task']}'")
        if mechanism_types[requirement["mechanism_type"]] < requirement["min_count"]:
            problems.append(f"task '{requirement['task']}' requires {requirement['min_count']} "
                            f"{requirement['mechanism_type']} mechanism(s); robot has "
                            f"{mechanism_types[requirement['mechanism_type']]}")
    declared = {item["key"]: item for item in tasks_data["run_options"]}
    options = {key: item["default"] for key, item in declared.items()}
    for key, value in data["run"]["options"].items():
        option = declared.get(key)
        if option is None:
            problems.append(f"/run/options/{key}: unknown run option")
        elif option["type"] == "bool" and not isinstance(value, bool):
            problems.append(f"/run/options/{key}: must be a boolean")
        elif option["type"] == "choice" and value not in option["choices"]:
            problems.append(f"/run/options/{key}: must be one of {option['choices']}")
        else:
            options[key] = value
    _quaternions(data, "", problems)
    return problems, options
