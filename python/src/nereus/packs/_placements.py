"""Scenario placements resolved into world poses, as the runtime consumes them.

Every scenario placement may name the frame it is measured in (``relative_to``): ``world``, ``pool``, a task
id, an equipment placement id, or ``<equipment placement id>/<item frame id>``. Normally the world is the
root and the pool is placed in it. A scenario may instead place the world itself (``world_placement``, e.g.
the map origin a team's stack derives from a calibration tag); the pool is then the root. Resolution rewrites
the pool, task and start placements into world poses (yaw only for pools and tasks) and adds the world pose of
every equipment placement as ``equipment.placed``. Placements already in the world, with ``yaw_deg``, are left
exactly as authored.
"""

from __future__ import annotations

import math
from typing import Any

from ._frames import Pose, Vector, _compose, _inverse, _multiply, _unit

WORLD = "world"
POOL = "pool"
UPRIGHT_TOLERANCE = 1e-9  # on 1 - cos(tilt): about 4.5e-5 rad of tilt
DIGITS = 12  # resolved poses are rounded to 1e-12 (m, deg, quaternion): exact layouts stay exact, no float residue
IDENTITY: Pose = ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])


def _about(axis: int, degrees: float) -> Vector:
    """Quaternion (wxyz) for a rotation about axis 0/1/2 (x/y/z)."""
    half = math.radians(degrees) / 2
    q = [math.cos(half), 0.0, 0.0, 0.0]
    q[1 + axis] = math.sin(half)
    return q


def _orientation(item: dict[str, Any]) -> Vector:
    """Placement orientation: ``rpy_deg`` as intrinsic Z-Y-X (yaw, then pitch, then roll)."""
    if "rpy_deg" in item:
        roll, pitch, yaw = (float(value) for value in item["rpy_deg"])
        return _unit(_multiply(_multiply(_about(2, yaw), _about(1, pitch)), _about(0, roll)))
    if "yaw_deg" in item:
        return _about(2, float(item["yaw_deg"]))
    return [1.0, 0.0, 0.0, 0.0]


def _placed(item: dict[str, Any]) -> Pose:
    """A scenario placement: position_m with yaw_deg, rpy_deg or neither (level)."""
    return [float(value) for value in item["position_m"]], _orientation(item)


def _pose(item: dict[str, Any]) -> Pose:
    """A fixed frame: position_m and orientation_wxyz."""
    return [float(value) for value in item["position_m"]], _unit(
        [float(value) for value in item["orientation_wxyz"]]
    )


def _rounded(values: Vector) -> Vector:
    """Round to ``DIGITS`` decimals for the manifest."""
    return [round(value, DIGITS) + 0.0 for value in values]  # + 0.0: no -0.0


def _upright(pose: Pose, where: str, problems: list[str]) -> dict[str, Any] | None:
    """World pose as {position_m, yaw_deg}, or a problem when it is tilted."""
    w, x, y, z = pose[1]
    up = 1 - 2 * (x * x + y * y)  # world z component of the placed +z axis
    if 1 - up > UPRIGHT_TOLERANCE:
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up))))
        problems.append(
            f"{where}: must be upright in the world (yaw only), but is tilted {tilt:.3g} deg"
        )
        return None
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) + 0.0
    return {"position_m": _rounded(pose[0]), "yaw_deg": _rounded([yaw])[0]}


def resolve_placements(
    scenario: dict[str, Any], task_ids: list[str], equipment: dict[str, Any] | None
) -> list[str]:
    """Check the placement tree and rewrite it into world poses, in place. Returns problems."""
    problems: list[str] = []
    # frame -> (parent or None for the root, pose in the parent, JSON pointer of its placement)
    nodes: dict[str, tuple[str | None, Pose, str]] = {}

    # Report a placement that gives both orientation spellings (or neither, when required)
    def oriented(item: dict[str, Any], where: str, required: bool) -> None:
        if "yaw_deg" in item and "rpy_deg" in item:
            problems.append(f"{where}: give yaw_deg or rpy_deg, not both")
        elif required and "yaw_deg" not in item and "rpy_deg" not in item:
            problems.append(f"{where}: needs yaw_deg or rpy_deg")

    # Register a frame in the placement tree; frame names must be unique across all placements
    def add(name: str, parent: str | None, pose: Pose, where: str) -> None:
        if name in nodes:
            problems.append(f"{where}: frame '{name}' is already placed at {nodes[name][2]}")
        else:
            nodes[name] = (parent, pose, where)

    # Root of the tree: the pool when the world itself is placed, else the world
    if "world_placement" in scenario:
        placement = scenario["world_placement"]
        oriented(placement, "/world_placement", True)
        if "pool_placement" in scenario:
            problems.append("/world_placement: give pool_placement or world_placement, not both")
        if placement.get("relative_to", WORLD) == WORLD:
            problems.append("/world_placement/relative_to: must name another frame")
        add(POOL, None, IDENTITY, "/pool_placement")
        add(WORLD, placement.get("relative_to"), _placed(placement), "/world_placement")
    else:
        add(WORLD, None, IDENTITY, "/world_placement")
        if "pool_placement" not in scenario:
            problems.append("/: needs pool_placement or world_placement")
        else:
            placement = scenario["pool_placement"]
            oriented(placement, "/pool_placement", True)
            add(POOL, placement.get("relative_to", WORLD), _placed(placement), "/pool_placement")

    # Task frames
    for index, placement in enumerate(scenario["task_placements"]):
        where = f"/task_placements/{index}"
        oriented(placement, where, True)
        if placement["task"] in task_ids:  # unknown tasks are reported by the scenario checks
            add(placement["task"], placement.get("relative_to", WORLD), _placed(placement), where)

    # Equipment placements, plus each placed item's own fixed frames as "<placement>/<frame>"
    items = {item["id"]: item for item in (equipment or {}).get("items", [])}
    placements = scenario.get("equipment_placements", [])
    if placements and equipment is None:
        problems.append("/equipment_placements: the scenario selects no equipment pack")
    for index, placement in enumerate(placements):
        where = f"/equipment_placements/{index}"
        oriented(placement, where, False)
        item = items.get(placement["item"])
        if item is None:
            if equipment is not None:
                problems.append(f"{where}/item: unknown equipment item '{placement['item']}'")
            continue
        name = placement.get("id", placement["item"])
        add(name, placement.get("relative_to", WORLD), _placed(placement), where)
        for frame in item.get("frames", []):
            add(f"{name}/{frame['id']}", name, _pose(frame), where)

    # Every relative_to (including the robot start's) must name a placed frame
    initial_parent = scenario["initial"].get("relative_to", WORLD)
    references = [(parent, where) for parent, _, where in nodes.values()] + [
        (initial_parent, "/initial")
    ]
    for parent, where in references:
        if parent is not None and parent not in nodes:
            problems.append(f"{where}/relative_to: unknown frame '{parent}'")
    if problems:
        return problems

    # Resolve every frame's pose in the root frame, memoized, reporting cycles
    in_root: dict[str, Pose] = {}

    def resolve(name: str, visiting: tuple[str, ...] = ()) -> Pose | None:
        if name in in_root:
            return in_root[name]
        parent, pose, where = nodes[name]
        if name in visiting:
            problems.append(f"{where}: placement cycle {' -> '.join((*visiting, name))}")
            return None
        if parent is None:
            in_root[name] = pose
            return pose
        above = resolve(parent, (*visiting, name))
        if above is None:
            return None
        in_root[name] = _compose(above, pose)
        return in_root[name]

    for name in nodes:
        resolve(name)
    if problems:
        return problems
    # Re-express root poses in the world (identity unless the world itself was placed)
    world_from_root = _inverse(in_root[WORLD])

    def in_world(name: str) -> Pose:
        return _compose(world_from_root, in_root[name])

    # Placements already in the world with yaw_deg (or level) keep their authored values
    def rewritten(placement: dict[str, Any]) -> bool:
        return placement.get("relative_to", WORLD) != WORLD or "rpy_deg" in placement

    # Pool and tasks must end up yaw-only in the world
    if "world_placement" in scenario or rewritten(scenario["pool_placement"]):
        upright = _upright(in_world(POOL), "/pool_placement", problems)
        if upright is not None:
            scenario["pool_placement"] = upright
    for index, placement in enumerate(scenario["task_placements"]):
        if rewritten(placement):
            upright = _upright(in_world(placement["task"]), f"/task_placements/{index}", problems)
            if upright is not None:
                scenario["task_placements"][index] = {"task": placement["task"], **upright}

    # Robot start: a full 6-DOF pose, rewritten only when it was relative to another frame
    initial = scenario["initial"]
    if initial_parent != WORLD:
        start = _compose(in_world(initial_parent), _pose(initial))
        initial.pop("relative_to")
        initial["position_m"], initial["orientation_wxyz"] = _rounded(start[0]), _rounded(start[1])

    # World pose of every equipment placement, for the runtime
    if equipment is not None:
        equipment["placed"] = []
        for placement in placements:
            name = placement.get("id", placement["item"])
            position, orientation = in_world(name)
            equipment["placed"].append(
                {
                    "id": name,
                    "item": placement["item"],
                    "asset": items[placement["item"]]["asset"],
                    "position_m": _rounded(position),
                    "orientation_wxyz": _rounded(orientation),
                }
            )
    return problems
