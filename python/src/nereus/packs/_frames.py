"""Robot geometry expressed in the physics body frame, as the runtime consumes it.

A robot pack may be measured from any datum: its frame tree is rooted wherever is convenient, the
centre of mass is one more frame (``frames.body``), and thrusters, collision boxes and the altitude
target may name the frame they sit in. Resolution rewrites those into the body frame, so a moved centre of mass is
one edited transform. Plain float arithmetic only: identical inputs give identical manifests on every
platform, and values that are already exact (identity mounts, unit quaternions) keep their digits.
"""

from __future__ import annotations

import math
from typing import Any

Vector = list[float]
Pose = tuple[Vector, Vector]  # (position_m, orientation_wxyz): child coordinates into the parent


def _rotate(q: Vector, v: Vector) -> Vector:
    """Rotate ``v`` by the unit quaternion ``q`` (wxyz) via its rotation matrix."""
    w, x, y, z = q
    rows = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return [row[0] * v[0] + row[1] * v[1] + row[2] * v[2] + 0.0 for row in rows]  # + 0.0: no -0.0


def _multiply(a: Vector, b: Vector) -> Vector:
    """Hamilton product ``a * b`` of wxyz quaternions (``b`` applied first)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz + 0.0,
        aw * bx + ax * bw + ay * bz - az * by + 0.0,
        aw * by - ax * bz + ay * bw + az * bx + 0.0,
        aw * bz + ax * by - ay * bx + az * bw + 0.0,
    ]


def _unit(vector: Vector) -> Vector:
    """Normalised, unless the norm is already 1 to rounding (authored digits are kept)."""
    norm = math.sqrt(sum(value * value for value in vector))
    return vector if abs(norm - 1.0) < 1e-15 else [value / norm for value in vector]


def _pose(item: dict[str, Any]) -> Pose:
    """Pose of a pack transform entry (``position_m`` + ``orientation_wxyz``)."""
    return [float(value) for value in item["position_m"]], _unit(
        [float(value) for value in item["orientation_wxyz"]]
    )


def _compose(parent: Pose, child: Pose) -> Pose:
    """Chain poses: ``child`` is expressed in ``parent``'s frame; result is in parent's parent."""
    position = [a + b for a, b in zip(parent[0], _rotate(parent[1], child[0]))]
    return position, _unit(_multiply(parent[1], child[1]))


def _inverse(pose: Pose) -> Pose:
    """Inverse transform: parent coordinates into the child."""
    w, x, y, z = pose[1]
    conjugate = [w, -x + 0.0, -y + 0.0, -z + 0.0]
    return _rotate(conjugate, [-value for value in pose[0]]), conjugate


def express_in_body(robot: dict[str, Any]) -> None:
    """Rewrite a validated robot pack's geometry into its body frame, in place.

    The frame tree is re-rooted at ``frames.body``; thrusters placed by ``frame`` gain ``position_m``
    and ``direction`` (the frame's +X); collision boxes leave their named frame; an altitude
    ``target_frame`` becomes its body position. A pack that names none of these is left untouched.
    """
    frames = robot["frames"]
    body = frames.pop("body", frames["root"])

    # Walk from the body frame up to the authored root, collecting the edges on that path
    edges = {item["child"]: item for item in frames["transforms"]}
    path, node = [], body
    while node != frames["root"]:
        path.append(edges[node])
        node = edges[node]["parent"]
    # The edges between the body frame and the authored root now point away from the body.
    for edge in path:
        position, orientation = _inverse(_pose(edge))
        edge.update(
            parent=edge["child"],
            child=edge["parent"],
            position_m=position,
            orientation_wxyz=orientation,
        )
    frames["root"] = body

    # Body-frame pose of every frame, resolved lazily through the re-rooted tree
    edges = {item["child"]: item for item in frames["transforms"]}
    resolved: dict[str, Pose] = {body: ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])}

    def from_body(name: str) -> Pose:
        if name not in resolved:
            edge = edges[name]
            resolved[name] = _compose(from_body(edge["parent"]), _pose(edge))
        return resolved[name]

    # Thrusters placed by frame: position plus thrust along the frame's +X
    for thruster in robot["thrusters"]:
        if "frame" in thruster:
            position, orientation = from_body(thruster.pop("frame"))
            thruster["position_m"] = position
            thruster["direction"] = _unit(_rotate(orientation, [1.0, 0.0, 0.0]))

    # Collision boxes authored in a named frame
    for box in robot["collision_boxes"]:
        if "frame" in box:
            local = (
                [float(value) for value in box["center_m"]],
                _unit([float(value) for value in box["orientation_wxyz"]]),
            )
            box["center_m"], box["orientation_wxyz"] = _compose(from_body(box.pop("frame")), local)

    # Altitude sensors reporting a named frame's altitude get that frame's body position
    for sensor in robot["sensors"]:
        if "target_frame" in sensor["parameters"]:
            target = from_body(sensor["parameters"].pop("target_frame"))
            sensor["parameters"]["target_position_body_m"] = target[0]
