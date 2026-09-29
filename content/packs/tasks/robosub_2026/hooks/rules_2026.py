"""Pure 2026 gate/style, home and torpedo scoring; other tasks remain unimplemented.

Geometry events are supplied by the platform. This hook owns competition rules only,
returns absolute max-per-row updates, and never changes the supplied run snapshot.
"""

import math
from collections.abc import Mapping, Sequence
from typing import Any


def _forward(event: Mapping[str, Any]) -> bool:
    return (event["task"] == "gate" and event["region"] == "gate_opening"
            and event["id"] == "forward_pass" and event["type"] == "pass_through"
            and event["data"]["attempt_id"] > 0
            and event["data"]["from_side"] == "positive"
            and event["data"]["to_side"] == "negative")


def _projectile_id(data: Mapping[str, Any]) -> int:
    identifier = data.get("projectile_id")
    if type(identifier) is not int or identifier < 0:
        raise ValueError("launcher event requires a nonnegative integer projectile_id")
    return identifier


def _distance(data: Mapping[str, Any]) -> float:
    value = data.get("release_distance_m")
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0):
        raise ValueError("launcher release requires a finite nonnegative release_distance_m")
    return float(value)


def evaluate(state: Mapping[str, Any], events: Sequence[Mapping[str, Any]],
             parameters: Mapping[str, Any]) -> dict[str, Any]:
    """Award from ordered events; emitted role/shot events preserve accepted ledger state.

    Launcher release/hit events with malformed scoring fields raise ValueError. Other
    mechanism types/tasks are ignored; bins/dropper and pinger bonuses are not implemented.
    """
    result: dict[str, Any] = {"scores": [], "events": []}
    run = state["run"]
    if not run["running"] or run["ended"]:
        return result
    options, points = run["options"], parameters["points"]
    history = [event for event in state["history"] if event["time_ns"] >= run["started_ns"]]
    selected = next((event for event in history
                     if event["type"] == "role_selected" and event["task"] == "gate"), None)
    role = None if selected is None else selected["data"]["role"]
    shots: dict[int, dict[str, Any]] = {}
    for event in history:
        if event["task"] != "torpedo":
            continue
        data = event["data"]
        if event["type"] == "shot_registered":
            shots.setdefault(data["projectile_id"], dict(data, result=None))
        elif event["type"] == "shot_result":
            shot = shots.get(data["projectile_id"])
            if shot is not None and shot["result"] is None:
                shot.update(data)
    passed_attempts = set()
    if selected is not None:
        passed_attempts = {event["data"]["attempt_id"] for event in history
                           if event["time_ns"] >= selected["time_ns"] and _forward(event)}
    scores = dict(state["scores"])
    updated = {}

    def award(row: str, value: int) -> None:
        if value > scores.get(row, 0):
            scores[row] = value
            updated[row] = value

    def gate_points(style: int = 0) -> int:
        bonus = points["role"] if options["role_coin"] and role == options["role"] else 0
        return points["gate"] + points["heading"] * options["heading_coin"] + bonus + style

    def emit_shot(event: Mapping[str, Any], kind: str, data: dict[str, Any]) -> None:
        result["events"].append({"id": f"torpedo:{kind}", "type": kind, "task": "torpedo",
                                 "region": event["region"], "time_ns": event["time_ns"],
                                 "data": data})

    def award_torpedoes() -> None:
        rules = parameters["torpedo"]
        good = [shot for shot in shots.values() if shot["result"] in rules["good_results"]]
        award("torpedoes", points["torpedo"] * len(good))
        award("distance", sum(points["distance_far"] if shot["release_distance_m"] >=
                              points["distance_far_m"] else points["distance_near"] if
                              shot["release_distance_m"] >= points["distance_near_m"] else 0
                              for shot in good))
        if (len(shots) == len(rules["sequence_order"])
                and all(shot["result"] in rules["good_results"] and shot["correct"]
                        for shot in shots.values())
                and [shot["hole_size"] for shot in shots.values()] == list(rules["sequence_order"])):
            award("sequence", points["sequence"])

    for event in events:
        if event["time_ns"] < run["started_ns"]:
            continue
        data = event["data"]
        if event["task"] == "torpedo" and data.get("mechanism_type") == "launcher":
            if event["type"] == "payload_released" and event["id"] == "payload_released":
                identifier, distance = _projectile_id(data), _distance(data)
                if identifier not in shots and len(shots) < points["max_shots"]:
                    registered = {"projectile_id": identifier, "mechanism_type": "launcher",
                                  "release_distance_m": distance, "eligible": role is not None}
                    shots[identifier] = dict(registered, result=None)
                    emit_shot(event, "shot_registered", registered)
            elif (event["type"] == "hit" and event["region"] == "panel"
                  and event["id"] in ("hole_pass", "panel_blocked")):
                identifier = _projectile_id(data)
                outcome = data.get("outcome")
                if outcome not in ("pass", "blocked", "timeout"):
                    raise ValueError("launcher hit requires pass, blocked or timeout outcome")
                if (outcome == "pass") != (event["id"] == "hole_pass"):
                    raise ValueError("launcher hit id differs from its outcome")
                if outcome == "pass" and any(not isinstance(data.get(key), str) or not data[key]
                                             for key in ("hole_id", "hole_class", "hole_size")):
                    raise ValueError("launcher passage requires hole_id, hole_class and hole_size")
                shot = shots.get(identifier)
                if role is not None and shot is not None and shot["eligible"] and shot["result"] is None:
                    correct = data.get("hole_class") == parameters["roles"][role]["target_class"]
                    accepted = {"projectile_id": identifier, "mechanism_type": "launcher",
                                "result": ("success" if correct else "wrong_target")
                                if outcome == "pass" else outcome,
                                "correct": correct, "hole_id": data.get("hole_id", ""),
                                "hole_size": data.get("hole_size", "")}
                    shot.update(accepted)
                    emit_shot(event, "shot_result", accepted)
                    award_torpedoes()
            continue
        if event["task"] != "gate" or event["region"] != "gate_opening":
            continue
        if _forward(event):
            if role is None:
                y = data["crossing_point_local"][1]
                reference = parameters["gate"]["role_reference_frame"]
                repair_y = state["tasks"]["gate"]["frames"][reference]["position_m"][1]
                role = "repair" if y * repair_y > 0 else "rescue"
                result["events"].append({
                    "id": "gate:role_selected", "type": "role_selected", "task": "gate",
                    "region": "gate_opening", "time_ns": event["time_ns"],
                    "data": {"role": role, "side": 1 if y > 0 else -1,
                             "attempt_id": data["attempt_id"]},
                })
            passed_attempts.add(data["attempt_id"])
            award("gate", gate_points())
        elif (event["id"] == "gate_opening:attempt_finished"
              and event["type"] == "attempt_finished" and role is not None
              and data["attempt_id"] in passed_attempts):
            style = parameters["gate"]["style"]
            quarters = [math.floor((abs(turn) + style["epsilon_rad"]) / style["quarter_rad"])
                        for turn in data["rotation_vector_body"]]
            limit = style["max_quarters"]
            if style["roll_pitch_first"]:
                rp = min(limit, quarters[0] + quarters[1])
                yaw = min(limit - rp, quarters[2])
            else:
                yaw = min(limit, quarters[2])
                rp = min(limit - yaw, quarters[0] + quarters[1])
            award("gate", gate_points(points["style_rp"] * rp + points["style_yaw"] * yaw))
        elif (event["id"] == "reverse_pass" and event["type"] == "pass_through"
              and data["from_side"] == "negative" and data["to_side"] == "positive"
              and role is not None
              and data["envelope_top_world"] < state["environment"]["surface_z_m"]):
            award("home", points["home"])
    result["scores"] = [{"row": row, "points": value} for row, value in updated.items()]
    return result
