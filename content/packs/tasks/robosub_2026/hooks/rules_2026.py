"""Pure 2026 gate/style and return-home scoring; other task rules remain unimplemented.

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


def evaluate(state: Mapping[str, Any], events: Sequence[Mapping[str, Any]],
             parameters: Mapping[str, Any]) -> dict[str, Any]:
    """Award gate/home from ordered events; emitted role_selected persists the first role."""
    result: dict[str, Any] = {"scores": [], "events": []}
    run = state["run"]
    if not run["running"] or run["ended"]:
        return result
    options, points = run["options"], parameters["points"]
    history = [event for event in state["history"] if event["time_ns"] >= run["started_ns"]]
    selected = next((event for event in history
                     if event["type"] == "role_selected" and event["task"] == "gate"), None)
    role = None if selected is None else selected["data"]["role"]
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

    for event in events:
        if (event["time_ns"] < run["started_ns"] or event["task"] != "gate"
                or event["region"] != "gate_opening"):
            continue
        data = event["data"]
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
