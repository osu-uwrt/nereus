"""Pure 2026 scoring: gate/style, home, torpedo, slalom, bins, lights, surface and table rows.

Geometry events are supplied by the platform. This hook owns competition rules only,
returns absolute max-per-row updates, and never changes the supplied run snapshot.

Gate and torpedo rows are folded incrementally as before. Slalom, bins, lights, surface and
table rows come from a ledger that replays the run history and then the new events in order,
so every derived fact (role, held prop, basket contents, shots, lights) is a pure function of
the ordered event stream; awards only ever raise a row. A breach event ends scoring for good.

Event contract by task (platform-emitted unless noted):
  gate      pass_through forward_pass/reverse_pass, attempt_finished (existing).
  torpedo   payload_released, hit hole_pass/panel_blocked (existing).
  slalom    pass_through on regions slalom_front|middle|back (positive -> negative), data
            crossing_point_local (row frame; y sign = side) and depth_overlap (bool).
  bins      payload_released (mechanism dropper registers a shot, at most max_shots);
            payload_landing outcome inside|blocked with region = crate id, data region_class,
            mechanism_type; activate on regions magnet_target1|2 (light latched);
            miss (ignored: a missed payload scores nothing).
  surface   surface_reached / surface_lost (0.5 s dwell inside the octagon), facing_reached
            {target} / facing_lost, breach (outside the octagon: scoring ends),
            rotation_judged {turns} (basket-count turn judge on region turn_zone).
  table     supplied by the prop world (TaskRuntime.record / observe_events), task "table":
              {type attach, id grasp, data {prop_id, mechanism_id}}
              {type detach, id release, data {prop_id, mechanism_id, reason}}
                  reason "released" credits objects_drop; "slipped"/"reset" do not.
              {type drop_into, id basket_drop, region <basket box region id>,
               data {prop_id, basket, expected_basket}}  prop settled in a basket.
              (drop_into object_dropped, prop at rest outside the baskets, scores nothing.)
  A basket holds a prop from its basket_drop until the prop is grasped again.
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


def _select_role(event: Mapping[str, Any], state: Mapping[str, Any],
                 parameters: Mapping[str, Any]) -> tuple[str, int]:
    """Role (by the gate half crossed) and side sign from a first forward gate passage."""
    y = event["data"]["crossing_point_local"][1]
    reference = parameters["gate"]["role_reference_frame"]
    repair_y = state["tasks"]["gate"]["frames"][reference]["position_m"][1]
    return ("repair" if y * repair_y > 0 else "rescue"), (1 if y > 0 else -1)


class _Ledger:
    """Slalom, bins, lights, surface and table rows as a fold over the ordered event stream.

    ``award(row, value)`` only ever raises a row. ``emit`` is None while replaying history.
    Eligibility is the original one: scoring open (running, no breach) and the gate passed.
    """

    def __init__(self, state: Mapping[str, Any], parameters: Mapping[str, Any], award: Any):
        self.state, self.parameters, self.points = state, parameters, parameters["points"]
        self.award, self.emit = award, None
        self.role: str | None = None
        self.side = 0
        self.ended = False
        self.shots: dict[int, dict[str, Any]] = {}
        self.lights: set[str] = set()
        self.grasped: set[str] = set()
        self.dropped: set[str] = set()
        self.surfaced: set[str] = set()
        self.contents: dict[str, str] = {}
        self.basket_awards: dict[str, int] = {}
        self.held: str | None = None
        self.surface_active = False
        self.facing: str | None = None

    def feed(self, event: Mapping[str, Any]) -> None:
        if self.ended:
            return
        task, kind, data = event["task"], event["type"], event["data"]
        if _forward(event) and self.role is None:
            self.role, self.side = _select_role(event, self.state, self.parameters)
        elif kind == "breach" and task == "surface":
            self.ended = True
            if self.emit is not None:
                self.emit({"id": "surface:scoring_ended", "type": "scoring_ended",
                           "task": task, "region": event["region"], "time_ns": event["time_ns"],
                           "data": {"reason": "breach outside octagon"}})
            return
        elif task == "slalom" and kind == "pass_through":
            self.slalom(event)
        elif task == "bins":
            self.bins(event)
        elif task == "surface":
            if kind == "surface_reached":
                self.surface_active = True
            elif kind == "surface_lost":
                self.surface_active, self.facing = False, None
            elif kind == "facing_reached":
                self.facing = data["target"]
            elif kind == "facing_lost":
                self.facing = None
            elif kind == "rotation_judged":
                self.basket_turns(data["turns"])
        elif task == "table":
            self.table(event)
        self.surface_rows()

    @property
    def target_class(self) -> str:
        return str(self.parameters["roles"][self.role]["target_class"])

    def slalom(self, event: Mapping[str, Any]) -> None:
        data = event["data"]
        if (self.role is None or event["region"] not in self.parameters["slalom"]["rows"]
                or data["from_side"] != "positive" or data["to_side"] != "negative"):
            return
        side = 1 if data["crossing_point_local"][1] > 0 else -1
        points = self.points
        self.award(event["region"], (points["slalom_same"] if side == self.side
                                     else points["slalom_other"])
                   + points["slalom_depth"] * bool(data["depth_overlap"]))

    def bins(self, event: Mapping[str, Any]) -> None:
        data, points = event["data"], self.points
        if event["type"] == "activate":
            if self.role is not None:
                self.lights.add(event["region"])
                self.award("lights", points["light"] * min(points["max_lights"], len(self.lights)))
            return
        if data.get("mechanism_type") != self.parameters["bins"]["mechanism_type"]:
            return
        identifier = data.get("projectile_id")
        if event["type"] == "payload_released":
            if identifier not in self.shots and len(self.shots) < points["max_shots"]:
                self.shots[identifier] = {"eligible": self.role is not None, "result": None,
                                          "target": "", "correct": False}
        elif event["type"] == "payload_landing" and data["outcome"] == "inside":
            shot = self.shots.get(identifier)
            if self.role is None or shot is None or not shot["eligible"] or shot["result"]:
                return
            correct = data["region_class"] == self.target_class
            shot.update(result="success" if correct else "wrong_target", correct=correct,
                        target=event["region"])
            good = [item for item in self.shots.values()
                    if item["result"] in ("success", "wrong_target")]
            unique = {item["target"] for item in good if item["correct"]}
            self.award("bins", points["bin"] * len(good) + points["bin_class"] * len(unique))

    def table(self, event: Mapping[str, Any]) -> None:
        data, points = event["data"], self.points
        prop = data.get("prop_id")
        if event["type"] == "attach":
            self.held = prop
            if self.role is not None:
                self.grasped.add(prop)
                self.contents.pop(prop, None)
        elif event["type"] == "detach":
            if self.held == prop:
                self.held = None
            if (self.role is not None and prop in self.grasped
                    and data.get("reason", "released") == "released"):
                self.dropped.add(prop)
                self.award("objects_drop", points["object_drop"] * len(self.dropped))
        elif (event["type"] == "drop_into" and self.role is not None
              and data.get("basket") in self.parameters["table"]["baskets"]):
            self.contents[prop] = data["basket"]
            value = (points["basket_correct"] if data["basket"] == data["expected_basket"]
                     else points["basket_other"])
            self.basket_awards[prop] = max(value, self.basket_awards.get(prop, 0))
            self.award("baskets", sum(self.basket_awards.values()))

    def basket_turns(self, turns: int) -> None:
        count, points = len(self.contents), self.points
        if self.role is not None and count > 0 and turns > 0:
            self.award("basket_count", points["basket_count"] if turns == count
                       else points["basket_count_near"] if abs(turns - count) == 1 else 0)

    def surface_rows(self) -> None:
        if self.role is None or not self.surface_active:
            return
        points = self.points
        self.award("surface", points["surface"])
        if self.held is not None and self.held in self.grasped:
            self.surfaced.add(self.held)
            self.award("objects_surface", points["object_surface"] * len(self.surfaced))
        if self.facing is not None:
            icons = self.parameters["roles"][self.role]["facing_icons"]
            value = points["facing_correct"] if self.facing in icons else points["facing_other"]
            count = len(self.contents)
            if count > 0 and self.facing == icons[min(count, 2) - 1]:
                value = points["facing_count"]
            self.award("facing", value)


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

    def handle(event: Mapping[str, Any]) -> None:
        nonlocal role
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
            return
        if event["task"] != "gate" or event["region"] != "gate_opening":
            return
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

    ledger = _Ledger(state, parameters, award)
    for event in history:
        ledger.feed(event)
    ledger.emit = result["events"].append
    for event in events:
        if event["time_ns"] < run["started_ns"] or ledger.ended:
            continue
        handle(event)
        ledger.feed(event)
    result["scores"] = [{"row": row, "points": value} for row, value in updated.items()]
    return result


ENDED_REASON = "Breach outside octagon; scoring ended (timer remains manual)"


def _replayed(state: Mapping[str, Any], parameters: Mapping[str, Any]) -> _Ledger:
    ledger = _Ledger(state, parameters, lambda row, value: None)
    started = state["run"]["started_ns"]
    for event in state["history"]:
        if event["time_ns"] >= started and not ledger.ended:
            ledger.feed(event)
    return ledger


def describe(state: Mapping[str, Any], parameters: Mapping[str, Any]) -> dict[str, Any]:
    """Viewer-facing scalar fields of the run scorecard (never used for scoring).

    Pure function of the frozen run state and its event history: the assigned and scored
    role, the target class of the current role, whether the gate was passed, the basket
    count, the breach message and the time-bonus prerequisites.
    """
    intended = state["run"]["options"]["role"]
    ledger = _replayed(state, parameters)
    role = ledger.role
    scores = state["scores"]
    slalom = parameters["slalom"]["rows"]
    return {
        "intended_role": intended,
        "role": role,
        "target_class": parameters["roles"][role or intended]["target_class"],
        "gate_passed": role is not None,
        "ended_reason": ENDED_REASON if ledger.ended else "",
        "basket_count": len(ledger.contents),
        "pinger": {"mode": "disabled", "first": None, "active": None, "stage": 0},
        "time_bonus_eligible": bool(
            scores.get("surface") and any(scores.get(row) for row in slalom)
            and (scores.get("bins") or scores.get("torpedoes"))),
    }


def feed(state: Mapping[str, Any], events: Sequence[Mapping[str, Any]],
         context: Mapping[str, Any], parameters: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Operator event feed for the events just committed (original task-node vocabulary).

    kind torpedo|dropper (released, success, wrong_target, blocked, miss; with slot), claw
    (grasped, released/slipped/reset, success, wrong_target) and magnet (activated). One
    result per payload; results are judged against the target class of the current role.
    """
    spec = parameters["feed"]
    kinds, baskets = spec["kinds"], spec["basket_names"]
    target_class = describe(state, parameters)["target_class"]
    released = context["payloads"]
    seen: set[tuple[Any, ...]] = set()
    items: list[dict[str, Any]] = []

    def add(identifier: Any, kind: str, result: str, target: str, time_ns: int,
            slot: int | None = None) -> None:
        key = (kind, identifier, result)
        if key in seen:
            return
        seen.add(key)
        item = {"id": identifier, "kind": kind, "result": result, "target": target,
                "time": time_ns / 1e9}
        if slot is not None:
            item["slot"] = slot
        items.append(item)

    for event in events:
        kind, data, time_ns = event["type"], event["data"], event["time_ns"]
        mechanism = data.get("mechanism_type")
        identifier = data.get("projectile_id")
        slot = released.get(identifier, {}).get("slot")
        if kind == "payload_released" and mechanism in kinds:
            add(identifier, kinds[mechanism], "released", "", time_ns, slot)
        elif kind == "hit" and mechanism in kinds:
            if data["outcome"] == "pass":
                result = "success" if data.get("hole_class") == target_class else "wrong_target"
                add(identifier, kinds[mechanism], result, data.get("hole_id") or "", time_ns, slot)
            elif data["outcome"] == "blocked":
                add(identifier, kinds[mechanism], "blocked", data.get("hole_id") or "", time_ns, slot)
        elif kind == "payload_landing" and mechanism in kinds:
            if data["outcome"] == "inside" and mechanism == parameters["bins"]["mechanism_type"]:
                result = "success" if data["region_class"] == target_class else "wrong_target"
            else:
                result = "blocked"
            add(identifier, kinds[mechanism], result, event["region"], time_ns, slot)
        elif kind == "miss" and mechanism in kinds:
            add(identifier, kinds[mechanism], "miss", str(data.get("reason", "")), time_ns, slot)
        elif kind == "activate":
            add("", spec["magnet_kind"], "activated", event["region"], time_ns)
        elif kind == "attach" and event["task"] == "table":
            add(data["prop_id"], kinds["claw"], "grasped", "", time_ns)
        elif kind == "detach" and event["task"] == "table":
            add(data["prop_id"], kinds["claw"], data.get("reason", "released"), "", time_ns)
        elif kind == "drop_into" and event["id"] == "basket_drop":
            result = "success" if data["basket"] == data["expected_basket"] else "wrong_target"
            add(data["prop_id"], kinds["claw"], result,
                baskets.get(data["basket"], data["basket"]), time_ns)
    return items
