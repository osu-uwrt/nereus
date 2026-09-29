"""Session run control, scorecard, event feed and visual-state accessors on the real packs.

No ROS and no middleware: the session is the single owner the bridge calls.
"""

from __future__ import annotations

import json
import math
import unittest
from pathlib import Path

from robotics_platform.pack_runtime import create_runtime
from robotics_platform.packs import resolve_scenario
from robotics_platform.session import Session

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt"
ROWS = ["gate", "slalom_front", "slalom_middle", "slalom_back", "bins", "lights", "torpedoes",
        "sequence", "distance", "surface", "facing", "objects_surface", "objects_drop", "baskets",
        "basket_count", "home", "pinger_first", "pinger_second"]


def make(auto_start: bool | None = True) -> Session:
    """auto_start None keeps the pack default (the run starts on the operator's "start")."""
    resolved = resolve_scenario(SCENARIO)
    if auto_start is not None:
        resolved.scenario["run"]["auto_start"] = auto_start
    sensors = [s["id"] for s in resolved.robot["sensors"] if s["type"] != "stereo_camera"]
    return Session(resolved, create_runtime(resolved, sensor_ids=sensors))


def run(session: Session, ticks: int) -> None:
    for _ in range(ticks):
        session.advance()
        for stream in session.pack.streams.values():
            stream.drain()


class RunControlTest(unittest.TestCase):
    def test_scorecard_rows_labels_and_fields_come_from_the_pack(self) -> None:
        session = make()
        run(session, 50)
        score = session.run_snapshot()
        assert score is not None
        json.dumps(score, allow_nan=False)
        self.assertEqual([row["key"] for row in score["rows"]], ROWS)
        self.assertEqual(score["rows"][0]["label"], "Gate passage / heading / role / style")
        self.assertEqual(score["rows"][14]["label"], "Signal basket count with turns")
        self.assertTrue(all(row["points"] == 0 for row in score["rows"]))
        for key in ("running", "elapsed", "scoring_open", "ended_reason", "intended_role", "role",
                    "target_class", "gate_passed", "total", "adjustment", "basket_count",
                    "pinger", "time_bonus_eligible", "message", "ui"):
            self.assertIn(key, score)
        self.assertEqual(score["ui"]["title"], "RoboSub 2026 scorecard")
        self.assertEqual(score["ui"]["run_options"][0]["choices"][1],
                         {"value": "rescue", "label": "Search & Rescue"})
        self.assertEqual((score["intended_role"], score["role"], score["target_class"]),
                         ("repair", None, "fire"))
        self.assertAlmostEqual(score["elapsed"], 0.1)  # auto-started run: sim seconds

    def test_start_resets_starts_with_options_and_is_rejected_while_running(self) -> None:
        session = make()
        run(session, 100)
        self.assertTrue(session.running)  # scenario auto_start
        rejected = session.run_start({"role": "rescue"})
        self.assertEqual((rejected.accepted, rejected.message), (False, "Stop the current run first"))
        self.assertTrue(session.run_stop().accepted)
        self.assertFalse(session.running)
        frozen = session.run_snapshot()["elapsed"]
        run(session, 50)
        self.assertEqual(session.run_snapshot()["elapsed"], frozen)  # frozen at the stop
        self.assertTrue(session.run_stop().accepted)  # idempotent
        self.assertEqual(session.run_snapshot()["elapsed"], frozen)
        result = session.run_start({"role": "rescue", "heading_coin": False, "role_coin": True})
        self.assertTrue(result.accepted, result.message)
        score = session.run_snapshot()
        self.assertEqual((score["running"], score["intended_role"], score["target_class"]),
                         (True, "rescue", "blood"))
        self.assertEqual(score["message"], "Run started; pass the gate first")
        self.assertEqual(score["elapsed"], 0.0)  # restarted at the current time
        options = session.tasks.snapshot()["run"]["options"]
        self.assertEqual((options["heading_coin"], options["role_coin"]), (False, True))
        run(session, 500)
        self.assertAlmostEqual(session.run_snapshot()["elapsed"], 1.0)

    def test_pack_default_waits_for_the_operator_start(self) -> None:
        session = make(auto_start=None)
        session.set_killed(True)  # boots killed: the positively buoyant robot floats up
        run(session, 3000)        # 6 s: surfaces outside the octagon before any run exists
        self.assertFalse(session.running)
        score = session.run_snapshot()
        self.assertEqual((score["running"], score["total"], score["ended_reason"]), (False, 0, ""))
        result = session.run_start()
        self.assertTrue(result.accepted, result.message)
        run(session, 500)
        score = session.run_snapshot()
        self.assertTrue(score["running"])
        self.assertEqual(score["ended_reason"], "")  # the pre-start surfacing is not a breach
        self.assertAlmostEqual(score["elapsed"], 1.0)

    def test_invalid_start_options_are_rejected_without_side_effects(self) -> None:
        session = make()
        session.run_stop()
        before = session.tasks.snapshot()["run"]["started_ns"]
        for options in ({"role": "pilot"}, {"heading_coin": "yes"}, {"nonsense": 1}):
            with self.subTest(options=options):
                self.assertFalse(session.run_start(options).accepted)
        self.assertEqual(session.tasks.snapshot()["run"]["started_ns"], before)
        self.assertFalse(session.running)

    def test_manual_adjustment_adds_to_total_and_resets_with_the_tasks(self) -> None:
        session = make()
        self.assertTrue(session.run_adjust(150).accepted)
        score = session.run_snapshot()
        self.assertEqual((score["adjustment"], score["total"]), (150.0, 150.0))
        self.assertEqual(score["message"], "Manual adjustment updated")
        for value in (math.nan, math.inf, "3", True):
            with self.subTest(value=value):
                self.assertFalse(session.run_adjust(value).accepted)
        self.assertEqual(session.run_snapshot()["adjustment"], 150.0)
        session.reset_tasks()
        score = session.run_snapshot()
        self.assertEqual((score["adjustment"], score["total"], score["message"]),
                         (0.0, 0.0, "Tasks and run reset"))

    def test_total_sums_every_awarded_row(self) -> None:
        session = make()
        session.tasks._scores.update({"gate": 100, "bins": 300, "extra/unlisted": 7})
        score = session.run_snapshot()
        rows = {row["key"]: row["points"] for row in score["rows"]}
        self.assertEqual((rows["gate"], rows["bins"], rows["extra/unlisted"]), (100, 300, 7))
        self.assertEqual(score["total"], 407.0)
        self.assertEqual(score["rows"][-1]["key"], "extra/unlisted")  # unlisted rows follow

    def test_feed_reports_release_and_result_with_counters_and_reset(self) -> None:
        session = make()
        session.take_feed()
        session.set_killed(False)
        self.assertTrue(session.set_armed(True).accepted)
        run(session, 1)
        self.assertTrue(session.fire("torpedo_launcher").accepted)
        run(session, 2000)
        feed = session.take_feed()
        self.assertEqual(feed[0], {"id": 0, "kind": "torpedo", "result": "released", "target": "",
                                   "time": 0.002, "slot": 0})
        self.assertEqual((feed[1]["kind"], feed[1]["result"], feed[1]["target"], feed[1]["slot"]),
                         ("torpedo", "miss", "pool_floor", 0))
        self.assertEqual(len(feed), 2)  # one release and one result per payload
        self.assertEqual(session.task_counters,
                         {"success": 0, "wrong_target": 0, "blocked": 0, "miss": 1})
        self.assertEqual(session.take_feed(), [])
        session.reset_tasks()
        self.assertEqual(session.task_counters["miss"], 0)
        reset = session.take_feed()
        self.assertEqual([{k: v for k, v in item.items() if k != "time"} for item in reset],
                         [{"kind": "tasks", "result": "reset", "target": ""}])

    def test_full_reset_publishes_a_reset_event_and_zeroes_counters(self) -> None:
        session = make()
        session.task_counters["miss"] = 3
        session.full_reset()
        self.assertEqual(session.task_counters["miss"], 0)
        self.assertEqual([item["kind"] for item in session.take_feed()], ["tasks"])


class VisualStateTest(unittest.TestCase):
    def test_thruster_forces_are_realized_and_in_pack_order(self) -> None:
        session = make()
        session.set_killed(False)
        forces = [0.0] * 8
        forces[2] = 12.0  # third robot-pack thruster
        session.command_thrusters(forces)
        run(session, 200)  # inside the 0.5 s command timeout
        realized = session.thruster_forces()
        self.assertEqual(realized.shape, (8,))
        self.assertGreater(realized[2], 5.0)
        self.assertLess(abs(realized[3]), 1e-9)

    def test_claw_jaws_follow_the_prop_world_and_indicators_latch(self) -> None:
        session = make()
        self.assertEqual(session.claw_jaws()["claw"], (0.0, 0.0))
        self.assertEqual([item["region"] for item in session.indicators()],
                         ["magnet_target1", "magnet_target2"])
        self.assertFalse(any(item["latched"] for item in session.indicators()))
        session.tasks._targets[("bins", "magnet_target2")].latched = True
        self.assertEqual([item["latched"] for item in session.indicators()], [False, True])

    def test_props_and_payloads_expose_assets_poses_and_slots(self) -> None:
        session = make()
        props = {item["id"]: item for item in session.prop_visuals()}
        self.assertEqual(set(props), {"pill", "bandage", "nut_and_bolt", "plug"})
        self.assertEqual(props["pill"]["asset"], "pill_visual")
        self.assertFalse(props["pill"]["held"])
        loaded = [item for item in session.payload_visuals() if item["loaded"]]
        self.assertEqual([(item["mechanism_id"], item["id"]) for item in loaded],
                         [("torpedo_launcher", 0), ("torpedo_launcher", 1),
                          ("dropper", 0), ("dropper", 1)])
        session.set_killed(False)
        session.set_armed(True)
        run(session, 1)
        session.fire("dropper")
        items = session.payload_visuals()
        self.assertEqual([(i["mechanism_type"], i["id"], i["loaded"]) for i in items
                          if i["mechanism_type"] == "dropper"],
                         [("dropper", 0, False), ("dropper", 1, True)])  # slot 0 fired
        self.assertAlmostEqual(items[0]["length_m"], 0.08299993)


if __name__ == "__main__":
    unittest.main()
