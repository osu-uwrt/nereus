"""The pack's pure gate/home hook, including optional original-ledger comparisons."""

import copy
import importlib.util
import math
import os
import unittest
from pathlib import Path
from types import MappingProxyType

from ruamel.yaml import YAML

ROOT = Path(__file__).resolve().parents[2]
PACK = ROOT / "content/packs/tasks/robosub_2026"


def module_at(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


HOOK = module_at(PACK / "hooks/rules_2026.py", "rules_2026_test")
PARAMETERS = YAML(typ="safe").load((PACK / "tasks.yaml").read_text())["scoring_hooks"][0][
    "parameters"]


def frozen(value):
    if isinstance(value, dict):
        return MappingProxyType({key: frozen(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(frozen(item) for item in value)
    return value


def state(**options):
    return {"run": {"running": True, "ended": False, "started_ns": 10,
                    "options": {"role": "repair", "heading_coin": True,
                                "role_coin": True, **options}},
            "scores": {}, "history": [],
            "tasks": {"gate": {"frames": {"gate_repair": {
                "position_m": (0.0, -0.75, 0.5), "orientation_wxyz": (1.0, 0.0, 0.0, 0.0)}}}},
            "environment": {"surface_z_m": 0.0}}


def crossing(y=-0.5, attempt=1, reverse=False, top=-0.1, time_ns=20):
    return {"id": "reverse_pass" if reverse else "forward_pass", "type": "pass_through",
            "task": "gate", "region": "gate_opening", "time_ns": time_ns,
            "data": {"from_side": "negative" if reverse else "positive",
                     "to_side": "positive" if reverse else "negative",
                     "crossing_point_local": (0.0, y, -0.5), "envelope_top_world": top,
                     "attempt_id": attempt}}


def finished(turns=(0.0, 0.0, 0.0), attempt=1, time_ns=30):
    return {"id": "gate_opening:attempt_finished", "type": "attempt_finished", "task": "gate",
            "region": "gate_opening", "time_ns": time_ns,
            "data": {"rotation_vector_body": turns, "attempt_id": attempt}}


def release(identifier=1, distance=0.5, time_ns=40):
    return {"id": "payload_released", "type": "payload_released", "task": "torpedo",
            "region": "", "time_ns": time_ns,
            "data": {"projectile_id": identifier, "mechanism_type": "launcher",
                     "release_distance_m": distance}}


def hit(identifier=1, *, outcome="pass", hole_class="fire", size="large", time_ns=50):
    return {"id": "hole_pass" if outcome == "pass" else "panel_blocked", "type": "hit",
            "task": "torpedo", "region": "panel", "time_ns": time_ns,
            "data": {"projectile_id": identifier, "mechanism_type": "launcher",
                     "outcome": outcome, "hole_id": f"{hole_class}_{size}",
                     "hole_class": hole_class, "hole_size": size,
                     "hit_point_local": (0.0, 0.1, 0.2)}}


def apply(snapshot, *events, parameters=PARAMETERS):
    before = copy.deepcopy(snapshot)
    result = HOOK.evaluate(frozen(snapshot), frozen(events), frozen(parameters))
    assert snapshot == before
    snapshot["scores"].update({item["row"]: item["points"] for item in result["scores"]})
    snapshot["history"].extend([*events, *result["events"]])
    return result


class GateHomeScoringTests(unittest.TestCase):
    def test_roles_coins_and_first_role_are_frozen(self):
        for intended in ("repair", "rescue"):
            for y, actual in ((-0.5, "repair"), (0.5, "rescue"), (0.0, "rescue")):
                for heading in (False, True):
                    for role_coin in (False, True):
                        with self.subTest(intended=intended, y=y, heading=heading, coin=role_coin):
                            run = state(role=intended, heading_coin=heading, role_coin=role_coin)
                            result = apply(run, crossing(y))
                            expected = 100 + 300 * heading + 150 * (role_coin and intended == actual)
                            self.assertEqual(run["scores"], {"gate": expected})
                            self.assertEqual(result["events"][0]["data"]["role"], actual)
                            self.assertEqual(result["events"][0]["data"]["side"], 1 if y > 0 else -1)
                            self.assertEqual(apply(run, crossing(-y, attempt=2))["events"], [])
                            self.assertEqual(run["scores"], {"gate": expected})

    def test_reference_frame_sign_comes_from_task_data(self):
        run = state()
        run["tasks"]["gate"]["frames"]["gate_repair"]["position_m"] = (0.0, 0.75, 0.5)
        result = apply(run, crossing(0.5))
        self.assertEqual(result["events"][0]["data"]["role"], "repair")

    def test_forward_requires_an_attempt_but_home_does_not(self):
        run = state()
        self.assertEqual(apply(run, crossing(attempt=0)), {"scores": [], "events": []})
        apply(run, crossing(attempt=1))
        result = apply(run, crossing(reverse=True, attempt=0))
        self.assertEqual(result["scores"], [{"row": "home", "points": 300}])

    def test_style_attempt_matching_quarters_epsilon_priority_and_max(self):
        q = math.pi / 2
        cases = [((q - 2e-6, 0, 0), 0), ((q - 0.5e-6, 0, 0), 200),
                 ((-q, -q, q), 500), ((7 * q, q, 8 * q), 1600),
                 ((2 * q, 0, 8 * q), 1000), ((0, 0, -20 * q), 800)]
        for turns, style in cases:
            with self.subTest(turns=turns):
                run = state()
                self.assertEqual(apply(run, finished(turns))["scores"], [])
                apply(run, crossing())
                self.assertEqual(apply(run, finished(turns, attempt=2))["scores"], [])
                apply(run, finished(turns))
                self.assertEqual(run["scores"]["gate"], 550 + style)
                self.assertEqual(apply(run, finished())["scores"], [])
                self.assertEqual(apply(run, crossing(attempt=2))["scores"], [])

    def test_same_batch_gate_style_and_home_ordering(self):
        run = state()
        result = apply(run, crossing(reverse=True), crossing(),
                       finished((math.pi, 0, 0)), crossing(reverse=True))
        self.assertEqual(run["scores"], {"gate": 950, "home": 300})
        self.assertEqual(len(result["events"]), 1)
        self.assertEqual(len(result["scores"]), 2)

    def test_home_requires_gate_and_strict_submergence_at_shifted_surface(self):
        run = state()
        self.assertEqual(apply(run, crossing(reverse=True))["scores"], [])
        apply(run, crossing())
        run["environment"]["surface_z_m"] = 2.0
        for top in (2.0, 2.1):
            self.assertEqual(apply(run, crossing(reverse=True, top=top))["scores"], [])
        apply(run, crossing(reverse=True, top=1.999))
        self.assertEqual(run["scores"]["home"], 300)
        self.assertEqual(apply(run, crossing(reverse=True))["scores"], [])

    def test_inactive_ended_earlier_or_unrelated_events_do_not_score(self):
        for running, ended in ((False, False), (True, True)):
            run = state()
            run["run"].update(running=running, ended=ended)
            self.assertEqual(apply(run, crossing(), finished(), crossing(reverse=True)),
                             {"scores": [], "events": []})
        run = state()
        events = [crossing(time_ns=9), {**crossing(), "task": "torpedo"},
                  {**crossing(), "region": "another_opening"},
                  {**crossing(), "id": "another_event"}]
        self.assertEqual(apply(run, *events), {"scores": [], "events": []})
        # An ignored pre-start crossing must not choose the role of the new run.
        result = apply(run, crossing(0.5, attempt=2))
        self.assertEqual(result["events"][0]["data"]["role"], "rescue")

    def test_all_inputs_are_recursively_readonly(self):
        run = frozen(state())
        with self.assertRaises(TypeError):
            run["run"]["options"]["role"] = "rescue"
        with self.assertRaises(TypeError):
            run["tasks"]["gate"]["frames"]["gate_repair"]["position_m"][0] = 1.0
        result = HOOK.evaluate(run, frozen([crossing()]), frozen(PARAMETERS))
        self.assertEqual(result["scores"], [{"row": "gate", "points": 550}])


class TorpedoScoringTests(unittest.TestCase):
    def test_sequence_uses_release_order_not_result_order(self):
        run = state()
        apply(run, crossing(), release(1, .3048), release(2, .4572))
        apply(run, hit(2, size="small"), hit(1, size="large"))
        self.assertEqual(run["scores"], {"gate": 550, "torpedoes": 1200,
                                         "distance": 600, "sequence": 1400})
        self.assertEqual(apply(run, hit(1, outcome="blocked"))["scores"], [])
        self.assertEqual(apply(run, release(3), hit(3))["events"], [])

    def test_pre_gate_shot_registration_consumes_capacity_but_never_scores(self):
        run = state()
        registered = apply(run, release(1))
        self.assertFalse(registered["events"][0]["data"]["eligible"])
        self.assertEqual(apply(run, hit(1))["events"], [])
        apply(run, crossing(), hit(1), release(2), release(3), hit(3), hit(2))
        self.assertEqual(run["scores"], {"gate": 550, "torpedoes": 600, "distance": 400})
        self.assertEqual(apply(run, release(1), hit(1))["events"], [])

    def test_wrong_class_still_scores_shot_and_distance_but_not_sequence(self):
        run = state(role="repair")
        apply(run, crossing(.5), release(1), release(2), hit(1), hit(2, size="small"))
        self.assertEqual(run["scores"], {"gate": 400, "torpedoes": 1200, "distance": 800})
        results = [event["data"]["result"] for event in run["history"]
                   if event["type"] == "shot_result"]
        self.assertEqual(results, ["wrong_target", "wrong_target"])

    def test_failed_first_result_is_terminal_and_unknown_results_are_ignored(self):
        for outcome in ("blocked", "timeout"):
            run = state()
            apply(run, crossing())
            self.assertEqual(apply(run, hit(1))["events"], [])
            apply(run, release(1), hit(1, outcome=outcome))
            self.assertEqual(apply(run, hit(1))["events"], [])
            self.assertEqual(run["scores"], {"gate": 550})

    def test_distance_thresholds_and_duplicate_release_keep_original_distance(self):
        for distance, expected in ((.3048 - 1e-9, 0), (.3048, 200),
                                   (.4572 - 1e-9, 200), (.4572, 400)):
            run = state()
            apply(run, crossing(), release(1, distance), release(1, 999), hit(1))
            self.assertEqual(run["scores"].get("distance", 0), expected)

    def test_reverse_sequence_stopped_ended_and_unsupported_mechanisms(self):
        run = state()
        apply(run, crossing(), release(1), release(2), hit(1, size="small"), hit(2))
        self.assertNotIn("sequence", run["scores"])
        for field in ("running", "ended"):
            closed = state()
            closed["run"][field] = field == "ended"
            self.assertEqual(apply(closed, release(), hit()), {"scores": [], "events": []})
        event = release()
        event["data"]["mechanism_type"] = "dropper"
        self.assertEqual(apply(state(), event), {"scores": [], "events": []})

    def test_malformed_launcher_events_raise_without_mutating_input(self):
        bad_events = [release(True), release(-1), release(distance=float("nan")),
                      release(distance=-1), hit(outcome="unknown")]
        bad = hit()
        del bad["data"]["hole_size"]
        bad_events.append(bad)
        for event in bad_events:
            with self.subTest(event=event), self.assertRaises(ValueError):
                HOOK.evaluate(frozen(state()), frozen([event]), frozen(PARAMETERS))

    def test_batch_and_separate_calls_replay_the_same_accepted_ledger(self):
        events = [release(1), crossing(), release(2), hit(2), hit(1), release(3)]
        together, separate = state(), state()
        apply(together, *events)
        for event in events:
            apply(separate, event)
        self.assertEqual(together["scores"], separate["scores"])
        def emitted(run):
            return [event for event in run["history"]
                    if event["type"] in ("shot_registered", "shot_result", "role_selected")]

        self.assertEqual(emitted(together), emitted(separate))


@unittest.skipUnless(os.environ.get("RP_SCORING_REFERENCE"),
                     "set RP_SCORING_REFERENCE to original behavior/scoring.py")
class OriginalScoringReferenceTests(unittest.TestCase):
    def test_torpedo_release_result_and_bonus_ledger_matches_original(self):
        original = module_at(Path(os.environ["RP_SCORING_REFERENCE"]), "original_shot_reference")
        traces = [
            [crossing(), release(1, .3048), release(2, .4572),
             hit(2, size="small"), hit(1), hit(1, outcome="blocked"), release(3), hit(3)],
            [release(1), hit(1), crossing(), hit(1), release(2), hit(2, size="small")],
            [crossing(.5), release(1), release(2), hit(1), hit(2, size="small")],
            [crossing(), release(1), hit(1, outcome="blocked"), hit(1),
             release(2, .3048 - 1e-9), hit(2, outcome="timeout"), hit(2)],
            [crossing(), release(1, .4572 - 1e-9), release(1, 999), release(2),
             hit(1, size="small"), hit(2)],
        ]
        for trace in traces:
            for intended in ("repair", "rescue"):
                ledger = original.RunScore()
                ledger.start(0, role=intended, heading_coin=True, role_coin=True)
                run = state(role=intended)
                for event in trace:
                    data = event["data"]
                    if event["type"] == "pass_through":
                        y = data["crossing_point_local"][1]
                        ledger.gate("repair" if y < 0 else "rescue", 1 if y > 0 else -1)
                    elif event["type"] == "payload_released":
                        ledger.release_payload("torpedo", data["projectile_id"],
                                               data["release_distance_m"])
                    else:
                        outcome = data["outcome"]
                        result = ("success" if data["hole_class"] == ledger.target_class
                                  else "wrong_target") if outcome == "pass" else outcome
                        ledger.payload_result("torpedo", data["projectile_id"], result,
                                              data["hole_id"], data["hole_class"], data["hole_size"])
                    apply(run, event)
                    for row in ("gate", "torpedoes", "distance", "sequence"):
                        self.assertEqual(run["scores"].get(row, 0), ledger.points[row],
                                         (intended, row, event))

    def test_gate_home_and_style_match_unchanged_runscore_and_coursejudge(self):
        original = module_at(Path(os.environ["RP_SCORING_REFERENCE"]), "original_score_reference")
        import numpy as np

        q = math.pi / 2
        for role in ("repair", "rescue"):
            for heading in (False, True):
                for coin in (False, True):
                    ledger = original.RunScore()
                    ledger.start(0, role=role, heading_coin=heading, role_coin=coin)
                    run = state(role=role, heading_coin=heading, role_coin=coin)
                    for index, (y, turns) in enumerate([
                            (-0.5, (q - 2e-6, 0, 0)), (0.5, (q - .5e-6, -q, q)),
                            (0.0, (7 * q, q, 8 * q)), (-0.5, (0, 0, 20 * q))], 1):
                        actual = "repair" if y < 0 else "rescue"
                        side = 1 if y > 0 else -1
                        ledger.gate(actual, side)
                        apply(run, crossing(y, index))
                        self.assertEqual(run["scores"]["gate"], ledger.points["gate"])
                        judge = object.__new__(original.CourseJudge)
                        judge.score = ledger
                        judge.gate_attempt = {"passed": True, "role": actual, "side": side,
                                              "turns": np.array(turns)}
                        judge.finish_gate_attempt()
                        apply(run, finished(turns, index))
                        self.assertEqual(run["scores"]["gate"], ledger.points["gate"])
                    ledger.award("home", ledger.rules["home"])
                    apply(run, crossing(reverse=True))
                    self.assertEqual(run["scores"]["home"], ledger.points["home"])
                    ledger.stop(1)
                    run["run"]["running"] = False
                    ledger.gate("rescue", 1, style=99999)
                    self.assertEqual(apply(run, finished((999, 0, 0)))["scores"], [])
                    self.assertEqual(run["scores"]["gate"], ledger.points["gate"])


if __name__ == "__main__":
    unittest.main()
