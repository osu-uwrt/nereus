"""Record every call of the Python robosub_2026 hook made by the reference tests.

Runs the scripted scenarios of the Python test-suite with the hook functions wrapped, and
writes the unique (function, inputs) -> output-or-error triples to rules_calls.json. The C++
port must reproduce each recorded output exactly (floats to 1e-9 relative).

    PYTHONPATH=python/src python3 extensions/rules/robosub_2026/tests/capture_fixture.py
"""
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).with_name("rules_calls.json")
TESTS = ["test_task_scoring_2026.py", "test_task_step4.py", "test_session_run_control.py",
         "test_task_runtime.py"]

from robotics_platform import task_runtime  # noqa: E402

records: dict[str, dict] = {}


def thaw(value):
    if hasattr(value, "items"):
        return {key: thaw(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw(item) for item in value]
    return value


HOOK = ROOT / "content/packs/tasks/robosub_2026/hooks/rules_2026.py"
namespace: dict = {"__name__": "rules_2026_capture"}
exec(compile(HOOK.read_text(), str(HOOK), "exec"), namespace)
CONTEXT = {"payloads": {0: {"mechanism_id": "a", "mechanism_type": "launcher", "slot": 0},
                        1: {"mechanism_id": "b", "mechanism_type": "dropper", "slot": 1},
                        2: {"mechanism_id": "c", "mechanism_type": "launcher", "slot": 0}}}


def extras(args, output):
    """Also record describe/feed on the states and event batches of every evaluate call."""
    state, events, parameters = args
    for name, call in (("describe", lambda: namespace["describe"](state, parameters)),
                       ("feed", lambda: namespace["feed"](state, events, CONTEXT, parameters))):
        if name == "feed" and not events:
            continue
        inputs = ([state, parameters] if name == "describe"
                  else [state, events, CONTEXT, parameters])
        entry = {"function": name, "inputs": [thaw(arg) for arg in inputs]}
        try:
            entry["output"] = json.loads(json.dumps(call(), allow_nan=False))
        except Exception as error:
            entry["error"] = type(error).__name__
        records.setdefault(json.dumps(entry, sort_keys=True), entry)


def wrap(name, callback):
    def recorded(*args):
        inputs = [thaw(arg) for arg in args]
        entry = {"function": name, "inputs": inputs}
        try:
            output = callback(*args)
        except Exception as error:  # ValueError/KeyError ... : the port must throw too
            entry["error"] = type(error).__name__
            key = json.dumps(entry, sort_keys=True)
            records.setdefault(key, entry)
            raise
        entry["output"] = json.loads(json.dumps(output, allow_nan=False))
        if name == "evaluate":
            extras(args, output)
        records.setdefault(json.dumps(entry, sort_keys=True), entry)
        return output
    return recorded


original = task_runtime.TaskRuntime.reset


def reset(self, *args, **kwargs):
    original(self, *args, **kwargs)
    def ours(source):  # other packs' test hooks are not this rules module
        return Path(source[0]).name == "rules_2026.py"

    sources = self._hook_sources
    self._hooks = [(wrap("evaluate", c), p) if ours(src) else (c, p)
                   for src, (c, p) in zip(sources, self._hooks)]
    status = iter(self._status_hooks)
    feed = iter(self._feed_hooks)
    self._status_hooks, self._feed_hooks = [], []
    for src in sources:
        if src[4] is not None:
            c, p = next(status)
            self._status_hooks.append((wrap("describe", c), p) if ours(src) else (c, p))
        if src[5] is not None:
            c, p = next(feed)
            self._feed_hooks.append((wrap("feed", c), p) if ours(src) else (c, p))


task_runtime.TaskRuntime.reset = reset
MUTATIONS = [
    ("payload_released", "projectile_id", -1), ("payload_released", "projectile_id", 1.5),
    ("payload_released", "projectile_id", True), ("payload_released", "release_distance_m", -0.1),
    ("payload_released", "release_distance_m", "far"), ("hit", "outcome", "bogus"),
    ("hit", "outcome", "blocked"), ("hit", "outcome", "timeout"), ("hit", "hole_id", ""),
    ("hit", "hole_size", None), ("hit", "projectile_id", None),
]


def mutate():
    """Malformed launcher events built from recorded ones: the hook must raise or agree."""
    evaluate = namespace["evaluate"]
    seen = set()
    for entry in list(records.values()):
        if entry["function"] != "evaluate":
            continue
        state, events, parameters = entry["inputs"]
        for index, event in enumerate(events):
            for kind, key, value in MUTATIONS:
                if event["type"] != kind or event["task"] != "torpedo" or key not in event["data"]:
                    continue
                variant = json.loads(json.dumps(events))
                variant[index]["data"][key] = value
                marker = json.dumps([state, variant], sort_keys=True)
                if marker in seen or len(seen) > 60:
                    continue
                seen.add(marker)
                item = {"function": "evaluate", "inputs": [state, variant, parameters]}
                try:
                    item["output"] = json.loads(json.dumps(evaluate(state, variant, parameters)))
                except Exception as error:
                    item["error"] = type(error).__name__
                records.setdefault(json.dumps(item, sort_keys=True), item)


def scripted_torpedoes():
    """Torpedo rows/sequence/distance and gate style, driven through the hook directly."""
    base = next(e for e in records.values() if e["function"] == "evaluate")["inputs"]
    parameters = base[2]
    state = json.loads(json.dumps(base[0]))
    state["history"], state["scores"] = [], {}
    state["run"].update(running=True, ended=False, started_ns=0)
    evaluate = namespace["evaluate"]
    rot = {"rotation_vector_body": [1.6, 0.0, 3.2]}

    def ev(kind, identifier, region, task, time_ns, data):
        return {"id": identifier, "type": kind, "task": task, "region": region,
                "time_ns": time_ns, "data": data}

    def hit(time_ns, projectile, outcome, hole_class="fire", size="large"):
        data = {"mechanism_type": "launcher", "projectile_id": projectile, "outcome": outcome}
        if outcome == "pass":
            data.update(hole_id="h" + size, hole_class=hole_class, hole_size=size)
        return ev("hit", "hole_pass" if outcome == "pass" else "panel_blocked", "panel",
                  "torpedo", time_ns, data)

    def release(time_ns, projectile, distance):
        return ev("payload_released", "payload_released", "", "torpedo", time_ns,
                  {"mechanism_type": "launcher", "projectile_id": projectile,
                   "release_distance_m": distance})

    def gate(time_ns, y, attempt=1):
        return ev("pass_through", "forward_pass", "gate_opening", "gate", time_ns,
                  {"attempt_id": attempt, "from_side": "positive", "to_side": "negative",
                   "crossing_point_local": [0.0, y, 0.5]})

    scripts = [
        [[release(1, 0, 0.5)], [hit(2, 0, "pass")]],                       # no role: ineligible
        [[gate(1, -0.5)], [release(2, 0, 0.5), release(3, 1, 0.35), release(4, 2, 1.0)],
         [hit(5, 0, "pass", "fire", "large"), hit(6, 1, "pass", "fire", "small")],
         [ev("attempt_finished", "gate_opening:attempt_finished", "gate_opening", "gate", 7,
             {"attempt_id": 1, **rot})]],
        [[gate(1, 0.5)], [release(2, 0, 0.1), release(3, 1, 0.2)],
         [hit(4, 0, "pass", "fire", "large"), hit(5, 1, "blocked")],
         [hit(6, 0, "pass", "blood", "large")]],
        [[gate(1, -0.5)], [release(2, 0, 0.5), release(3, 1, 0.5)],
         [hit(4, 0, "pass", "blood", "small"), hit(5, 1, "timeout")]],
        [[gate(1, -0.5)], [release(2, 0, 0.5), release(3, 1, 0.5)],
         [hit(4, 0, "pass", "fire", "small"), hit(5, 1, "pass", "fire", "large")]],
    ]
    for script in scripts:
        run_state = json.loads(json.dumps(state))
        for batch in script:
            args = (run_state, batch, parameters)
            output = evaluate(*json.loads(json.dumps(args)))
            item = {"function": "evaluate", "inputs": json.loads(json.dumps(args)),
                    "output": json.loads(json.dumps(output))}
            records.setdefault(json.dumps(item, sort_keys=True), item)
            extras(args, output)
            run_state["history"] += batch + output["events"]
            for row in output["scores"]:
                run_state["scores"][row["row"]] = row["points"]


code = pytest.main(["-q", "-p", "no:cacheprovider", *[str(ROOT / "tests/python" / t) for t in TESTS]])
scripted_torpedoes()
mutate()
calls = sorted(records.values(), key=lambda e: json.dumps(e, sort_keys=True))
OUT.write_text(json.dumps({"calls": calls}, sort_keys=True, separators=(",", ":")) + "\n")
print(f"{len(calls)} unique calls, pytest exit {int(code)}", file=sys.stderr)
