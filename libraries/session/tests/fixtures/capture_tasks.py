"""Drive the Python TaskRuntime (scoring hooks removed) on scripted inputs and write the
deterministic fixture `task_runtime_capture.json` that task_runtime_test.cpp replays.

    PYTHONPATH=python/src python3 libraries/session/tests/fixtures/capture_tasks.py

Every case records its ordered ops (inputs) and the Python results; the C++ test replays the
inputs and compares events (exact for discrete fields, tight tolerance for floats) and the final
runtime snapshot. Rule outputs are excluded by construction: the hooks are removed here and the
C++ test registers a no-op Rules.
"""

import copy
import json
import math
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from robotics_platform.packs import resolve_scenario
from robotics_platform.task_runtime import TaskRuntime, _placement, _pose

ROOT = Path(__file__).resolve().parents[4]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt/scenario.yaml"
OUT = Path(__file__).with_name("task_runtime_capture.json")
MS = 1_000_000
STEP = 100 * MS


def plain(value):
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    if isinstance(value, np.ndarray):
        return [plain(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    return value


def quat_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2),
                              math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2))
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


class Case:
    def __init__(self, resolved, name, task_ids):
        self.name, self.task_ids = name, task_ids
        self.rt = TaskRuntime(copy.deepcopy(resolved), task_ids=task_ids)
        self.rt._hook_sources = []  # scoring is the Rules implementation's job, not compared here
        self.rt.reset()
        self.place = {item["task"]: _placement(item) for item in resolved.scenario["task_placements"]}
        self.ops, self.results, self.snapshots = [], [], []

    def world(self, task, local, quat=(1, 0, 0, 0)):
        pose = self.place[task].compose(_pose(local, quat))
        return [float(x) for x in pose.translation], [float(x) for x in pose.orientation_wxyz]

    def vec(self, task, local):  # point in task frame -> world
        return [float(x) for x in self.place[task].apply(np.asarray(local, float))]

    def axis(self, task, local):
        rot = _pose([0, 0, 0], self.place[task].orientation_wxyz)
        return [float(x) for x in rot.apply(np.asarray(local, float))]

    def add(self, op, result):
        self.ops.append(op)
        self.results.append(plain(result))
        return result

    def observe(self, t, position, wxyz):
        pose = _pose(position, wxyz)
        events = self.rt.observe(t, pose)
        return self.add({"op": "observe", "t": t, "position": position, "wxyz": wxyz},
                        {"events": events})

    def observe_local(self, task, t, local, quat=(1, 0, 0, 0)):
        position, wxyz = self.world(task, local, quat)
        return self.observe(t, position, wxyz)

    def release(self, t, ident, mech, tip, radius, length):
        events = self.rt.release_projectile(t, ident, mech, tip, radius, length)
        return self.add({"op": "release", "t": t, "id": ident, "mech": mech, "tip": tip,
                         "radius": radius, "length": length}, {"events": events})

    def step(self, t, ident, start, end, axis, velocity):
        result = self.rt.step_projectile(t, ident, start, end, axis, velocity)
        return self.add({"op": "step", "t": t, "id": ident, "start": start, "end": end,
                         "axis": axis, "velocity": velocity},
                        {"events": result.events, "stop": result.stop,
                         "position": result.position_world, "velocity": result.velocity_world})

    def record(self, t, events):
        return self.add({"op": "record", "t": t, "events": events},
                        {"events": self.rt.record(t, events)})

    def start(self, t, options=None):
        self.rt.start(t, options)
        return self.add({"op": "start", "t": t, "options": options or {}}, {})

    def stop(self, t):
        return self.add({"op": "stop", "t": t}, {"events": self.rt.stop(t)})

    def reset(self, t):
        self.rt.reset(t)
        return self.add({"op": "reset", "t": t}, {})

    def checkpoint(self):
        self.snapshots.append(len(self.ops))
        return None

    def finish(self):
        return {"name": self.name, "task_ids": self.task_ids, "ops": self.ops,
                "results": self.results, "snapshot": plain(self.rt.snapshot()),
                "indicators": plain(self.rt.indicators()), "describe": plain(self.rt.describe()),
                "event_count": sum(len(r.get("events", [])) for r in self.results)}


def gate_case(resolved):
    c = Case(resolved, "gate", ["gate"])
    t = 0
    c.start(t, {"role": "rescue"})
    # forward pass with a roll/yaw spin near the plane, then the reverse pass
    xs = [x / 4 for x in range(16, -17, -1)] + [x / 4 for x in range(-15, 17)]
    for i, x in enumerate(xs):
        t += STEP
        c.observe_local("gate", t, [x, -0.75, -0.2], quat_rpy(roll=0.35 * i, yaw=0.05 * i))
    c.checkpoint()
    # outside the width: the crossing does not fit (no pass event)
    for x in [4, 2, 1, .5, 0, -.5, -1, -2, -4]:
        t += STEP
        c.observe_local("gate", t, [x, 1.45, -0.2])
    # too low: envelope under the pool floor
    for x in [-3, -2, -1, -.5, 0, .5, 1, 2, 3]:
        t += STEP
        c.observe_local("gate", t, [x, 0.0, -1.5])
    # a teleport ends the attempt; one more approach; explicit stop finishes the open attempt
    t += STEP
    c.observe_local("gate", t, [30, 0, -.2])
    for x in [2.0, 1, .5, 0, -.5, -1, -2]:
        t += STEP
        c.observe_local("gate", t, [x, 0.4, -.2], quat_rpy(pitch=0.2))
    t += STEP
    c.stop(t)
    c.reset(t)
    for x in [3, 2, 1, .5, 0, -.5, -1]:
        t += STEP
        c.observe_local("gate", t, [x, -0.75, -0.2])
    return c.finish()


def slalom_case(resolved):
    c = Case(resolved, "slalom", ["slalom"])
    t = 0
    c.start(t)
    path = [(1.5, 0.0), (0.5, 0.0), (0.0, 0.0), (-0.5, 0.2), (-1.5, 0.5), (-2.127, 0.694),
            (-2.6, 0.5), (-3.5, 0.3), (-4.232, 0.118), (-4.8, 0.0), (-4.0, 0.1), (-2.7, 0.6),
            (-2.127, 0.694), (-1.0, 0.3), (0.5, 0.0), (1.0, 0.0)]
    dense = []
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        for k in range(4):
            f = k / 4
            dense.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    for x, y in dense:
        t += STEP
        c.observe_local("slalom", t, [x, y, -0.3], quat_rpy(yaw=0.1 * x))
    # outside the 1.55 m half width: crossing does not count
    for x in [1, .5, 0, -.5, -1]:
        t += STEP
        c.observe_local("slalom", t, [x, 1.6, -0.3])
    t += STEP
    c.stop(t)
    return c.finish()


def torpedo_case(resolved):
    c = Case(resolved, "torpedo", ["torpedo"])
    t = 0
    c.start(t)
    T = "torpedo"
    radius, length = 0.03, 0.12

    def shoot(ident, mech, y, z, axis_local, t0, speed=3.0):
        nonlocal t
        t = t0
        c.release(t, ident, mech, c.vec(T, [0.8, y, z]), radius, length)
        axis = c.axis(T, axis_local)
        velocity = [speed * a for a in axis]
        pos = [0.6, y, z]
        for _ in range(12):
            start = c.vec(T, pos)
            pos = [pos[0] + axis_local[0] * 0.1, pos[1] + axis_local[1] * 0.1,
                   pos[2] + axis_local[2] * 0.1]
            t += 10 * MS
            c.step(t, ident, start, c.vec(T, pos), axis, velocity)
        return t

    shoot(1, "launcher", -0.206, 0.0543, [-1, 0, 0], 100 * MS)        # through fire_large
    shoot(2, "launcher", 0.20, -0.2, [-1, 0, 0], 300 * MS)            # blocked on the panel
    shoot(3, "launcher", 0.5, 0.0, [-1, 0, 0], 500 * MS)              # misses the panel plane
    s = math.sqrt(0.5)
    shoot(4, "launcher", 0.5137, 0.2, [-s, s, 0], 700 * MS)           # oblique near small hole
    shoot(5, "launcher", 0.0, 0.0543, [-0.2, 0, -0.9797959], 900 * MS)  # grazing, cosine clamp
    shoot(6, "dropper", -0.206, 0.0543, [-1, 0, 0], 1100 * MS)        # no dropper binding
    # a stopped projectile: further steps are inert
    c.step(t + 10 * MS, 2, c.vec(T, [0.5, 0.2, -0.2]), c.vec(T, [-0.5, 0.2, -0.2]),
           c.axis(T, [-1, 0, 0]), [1, 0, 0])
    # sinks to the floor: pool_floor miss for a payload that never hit the panel
    t += 100 * MS
    c.release(t, 7, "launcher", c.vec(T, [3, 3, 0]), radius, length)
    for k in range(6):
        start = c.vec(T, [3, 3, -0.2 * k])
        t += 10 * MS
        c.step(t, 7, start, c.vec(T, [3, 3, -0.2 * (k + 1)]), [0, 0, -1.0], [0, 0, -1.0])
    # timeout in open water
    t += 100 * MS
    c.release(t, 8, "launcher", c.vec(T, [3, 3, 1]), radius, length)
    c.step(t + 31_000 * MS, 8, c.vec(T, [3, 3, 1]), c.vec(T, [3.1, 3, 1]), [1.0, 0, 0], [1.0, 0, 0])
    c.step(t + 31_010 * MS, 8, c.vec(T, [3, 3, 1]), c.vec(T, [3.1, 3, 1]), [1.0, 0, 0], [1.0, 0, 0])
    return c.finish()


def bins_case(resolved):
    c = Case(resolved, "bins", ["bins"])
    t = 0
    c.start(t)
    B = "bins"
    frames = {f["id"]: f["position_m"] for f in resolved.task_definitions
              if f["id"] == "bins" for f in f["frames"]}
    frames = {f["id"]: f["position_m"] for task in resolved.task_definitions if task["id"] == "bins"
              for f in task["frames"]}
    ident = [0]

    def drop(mech, crate, dx, dy, radius=0.02, length=0.08, z0=0.9, down=-0.06, axis=(0, 0, 1.0),
             velocity=(0, 0, -1.5)):
        nonlocal t
        ident[0] += 1
        i = ident[0]
        fx, fy, fz = frames[crate]
        t += 50 * MS
        c.release(t, i, mech, c.vec(B, [fx + dx, fy + dy, fz + z0]), radius, length)
        z = z0
        pos = [fx + dx, fy + dy, fz + z0]
        while z > -0.4:
            start = c.vec(B, pos)
            z += down
            pos = [fx + dx, fy + dy, fz + z]
            t += 10 * MS
            r = c.step(t, i, start, c.vec(B, pos), list(axis), list(velocity))
            if r.stop:
                break
        return i

    drop("dropper", "bin_vinyl1", 0.0, 0.0)               # inside
    drop("launcher", "bin_vinyl2", 0.0, 0.14, radius=0.03)  # rim
    drop("dropper", "bin_vinyl3", 0.5, 0.5)               # beside: pool floor miss
    drop("dropper", "bin_vinyl4", 0.0, 0.0, axis=(0.6, 0, 0.8), length=0.2)  # tilted, long
    drop("launcher", "bin_vinyl1", 0.0, 0.0, z0=0.5, down=-0.5)  # coarse steps
    # horizontal flight into a crate wall, then fall (owner applies the correction)
    ident[0] += 1
    i = ident[0]
    fx, fy, fz = frames["bin_vinyl1"]
    t += 50 * MS
    pos = [fx + 0.5, fy + 0.02, fz + 0.1]
    vel = c.axis(B, [-2.0, 0, 0])
    c.release(t, i, "launcher", c.vec(B, pos), 0.02, 0.08)
    for _ in range(14):
        start_local = list(pos)
        pos = [pos[0] - 0.05, pos[1], pos[2]]
        t += 10 * MS
        r = c.step(t, i, c.vec(B, start_local), c.vec(B, pos), [0, 0, 1.0], vel)
        if r.position_world is not None:
            pos = list(np.linalg.solve(np.array(c.place[B].orientation_wxyz and
                                                _pose([0, 0, 0], c.place[B].orientation_wxyz)
                                                .apply(np.eye(3)[k]) for k in range(3)).T
                                       if False else np.eye(3), np.zeros(3)))
            break
    # free flight into the crate top rim from the side with wall entry
    drop("launcher", "bin_vinyl3", 0.16, 0.0, radius=0.03)
    # timeout release far above
    t += 50 * MS
    ident[0] += 1
    i = ident[0]
    c.release(t, i, "dropper", c.vec(B, [0, 0, 2.0]), 0.02, 0.08)
    c.step(t + 30_500 * MS, i, c.vec(B, [0, 0, 2.0]), c.vec(B, [0, 0, 2.0]), [0, 0, 1.0], [0, 0, 0])
    t += 30_600 * MS
    # magnet targets: dwell, leave, dwell again
    for key in ("magnet_target1", "magnet_target2"):
        target = c.rt._targets[(B, key)]
        base = np.asarray(target._sensor) - np.asarray(target._probe)
        seq = [(0.0, 5), (0.5, 3), (0.0, 3), (0.0, 8)] if key == "magnet_target1" else [(0.0, 4)]
        for offset, n in seq:
            for _ in range(n):
                t += 100 * MS
                pos = [float(base[0] + offset), float(base[1]), float(base[2])]
                c.observe(t, pos, [1.0, 0.0, 0.0, 0.0])
    c.checkpoint()
    return c.finish()


def surface_case(resolved, breach):
    name = "surface_breach" if breach else "surface"
    c = Case(resolved, name, ["surface", "table"])
    t = 0
    c.start(t)
    S = "surface"
    surf = c.rt._surfaces[(S, "octagon")]

    def facing_yaw(target, position):
        p = np.asarray(surf._targets[target])[:2] - np.asarray(position)[:2]
        return math.atan2(p[1], p[0])

    base = c.vec(S, [0, 0, 0])
    x0, y0 = (base[0] + (3.0 if breach else 0.0)), base[1]
    yaw = facing_yaw("compass", [x0, y0])
    for z in [-1.0, -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.1, 0.3]:
        t += STEP
        c.observe(t, [x0, y0, z], quat_rpy(yaw=yaw))
    if breach:
        for _ in range(3):
            t += STEP
            c.observe(t, [x0, y0, 0.3], quat_rpy(yaw=yaw))
        return c.finish()
    for _ in range(14):
        t += STEP
        c.observe(t, [x0, y0, 0.3], quat_rpy(yaw=yaw))
    yaw = facing_yaw("buoy", [x0, y0])
    for _ in range(9):
        t += STEP
        c.observe(t, [x0, y0, 0.3], quat_rpy(yaw=yaw))
    yaw = facing_yaw("sos", [x0, y0]) + 0.8        # not facing any target
    for _ in range(3):
        t += STEP
        c.observe(t, [x0, y0, 0.3], quat_rpy(yaw=yaw))
    yaw = facing_yaw("sos", [x0, y0])
    for _ in range(8):
        t += STEP
        c.observe(t, [x0, y0, 0.3], quat_rpy(yaw=yaw))
    for z in [0.1, -0.2, -0.6, -1.0]:              # dive: surface_lost + facing_lost
        t += STEP
        c.observe(t, [x0, y0, z], quat_rpy(yaw=yaw))
    c.checkpoint()
    # rise again outside the octagon but low: no breach; then teleport
    t += STEP
    c.observe(t, [x0 + 30, y0, -1.0], quat_rpy(yaw=yaw))
    # turn zone: spin, hold, reverse, restart via a table event, leave gradually
    tz = c.rt._turns[(S, "turn_zone")]
    centre = np.asarray(c.place[S].apply(np.zeros(3)))  # unused anchor; use the zone frame
    frame = [f for task in resolved.task_definitions if task["id"] == S for f in task["frames"]
             if f["id"] == "table_origin"][0]
    zone = np.array(c.vec(S, frame["position_m"]))
    here = [float(zone[0] + 0.5), float(zone[1]), float(zone[2] + 0.3)]
    heading = 0.0
    for k in range(22):
        t += STEP
        heading += math.radians(40)
        c.observe(t, here, quat_rpy(yaw=heading))
    for _ in range(12):
        t += STEP
        c.observe(t, here, quat_rpy(yaw=heading))
    for _ in range(8):
        t += STEP
        heading -= math.radians(40)
        c.observe(t, here, quat_rpy(yaw=heading))
    c.record(t, [{"id": "basket_drop", "type": "drop_into", "task": "table", "region": "",
                  "time_ns": t, "data": {"prop_id": "pill", "basket": "helmet_basket",
                                         "expected_basket": "helmet_basket"}}])
    for _ in range(9):
        t += STEP
        heading += math.radians(40)
        c.observe(t, here, quat_rpy(yaw=heading))
    for k in range(9):
        t += STEP
        c.observe(t, [here[0] + 0.5 * (k + 1), here[1], here[2]], quat_rpy(yaw=heading))
    c.stop(t + STEP)
    return c.finish()


def table_case(resolved):
    c = Case(resolved, "table", ["table"])
    t = 0
    c.start(t)
    ev = [("grasp", "attach", None, {"prop_id": "pill", "mechanism_id": "claw"}),
          ("release", "detach", "", {"prop_id": "pill", "mechanism_id": "claw", "reason": "opened"}),
          ("basket_drop", "drop_into", "", {"prop_id": "pill", "basket": "warning_basket",
                                            "expected_basket": "warning_basket"}),
          ("object_dropped", "drop_into", "", {"prop_id": "plug"})]
    for identifier, kind, region, data in ev:
        t += STEP
        c.record(t, [{"id": identifier, "type": kind, "task": "table", "region": region,
                      "time_ns": t, "data": data}])
    t += STEP
    c.record(t, [])
    c.stop(t + STEP)
    return c.finish()


def main():
    resolved = resolve_scenario(SCENARIO)
    cases = [gate_case(resolved), slalom_case(resolved), torpedo_case(resolved),
             bins_case(resolved), surface_case(resolved, False), surface_case(resolved, True),
             table_case(resolved)]
    OUT.write_text(json.dumps({"cases": cases}, sort_keys=True, separators=(",", ":")) + "\n")
    for case in cases:
        kinds = {}
        for r in case["results"]:
            for e in r.get("events", []):
                kinds[e["type"]] = kinds.get(e["type"], 0) + 1
        print(case["name"], len(case["ops"]), kinds)


if __name__ == "__main__":
    main()
