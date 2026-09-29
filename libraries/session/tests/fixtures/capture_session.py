"""Drive the Python reference (pack_runtime, mechanisms, session with tasks disabled, run_snapshot)
on scripted inputs and write deterministic JSON fixtures next to this file.

    PYTHONPATH=python/src python libraries/session/tests/fixtures/capture_session.py

The fixtures embed the scripts, so the C++ gtests replay exactly what was captured.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from robotics_platform import _native as native
from robotics_platform.mechanisms import Mechanisms
from robotics_platform.pack_runtime import create_runtime
from robotics_platform.packs import resolve_scenario
from robotics_platform.session import Session

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt"


def norm(value):
    """JSON-normalise task documents (frozen mappings, tuples, numpy scalars)."""
    from collections.abc import Mapping

    def default(o):
        if isinstance(o, Mapping):
            return dict(o)
        if isinstance(o, np.generic):
            return o.item()
        return list(o)

    return json.loads(json.dumps(value, default=default, allow_nan=False))


def f(values) -> list[float]:
    return [float(v) for v in np.asarray(values, float).ravel()]


def body(state) -> dict:
    return {"position": f(state.position), "orientation": f(state.orientation_wxyz),
            "linear_velocity": f(state.linear_velocity),
            "angular_velocity": f(state.angular_velocity)}


def sensor_values(kind: str, reading) -> dict:
    if kind == "ahrs":
        return {"specific_force": f(reading.inertial.specific_force),
                "angular_velocity": f(reading.inertial.angular_velocity),
                "force_covariance": f(reading.inertial.force_covariance),
                "angular_covariance": f(reading.inertial.angular_covariance),
                "attitude": f(reading.attitude.orientation_wxyz),
                "attitude_covariance": f(reading.attitude.covariance)}
    if kind == "fog":
        return {"angular_rates": f(reading.angular_rates), "covariance": f(reading.covariance)}
    if kind == "reference_velocity":
        return {"velocity": f(reading.reference_relative_velocity), "covariance": f(reading.covariance)}
    if kind == "reference_altitude":
        return {"mounted_world_z": reading.mounted_world_z, "target_world_z": reading.target_world_z,
                "variance": reading.variance}
    raise ValueError(kind)


def sensors(pack) -> dict:
    out = {}
    for name, stream in pack.streams.items():
        sample = stream.latest()
        kind = next(s["type"] for s in RESOLVED.robot["sensors"] if s["id"] == name)
        out[name] = None if sample is None or sample.value is None else {
            "sequence": sample.header.sequence, "acquired_ns": sample.header.acquired_ns,
            "stats": stream.stats.acquired, **sensor_values(kind, sample.value)}
    return out


def drain(pack) -> None:
    for stream in pack.streams.values():
        stream.drain()


RESOLVED = resolve_scenario(SCENARIO)
SENSORS = [s["id"] for s in RESOLVED.robot["sensors"] if s["type"] != "stereo_camera"]


def capture_pack_runtime() -> dict:
    pack = create_runtime(RESOLVED, sensor_ids=SENSORS)
    p = pack.parameters
    box = lambda b: {"id": b.id, "size": f(b.size), "center": f(b.center),  # noqa: E731
                     "orientation": f(b.orientation_wxyz)}
    plant = {
        "mass": p.body.mass, "inertia": f(p.body.inertia), "added_mass": f(p.body.added_mass),
        "linear_damping": f(p.body.linear_damping),
        "quadratic_damping": f(p.body.quadratic_damping),
        "damping_center": f(p.body.damping_center), "displaced_volume": p.body.displaced_volume,
        "buoyancy_center": f(p.body.buoyancy_center), "buoyancy_radii": f(p.body.buoyancy_radii),
        "command_timeout": p.command_timeout, "timestep_ns": p.timestep_ns,
        "pool": {"origin_xy": f(p.pool.origin_xy_world), "yaw": p.pool.yaw_world,
                 "length": p.pool.length, "width": p.pool.width, "depth": p.pool.depth,
                 "water_level": p.pool.water_level, "water_density": p.pool.water_density,
                 "current": f(p.pool.current_velocity),
                 "amplitude": f(p.pool.current_oscillation_amplitude),
                 "frequency": p.pool.current_oscillation_frequency},
        "thrusters": [{"id": t.id, "position": f(t.position), "direction": f(t.direction),
                       "delay": t.delay, "rise_time": t.rise_time, "fall_time": t.fall_time,
                       "slew_rate": t.slew_rate, "forward_limit": t.forward_limit,
                       "reverse_limit": t.reverse_limit, "deadband": t.deadband,
                       "forward_scale": t.forward_scale, "reverse_scale": t.reverse_scale,
                       "efficiency": t.efficiency, "propeller_radius": t.propeller_radius}
                      for t in p.thrusters],
        "contacts": {"model": str(p.contacts.model), "restitution": p.contacts.restitution,
                     "friction": p.contacts.friction,
                     "body_boxes": [box(b) for b in p.contacts.body_boxes],
                     "world_boxes": [box(b) for b in p.contacts.world_boxes]},
    }
    frames = {name: {"translation": f(pack.frames.from_root(name).translation),
                     "orientation": f(pack.frames.from_root(name).orientation_wxyz)}
              for name in ("base_link", "imu_mount", "claw_pinch", "torpedoes_mount")}
    forces = [[3.0, 3.0, 3.0, 3.0, 1.0, -1.0, 2.0, -2.0], [0.0] * 8, [8.0, -6.0, 4.0, 2.0, 0.0, 0.0, 5.0, 1.0]]
    checkpoints = []
    for tick in range(1, 1501):
        if tick in (1, 501, 1001):
            pack.runtime.command(np.asarray(forces[(tick - 1) // 500], float))
        snapshot = pack.runtime.advance(1)
        drain_now = tick % 100 == 0
        if drain_now:
            checkpoints.append({"tick": tick, "elapsed_ns": snapshot.elapsed_ns,
                                "body": body(snapshot.body), "forces": f(snapshot.thruster_forces),
                                "sensors": sensors(pack)})
        drain(pack)
    return {"plant": plant, "frames": frames, "initial": body(pack.initial), "seed": RESOLVED.scenario["seed"],
            "sensor_ids": SENSORS, "deferred": list(pack.deferred_sensor_ids),
            "forces": forces, "checkpoints": checkpoints}


def result(r) -> dict:
    out = {"accepted": r.accepted, "message": r.message}
    release = getattr(r, "release", None)
    if release is not None:
        out["release"] = {"slot_id": release.slot_id, "slot_index": release.slot_index,
                          "time_ns": release.time_ns, "position": f(release.position_world_m),
                          "orientation": f(release.orientation_wxyz),
                          "velocity": f(release.velocity_com_world_m_s),
                          "angular_velocity": f(release.angular_velocity_world_rad_s)}
    return out


def mech_state(state) -> dict:
    return {"time_ns": state.time_ns, "armed": state.armed, "any_busy": state.any_busy,
            "releases": {k: {"state": v.state, "available": v.available} for k, v in state.releases.items()},
            "claws": {k: {"state": v.state, "gap_m": v.gap_m, "target_gap_m": v.target_gap_m,
                          "joints": list(v.joint_positions_m)} for k, v in state.claws.items()}}


def capture_mechanisms() -> dict:
    m = Mechanisms(RESOLVED.robot)
    pose = native.Pose()
    pose.translation = [1.0, -2.0, -1.5]
    q = [math.cos(0.3), 0.0, 0.0, math.sin(0.3)]
    pose.orientation_wxyz = q
    lin, ang = [0.3, -0.1, 0.05], [0.02, -0.03, 0.4]
    ops = [
        ["snapshot"], ["arm", True], ["arm", False], ["fire", "torpedo_launcher"], ["claw", "claw", True],
        ["unkill"], ["arm", True], ["snapshot"], ["fire", "torpedo_launcher"], ["fire", "dropper"],
        ["advance", 100], ["fire", "dropper"], ["advance", 150], ["fire", "dropper"], ["fire", "torpedo_launcher"],
        ["advance", 300], ["fire", "torpedo_launcher"], ["fire", "torpedo_launcher"], ["fire", "dropper"],
        ["claw", "claw", True], ["advance", 100], ["snapshot"], ["advance", 4000], ["snapshot"],
        ["move_claw", "claw", -0.5], ["advance", 100], ["snapshot"], ["move_claw", "claw", 0.0],
        ["advance", 10], ["move_claw", "claw", 1.5], ["advance", 250], ["snapshot"], ["claw", "claw", False],
        ["advance", 5], ["kill"], ["snapshot"], ["arm", True], ["advance", 20], ["unkill"], ["snapshot"],
        ["reload"], ["snapshot"], ["arm", True], ["fire", "torpedo_launcher"], ["advance", 1],
        ["reset"], ["snapshot"], ["unkill"], ["arm", True], ["fire", "dropper"],
    ]
    log = []
    killed = True
    dt = 2_000_000
    for op in ops:
        name = op[0]
        entry = {}
        if name == "snapshot":
            pass
        elif name == "arm":
            entry["result"] = result(m.set_armed(op[1], killed=killed))
        elif name == "fire":
            entry["result"] = result(m.fire(
                op[1], pose, lin, ang, reference_frame="base_link", water_density_kg_m3=998.2, killed=killed))
        elif name == "claw":
            entry["result"] = result(m.command_claw(op[1], op[2], killed=killed))
        elif name == "move_claw":
            entry["result"] = result(m.move_claw(op[1], op[2], killed=killed))
        elif name == "advance":
            for _ in range(op[1]):
                m.advance(dt, killed=killed)
        elif name in ("kill", "unkill"):
            killed = name == "kill"
            m.advance(0, killed=killed)
        elif name == "reload":
            entry["result"] = result(m.reload_all(killed=killed))
        elif name == "reset":
            m.reset(killed=killed)
        entry["state"] = mech_state(m.snapshot(killed=killed))
        log.append(entry)
    mounts = {k: [{"translation": f(m.slot_mount(k, i).translation),
                   "orientation": f(m.slot_mount(k, i).orientation_wxyz)}
                  for i in range(m.slot_count(k))] for k in ("torpedo_launcher", "dropper")}
    return {"pose": {"translation": f(pose.translation), "orientation": f(pose.orientation_wxyz)},
            "linear": lin, "angular": ang, "dt_ns": dt, "ops": ops, "log": log, "mounts": mounts}


SESSION_OPS = [
    ["advance", 50], ["arm", True], ["unkill"], ["arm", True],
    ["thrusters", [4.0, 4.0, 4.0, 4.0, 2.0, -2.0, 3.0, -3.0]], ["advance", 250],
    ["fire", "torpedo_launcher"], ["advance", 5], ["fire", "torpedo_launcher"], ["advance", 250],
    ["fire", "torpedo_launcher"], ["fire", "torpedo_launcher"], ["fire", "dropper"], ["advance", 600],
    ["claw", "claw", True], ["advance", 100], ["move_claw", "claw", -0.3], ["advance", 200],
    ["move_claw", "claw", 0.2], ["advance", 100], ["claw", "nope", True], ["fire", "nope"],
    ["kill"], ["advance", 50], ["unkill"], ["advance", 50],
    ["place", [2.0, 1.0, -1.2], [math.cos(0.4), 0.0, 0.0, math.sin(0.4)], True], ["advance", 50],
    ["reload"], ["advance", 10], ["reset_tasks"], ["advance", 50], ["run_start"], ["run_stop"],
    ["run_adjust", 3.0], ["full_reset", 11], ["advance", 100], ["unkill"], ["arm", True],
    ["thrusters", [1.0, 2.0, 3.0, 4.0, 0.0, 0.0, -1.0, 1.0]], ["fire", "dropper"], ["advance", 200],
    ["full_reset", None], ["advance", 100],
]


TASK_IDS = ["gate", "torpedo", "slalom"]  # no contact_world task: the prop world is separate
Q0 = [1.0, 0.0, 0.0, 0.0]
TASK_OPS = [
    ["run_start", {"role": "rescue"}], ["advance", 50],
    ["place_moving", [4.6, -2.9, -0.75], Q0, [2.5, 0.0, 0.0]], ["advance", 250],
    ["run_start", {}], ["run_start", None], ["run_adjust", 5.0],
    ["place_moving", [17.6, 2.48, -1.37], Q0, [0.0, 0.0, 0.0]], ["unkill"], ["arm", True],
    ["fire", "torpedo_launcher"], ["advance", 250], ["fire", "dropper"], ["advance", 500],
    ["fire", "torpedo_launcher"], ["advance", 250], ["run_stop"], ["advance", 10],
    ["run_start", {"bogus": 1}], ["run_start", {"role": "x"}], ["run_start", {"heading_coin": "yes"}],
    ["reset_tasks"], ["advance", 20], ["run_stop"], ["run_start", {"role": "repair", "heading_coin": False}],
    ["advance", 30], ["full_reset", None], ["advance", 20],
]


class FakeTasks:
    """Stands in for TaskRuntime to capture the run_snapshot document builder."""

    def __init__(self, snapshot, extra):
        self._s, self._e = snapshot, extra

    def snapshot(self):
        return self._s

    def describe(self):
        return dict(self._e)


def capture_session(ops=None, task_ids=None, tasks=False) -> dict:
    ops = SESSION_OPS if ops is None else ops
    pack = create_runtime(RESOLVED, sensor_ids=SENSORS)
    s = Session(RESOLVED, pack, task_ids=task_ids, tasks=tasks)
    dt = s.timestep_ns
    log = []
    tick = 0

    def checkpoint() -> dict:
        snapshot = s.last_step.snapshot
        state = s.mechanism_state()
        extra = {"run": norm(s.run_snapshot()), "indicators": norm(s.indicators())} if tasks else {}
        return {**extra, "time_ns": s.time_ns, "killed": s.killed, "body": body(snapshot.body),
                "forces": f(s.thruster_forces()),
                "payloads": [{"id": p.identifier, "mechanism_id": p.mechanism_id,
                              "mechanism_type": p.mechanism_type, "active": p.active,
                              "outcome": p.outcome, "released_ns": p.released_ns,
                              "position": f(p.state.position), "orientation": f(p.state.orientation_wxyz),
                              "velocity": f(p.state.velocity),
                              "angular_velocity": f(p.state.angular_velocity)}
                             for p in s.payloads.values()],
                "mechanisms": mech_state(state), "jaws": {k: list(v) for k, v in s.claw_jaws().items()},
                "sensors": sensors(pack)}

    for op in ops:
        name = op[0]
        entry: dict = {}
        events: list = []
        if name == "advance":
            entry["checkpoints"] = []
            for _ in range(op[1]):
                events += norm(list(s.advance().task_events))
                tick += 1
                if tick % 50 == 0:
                    entry["checkpoints"].append(checkpoint())
                drain(pack)
        elif name == "arm":
            entry["result"] = result(s.set_armed(op[1]))
        elif name == "kill":
            s.set_killed(True)
        elif name == "unkill":
            s.set_killed(False)
        elif name == "thrusters":
            s.command_thrusters(op[1])
        elif name == "fire":
            entry["result"] = result(s.fire(op[1]))
        elif name == "claw":
            entry["result"] = result(s.command_claw(op[1], op[2]))
        elif name == "move_claw":
            entry["result"] = result(s.move_claw(op[1], op[2]))
        elif name == "reload":
            entry["result"] = result(s.reload_all())
        elif name == "place":
            state = native.BodyState()
            state.position, state.orientation_wxyz = op[1], op[2]
            snap = s.place(state, clear_actuators=op[3])
            entry["body"] = body(snap.body)
        elif name == "place_moving":
            state = native.BodyState()
            state.position, state.orientation_wxyz, state.linear_velocity = op[1], op[2], op[3]
            entry["body"] = body(s.place(state, clear_actuators=True).body)
        elif name == "reset_tasks":
            ok, message = s.reset_tasks()
            entry["result"] = {"accepted": ok, "message": message}
        elif name == "full_reset":
            snap = s.full_reset(op[1])
            tick = 0
            entry["body"] = body(snap.body)
            entry["seed"] = s.seed
        elif name.startswith("run_"):
            r = {"run_start": lambda: s.run_start(op[1] if len(op) > 1 else {}), "run_stop": s.run_stop,
                 "run_adjust": lambda: s.run_adjust(op[1])}[name]()
            entry["result"] = result(r)
            entry["running"] = s.running
            entry["snapshot"] = s.run_snapshot()
        if name != "advance":
            events = norm(list(s.last_step.task_events))
        entry["events"] = events
        entry["feed"] = norm(s.take_feed())
        entry["counters"] = dict(s.task_counters)
        entry["final"] = checkpoint()
        log.append(entry)

    # run_snapshot document, driven by a stand-in TaskRuntime (the real one is ported separately).
    rows = RESOLVED.tasks["score_rows"]
    cases = []
    for running, started, stopped, scores, extra, now, adjust, msg in [
        (True, 1_000_000_000, None, {"gate": 12, "bins": 5.5, "extra_row": 3}, {"role": "repair", "ended_reason": ""}, 4_500_000_000, 2.0, "Run started"),
        (False, 1_000_000_000, 3_000_000_000, {"gate": 12}, {"ended_reason": "time", "target_class": "fire"}, 9_000_000_000, -1.5, "Run stopped"),
        (False, 0, None, {}, {}, 250_000_000, 0.0, ""),
    ]:
        fake = FakeTasks({"run": {"running": running, "started_ns": started, "stopped_ns": stopped},
                          "scores": scores}, extra)
        session = Session(RESOLVED, create_runtime(RESOLVED, sensor_ids=SENSORS), tasks=False)
        session.tasks = fake
        session.run_adjustment, session.run_message = adjust, msg
        session.last_step = type(session.last_step)(type("S", (), {"elapsed_ns": now})())
        cases.append({"snapshot": fake._s, "extra": extra, "now_ns": now, "adjustment": adjust,
                      "message": msg, "expected": session.run_snapshot()})
    if tasks:
        return {"timestep_ns": dt, "ops": ops, "task_ids": task_ids, "log": log}
    return {"timestep_ns": dt, "ops": ops, "log": log, "run_snapshot_cases": cases,
            "score_row_keys": [r["key"] for r in rows]}


def main() -> None:
    for name, data in (("pack_runtime", capture_pack_runtime()), ("mechanisms", capture_mechanisms()),
                       ("session", capture_session()),
                       ("session_tasks", capture_session(TASK_OPS, TASK_IDS, True))):
        (HERE / f"{name}_reference.json").write_text(json.dumps(data, sort_keys=True, allow_nan=False))
        print("wrote", f"{name}_reference.json")


if __name__ == "__main__":
    main()
