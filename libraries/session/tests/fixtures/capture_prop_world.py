"""Records the Python PropWorld (PyBullet) over the scripted cases of tests/python/test_prop_world.py
and writes prop_world.json for the C++ port's gtest (no absolute paths).

Run from the repository root:
    PYTHONPATH=python/src python libraries/session/tests/fixtures/capture_prop_world.py

The fixture holds, per case, the exact inputs fed to step (constant-velocity segments, run-length
encoded claw joints, the constant body rotation) and the reference outputs: ordered events, and
prop states every SAMPLE_TICKS ticks. Python-side step timing is printed, not stored.
"""

import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tests/python"))
import test_prop_world as t  # noqa: E402
from robotics_platform.packs import resolve_scenario  # noqa: E402

SAMPLE_TICKS = 50


def rle(values):
    out = []
    for v in values:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return out


def state_of(world):
    return {
        k: {
            "position": list(s.position_m),
            "orientation_wxyz": list(s.orientation_wxyz),
            "attached": s.attached,
            "basket": s.basket,
        }
        for k, s in world.props_state().items()
    }


class Recorder:
    def __init__(self, resolved):
        self.rig = t.Rig(resolved)
        self.ops = []
        self.joints = []
        self.samples = []
        self.events = []
        self.tick = 0
        self.step_seconds = 0.0
        self.body_rotation = None
        world = self.rig.world
        real = world.step

        def step(dt, time_ns, pose, velocity, angular, joints, water, **kw):
            if self.body_rotation is None:
                self.body_rotation = list(pose.orientation_wxyz)
            begin = time.perf_counter()
            events = real(dt, time_ns, pose, velocity, angular, joints, water, **kw)
            self.step_seconds += time.perf_counter() - begin
            self.tick += 1
            self.joints.append(float(np.mean(joints)))
            for e in events:
                self.events.append({"tick": self.tick, **e})
            if self.tick % SAMPLE_TICKS == 0:
                self.samples.append(
                    {"tick": self.tick, "jaw": world.jaw_position_m, "props": state_of(world)}
                )
            return events

        world.step = step
        # Ops are captured by wrapping the Rig helpers that change the mount or the velocity.
        original_move, original_place = self.rig.move, self.rig.place_mount

        def move(seconds, velocity=(0.0, 0.0, 0.0)):
            self.ops.append(
                {"move": round(seconds / t.DT), "velocity": [float(v) for v in velocity]}
            )
            original_move(seconds, velocity)

        def place(xyz):
            self.ops.append({"place": [float(v) for v in xyz]})
            original_place(xyz)

        self.rig.move, self.rig.place_mount = move, place

    def case(self):
        rig = self.rig
        body_from_mount = np.linalg.inv(rig.world._mount)
        out = {
            "mount_local": rig.world._mount.tolist(),
            "mount_rotation": rig.mount[:3, :3].tolist(),
            "initial": state_of(rig.world),
            "body_rotation_wxyz": self.body_rotation,
            "ops": self.ops,
            "joints_rle": rle(self.joints),
            "events": self.events,
            "samples": self.samples,
            "final": state_of(rig.world),
            "final_jaw": rig.world.jaw_position_m,
            "final_held": rig.world.held_prop,
            "final_basket_contents": dict(rig.world.basket_contents()),
            "ticks": self.tick,
        }
        del body_from_mount
        return out


def record(resolved, name, script):
    recorder = Recorder(resolved)
    initial = state_of(recorder.rig.world)
    script(recorder.rig)
    data = recorder.case()
    data["initial"] = initial
    data["body_rotation_wxyz"] = recorder.body_rotation
    print(
        f"{name}: {data['ticks']} ticks, {1e6 * recorder.step_seconds / data['ticks']:.0f} us/step "
        f"(python), events={[(e['tick'], e['type'], e['data'].get('prop_id')) for e in data['events']]}",
        file=sys.stderr,
    )
    recorder.rig.close()
    return name, data


def main():
    resolved = resolve_scenario(t.SCENARIO)
    cases = {}

    def grasp_carry_release(rig):
        rig.pick("pill", height=0.25)
        rig.goto(rig.mount[:3, 3] + [0.2, 0.0, 0.0], 0.4)
        rig.command(True)
        rig.move(1.0)

    def drop(basket):
        def script(rig):
            rig.pick("pill")
            rig.carry_to(basket)
            rig.release()

        return script

    def bandage_elsewhere(rig):
        rig.pick("bandage")
        rig.release()

    def empty_jaws(rig):
        free = rig.local("task") + rig.table[:3, :3] @ [-0.2, 0.0, 0.001]
        rig.place_mount(free)
        rig.command(True)
        rig.move(2.4)
        rig.command(False)
        rig.move(4.0)

    scripts = {
        "grasp_carry_release": grasp_carry_release,
        "drop_helmet_basket": drop("helmet_basket"),
        "drop_warning_basket": drop("warning_basket"),
        "release_elsewhere": bandage_elsewhere,
        "empty_jaws": empty_jaws,
    }
    for name, script in scripts.items():
        key, data = record(resolved, name, script)
        cases[key] = data
    # reset: pick, world.reset() + mechanisms.reset(), pick again (ops after the reset marker).
    recorder = Recorder(resolved)
    rig = recorder.rig
    initial = state_of(rig.world)
    rig.pick("pill")
    marker = len(recorder.ops)
    before = state_of(rig.world)
    rig.world.reset()
    rig.mechanisms.reset(killed=False)
    rig.mechanisms.set_armed(True, killed=False)
    recorder.ops.append({"reset": True})
    after_reset = state_of(rig.world)
    rig.time_ns, rig.events = 0, []
    rig.pick("bandage")
    data = recorder.case()
    data["initial"], data["body_rotation_wxyz"] = initial, recorder.body_rotation
    data["before_reset"], data["after_reset"], data["reset_op_index"] = before, after_reset, marker
    cases["reset_then_pick"] = data
    print(f"reset_then_pick: {data['ticks']} ticks", file=sys.stderr)
    rig.close()
    out = Path(__file__).with_name("prop_world.json")
    out.write_text(json.dumps({"dt": t.DT, "sample_ticks": SAMPLE_TICKS, "cases": cases}))
    print(f"wrote {out} ({out.stat().st_size // 1024} KiB)", file=sys.stderr)


if __name__ == "__main__":
    main()
