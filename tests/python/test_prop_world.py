"""Scripted physical cases for the pack-driven claw/table prop world (no ROS, no wall time).

Jaw motion comes from the real Mechanisms claw runtime; the robot pose is driven directly. When
the original riptide_simulator claw_world.py is importable (RP_ORIGINAL_CLAW_WORLD or the sibling
release checkout) the same script also runs through its ClawWorld and outcomes are compared.
"""

import copy
import importlib.util
import math
import os
import sys
import unittest
from pathlib import Path

import numpy as np
from robotics_platform import _native as native
from robotics_platform.mechanisms import Mechanisms
from robotics_platform.packs import resolve_scenario
from robotics_platform.prop_world import PropWorld, Water, _matrix, _xyzw
from ruamel.yaml import YAML

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt/scenario.yaml"
ORIGINAL = Path(
    os.environ.get(
        "RP_ORIGINAL_CLAW_WORLD",
        ROOT.parent.parent / "release/src/riptide_simulator/c_simulator",
    )
)
DT = 0.004
POSITION_TOLERANCE_M = 1e-3  # measured agreement ~1e-5 m (same Bullet solver and inputs)

try:
    import pybullet  # noqa: F401
except ImportError:  # pragma: no cover
    pybullet = None


def pose_of(matrix):
    q = _xyzw(matrix[:3, :3])
    pose = native.Pose()
    pose.translation = matrix[:3, 3]
    pose.orientation_wxyz = [q[3], q[0], q[1], q[2]]
    return pose


class Rig:
    """Drives one PropWorld (and optionally the original ClawWorld) from a motion script."""

    def __init__(self, resolved, original=None):
        self.resolved = resolved
        self.world = PropWorld(resolved)
        self.mechanisms = Mechanisms(resolved.robot)
        self.mechanisms.set_armed(True, killed=False)
        self.original = original
        self.time_ns = 0
        self.events = []
        self.original_events = []
        self.table = self.world._world_from_task
        yaw = self.table[:3, :3]
        self.mount = np.eye(4)
        self.mount[:3, :3] = yaw  # jaw axis across the props, table-aligned
        self.water = Water()

    def local(self, name):
        return self.world._frames[name][:3, 3]

    def prop_position(self, name):
        return np.array(self.world.props_state()[name].position_m)

    def place_mount(self, world_xyz):
        self.mount[:3, 3] = world_xyz

    def command(self, opened):
        self.mechanisms.command_claw("claw", opened, killed=False)
        if self.original is not None:
            self.original.command(opened)

    def move(self, seconds, velocity=(0.0, 0.0, 0.0)):
        velocity = np.asarray(velocity, dtype=float)
        for _ in range(round(seconds / DT)):
            self.mechanisms.advance(round(DT * 1e9), killed=False)
            self.time_ns += round(DT * 1e9)
            self.mount[:3, 3] += velocity * DT
            body = self.mount @ np.linalg.inv(self.world._mount)
            joints = self.mechanisms.snapshot(killed=False).claws["claw"].joint_positions_m
            self.events += self.world.step(
                DT, self.time_ns, pose_of(body), velocity, [0, 0, 0], joints, self.water
            )
            if self.original is not None:
                self.original.step(DT, body, velocity, np.zeros(3), np.zeros(3), 998.2)
                self.original_events += list(self.original.events)
                self.original.events.clear()

    def goto(self, target, speed):
        delta = np.asarray(target, dtype=float) - self.mount[:3, 3]
        distance = float(np.linalg.norm(delta))
        if distance > 1e-9:
            self.move(distance / speed, delta / distance * speed)

    def pick(self, name, height=0.25):
        """Open above the prop, descend, close until grasped, then lift ``height`` m."""
        rest = self.prop_position(name)
        bottom = rest[2] + self.world.props[name]["center"][2] - self.world.props[name]["half"][2]
        self.place_mount([rest[0], rest[1], bottom + 0.25])
        self.command(True)
        self.move(2.4)
        self.move(1.0, (0, 0, -0.25))
        self.move(0.3)
        self.command(False)
        self.move(3.0)
        self.move(height / 0.25, (0, 0, 0.25))

    def carry_to(self, frame, clearance=0.2):
        """Fly the held prop above the named table frame at ``clearance`` m above the table."""
        target = self.local(frame).copy()
        target[2] = self.local("task")[2] + clearance
        self.goto([target[0], target[1], self.mount[2, 3]], 0.3)
        self.move(0.5)

    def release(self):
        self.command(True)
        self.move(4.0)

    def ids(self):
        return [(e["type"], e["id"], e["region"]) for e in self.events]

    def close(self):
        self.world.close()
        close_original(self.original)
        self.original = None


def close_original(original):
    if original is not None:
        try:
            original.close()
        except pybullet.error:  # already disconnected
            pass


def original_module():
    scripts = ORIGINAL / "scripts/claw_world.py"
    if pybullet is None or not scripts.is_file():
        return None
    sys.path.insert(0, str(scripts.parent))
    try:
        spec = importlib.util.spec_from_file_location("original_claw_world", scripts)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception:  # pragma: no cover
        return None
    finally:
        sys.path.remove(str(scripts.parent))


@unittest.skipIf(pybullet is None, "optional pybullet is not installed")
class PropWorldTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.resolved = resolve_scenario(SCENARIO)

    def rig(self, original=None):
        rig = Rig(self.resolved, original)
        self.addCleanup(rig.close)
        return rig

    def test_grasp_follows_robot_and_open_releases(self):
        rig = self.rig()
        start = rig.prop_position("pill")
        rig.pick("pill", height=0.25)
        self.assertEqual([e["id"] for e in rig.events], ["grasp"])
        attach = rig.events[0]
        self.assertEqual(attach["task"], "table")
        self.assertEqual(attach["type"], "attach")
        self.assertEqual(attach["data"], {"prop_id": "pill", "mechanism_id": "claw"})
        state = rig.world.props_state()["pill"]
        self.assertTrue(state.attached)
        self.assertEqual(state.mechanism_id, "claw")
        self.assertGreater(rig.prop_position("pill")[2], start[2] + 0.2)
        rig.goto(rig.mount[:3, 3] + [0.2, 0.0, 0.0], 0.4)
        self.assertLess(np.linalg.norm(rig.prop_position("pill")[:2] - (start[:2] + [0.2, 0])), 0.3)
        moved = rig.prop_position("pill")[:2] - start[:2]
        self.assertGreater(np.linalg.norm(moved), 0.15)  # it travelled with the robot
        rig.command(True)
        rig.move(1.0)
        detach = [e for e in rig.events if e["type"] == "detach"]
        self.assertEqual(len(detach), 1)
        self.assertEqual(detach[0]["id"], "release")
        self.assertEqual(detach[0]["data"]["reason"], "released")
        self.assertFalse(rig.world.props_state()["pill"].attached)

    def carry_and_drop(self, prop, basket, original=None):
        rig = self.rig(original)
        rig.pick(prop)
        rig.carry_to(basket)
        rig.release()
        return rig

    def test_basket_drop_correct_and_incorrect(self):
        rig = self.carry_and_drop("pill", "helmet_basket")
        drop = [e for e in rig.events if e["id"] == "basket_drop"]
        self.assertEqual(len(drop), 1)
        self.assertEqual(drop[0]["type"], "drop_into")
        self.assertEqual(drop[0]["region"], "helmet_basket")
        self.assertEqual(
            drop[0]["data"],
            {"prop_id": "pill", "basket": "helmet_basket", "expected_basket": "helmet_basket"},
        )
        self.assertEqual(rig.world.basket_contents(), {"pill": "helmet_basket"})
        self.assertEqual(rig.world.props_state()["pill"].basket, "helmet_basket")
        wrong = self.carry_and_drop("pill", "warning_basket")
        drop = [e for e in wrong.events if e["id"] == "basket_drop"]
        self.assertEqual(len(drop), 1)
        self.assertEqual(drop[0]["region"], "warning_basket")
        self.assertEqual(drop[0]["data"]["basket"], "warning_basket")
        self.assertEqual(drop[0]["data"]["expected_basket"], "helmet_basket")
        self.assertFalse([e for e in wrong.events if e["id"] == "object_dropped"])

    def test_release_elsewhere_is_object_dropped(self):
        rig = self.rig()
        rig.pick("bandage")
        rig.release()
        dropped = [e for e in rig.events if e["id"] == "object_dropped"]
        self.assertEqual(len(dropped), 1)
        self.assertEqual(dropped[0]["type"], "drop_into")
        self.assertEqual(dropped[0]["region"], "table")
        self.assertEqual(dropped[0]["data"], {"prop_id": "bandage"})
        self.assertFalse([e for e in rig.events if e["id"] == "basket_drop"])

    def test_jaws_without_prop_do_not_attach(self):
        rig = self.rig()
        free = rig.local("task") + rig.table[:3, :3] @ [-0.2, 0.0, 0.001]
        rig.place_mount(free)
        rig.command(True)
        rig.move(2.4)
        rig.command(False)
        rig.move(4.0)
        self.assertEqual(rig.events, [])
        self.assertIsNone(rig.world.held_prop)
        self.assertAlmostEqual(rig.world.jaw_position_m, 0.0, places=6)

    def test_reset_restores_initial_prop_poses(self):
        rig = self.rig()
        before = {k: v.position_m for k, v in rig.world.props_state().items()}
        rig.pick("pill")
        self.assertNotEqual(rig.world.props_state()["pill"].position_m, before["pill"])
        rig.world.reset()
        rig.mechanisms.reset(killed=False)
        after = {k: v.position_m for k, v in rig.world.props_state().items()}
        self.assertFalse(rig.world.props_state()["pill"].attached)
        self.assertIsNone(rig.world.held_prop)
        for key in before:
            np.testing.assert_allclose(after[key], before[key], atol=1e-9)

    def test_instances_are_independent_and_replay_identically(self):
        first, second = self.rig(), self.rig()
        second_start = {k: v.position_m for k, v in second.world.props_state().items()}
        first.pick("pill")
        self.assertTrue(first.world.props_state()["pill"].attached)
        self.assertEqual(
            {k: v.position_m for k, v in second.world.props_state().items()}, second_start
        )
        second.pick("pill")
        self.assertEqual(first.events, second.events)
        self.assertEqual(first.world.props_state(), second.world.props_state())

    def test_invalid_inputs_and_missing_dependency_paths(self):
        rig = self.rig()
        pose = pose_of(np.eye(4))
        with self.assertRaises(ValueError):
            rig.world.step(0.0, 0, pose, [0, 0, 0], [0, 0, 0], [0, 0], Water())
        with self.assertRaises(ValueError):
            rig.world.step(DT, 0, pose, [0, math.nan, 0], [0, 0, 0], [0, 0], Water())
        bad = copy.deepcopy(self.resolved)
        for item in bad.robot["mechanisms"]:
            if item["type"] == "claw":
                item["parameters"]["pads"]["left"] = "missing_asset"
        with self.assertRaises(ValueError):
            PropWorld(bad)

    def test_quaternion_helper_round_trips(self):
        m = _matrix([1, 2, 3], (math.cos(0.4), 0, math.sin(0.4), 0))
        q = _xyzw(m[:3, :3])
        np.testing.assert_allclose(_matrix([1, 2, 3], (q[3], q[0], q[1], q[2])), m, atol=1e-12)

    def test_matches_original_claw_world(self):
        module = original_module()
        if module is None:
            self.skipTest("original claw_world.py is unavailable")
        rig = Rig(self.resolved)
        self.addCleanup(rig.close)
        for basket, other in (("helmet_basket", "helmet"), ("warning_basket", "warning")):
            rig.world.reset()
            rig.mechanisms.reset(killed=False)
            rig.mechanisms.set_armed(True, killed=False)
            rig.time_ns, rig.events = 0, []
            rig.original = self.original(module, rig)
            rig.original_events = []
            self.addCleanup(close_original, rig.original)
            rig.pick("pill")
            rig.carry_to(basket)
            rig.release()
            mine = [(e["type"], e["data"]["prop_id"]) for e in rig.events]
            self.assertEqual(mine, [("attach", "pill"), ("detach", "pill"), ("drop_into", "pill")])
            outcome = [(e[0], e[1]) for e in rig.original_events]
            results = {"helmet": "success", "warning": "wrong_target"}
            self.assertEqual(
                outcome, [("pill", "grasped"), ("pill", "released"), ("pill", results[other])]
            )
            self.assertEqual(rig.original_events[-1][2], other)
            self.assertEqual(rig.events[-1]["region"], basket)
            for key in rig.world.props:
                expected = rig.original.prop_pose(key)[:3, 3]
                actual = rig.prop_position(key)
                self.assertLess(np.linalg.norm(actual - expected), POSITION_TOLERANCE_M, key)
            close_original(rig.original)

    def original(self, module, rig):
        config = YAML(typ="safe").load((ORIGINAL / "config/talos_tasks.yaml").read_text())["claw"]
        for key, prop in config["props"].items():
            declared = next(p for p in rig.world._rigid if p["id"] == key)["parameters"]
            self.assertEqual(
                (prop["mass"], prop["volume"]), (declared["mass_kg"], declared["volume_m3"])
            )
        frames = {"table": rig.table}
        for name, original_name in (("helmet_basket", "helmet"), ("warning_basket", "warning")):
            frames[original_name] = rig.world._frames[name]
        for key in config["props"]:
            frames[key] = rig.world._frames[key]
        placement = self.resolved.scenario["pool_placement"]
        half = math.radians(placement["yaw_deg"]) / 2
        pool_pose = _matrix(placement["position_m"], (math.cos(half), 0, 0, math.sin(half)))
        return module.ClawWorld(
            config,
            frames,
            rig.world._mount,
            ORIGINAL / "collision_files/tasks",
            np.linalg.inv(pool_pose),
        )


if __name__ == "__main__":
    unittest.main()
