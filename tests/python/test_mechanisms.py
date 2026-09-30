"""Mechanism command contracts and independent rigid-point release equations.

Legacy behavioral reference: riptide_simulator 07647eeb mechanisms.py fire/reload and
claw_world.py command/step. Tests do not import or execute that repository.
"""

import copy
import dataclasses
import math
import unittest

import numpy as np
from nereus import _native as native
from nereus.mechanisms import Mechanisms


def robot():
    projectile = {"model": "finned_rigid_body", "mass_kg": 2., "added_mass_kg": 1.,
                  "displaced_volume_m3": .003, "neutral_buoyancy": False,
                  "center_of_mass_m": .2}
    launcher = {"id": "launcher", "type": "launcher", "frame": "mount", "parameters": {
        "slots": [{"id": str(i), "position_m": [0, i, 0],
                   "orientation_wxyz": [1, 0, 0, 0]} for i in range(2)],
        "projectile": projectile, "launch": {"spring_energy_j": 6.}, "capacity": 2,
        "cooldown_s": .5, "cooldown_group": "shared"}}
    dropper = copy.deepcopy(launcher)
    dropper.update(id="dropper", type="dropper")
    return {
        "frames": {"root": "center", "transforms": [
            {"parent": "center", "child": "reference", "position_m": [1, 0, 0],
             "orientation_wxyz": [1, 0, 0, 0]},
            {"parent": "center", "child": "mount", "position_m": [2, 0, 0],
             "orientation_wxyz": [1, 0, 0, 0]}]},
        "safety": {"initially_killed": True, "kill_disarms_mechanisms": True,
                   "arming": {"initially_armed": False, "arm_rejected_while_killed": True,
                              "applies_to": ["launcher", "dropper", "claw"]}},
        "mechanisms": [launcher, dropper, {
            "id": "claw", "type": "claw", "frame": "mount", "parameters": {
                "min_gap_m": .01, "max_gap_m": .21, "jaw_speed_m_s": .05,
                "completion_tolerance_m": .002, "initial_state": "closed",
                "timed_command": "signed_duration"}}, {
            "id": "magnet", "type": "magnet", "frame": "mount",
            "parameters": {"actuated": False}}]}


def fire(model, identifier="launcher", **overrides):
    args = dict(reference_frame="reference", water_density_kg_m3=1000., killed=False)
    args.update(overrides)
    return model.fire(identifier, native.Pose(), [0, 0, 0], [0, 0, 0], **args)


class MechanismTests(unittest.TestCase):
    def armed(self):
        model = Mechanisms(robot())
        self.assertTrue(model.set_armed(True, killed=False).accepted)
        return model

    def test_shared_cooldown_order_empty_reload_and_kill(self):
        model = Mechanisms(robot())
        self.assertFalse(model.set_armed(True, killed=True).accepted)
        self.assertFalse(fire(model).accepted)
        model.set_armed(True, killed=False)
        first = fire(model).release
        self.assertEqual((first.slot_id, first.slot_index), ("0", 0))
        self.assertFalse(fire(model, "dropper").accepted)
        state = model.snapshot(killed=False)
        self.assertEqual(state.releases["launcher"].state, "busy")
        self.assertEqual(state.releases["dropper"].state, "loaded")
        model.set_armed(False, killed=False)
        self.assertTrue(model.snapshot(killed=False).any_busy)
        model.advance(499_999_999, killed=False)
        model.set_armed(True, killed=False)
        self.assertFalse(fire(model).accepted)
        model.advance(1, killed=False)
        self.assertEqual(fire(model).release.slot_index, 1)
        model.advance(500_000_000, killed=False)
        self.assertFalse(fire(model).accepted)
        self.assertEqual(model.snapshot(killed=False).releases["launcher"].state, "empty")
        model.reload_all(killed=False)
        state = model.snapshot(killed=False)
        self.assertFalse(state.armed)
        self.assertFalse(state.any_busy)
        self.assertEqual(state.releases["launcher"].available, 2)
        self.assertEqual(first.slot_index, 0)  # previously released body remains owned and valid
        model.set_armed(True, killed=False)
        model.advance(0, killed=True)
        self.assertFalse(model.snapshot(killed=False).armed)  # deasserting kill does not rearm

    def test_release_equations_nonroot_reference_and_com_velocity(self):
        model = self.armed()
        pose = native.Pose()
        pose.translation = [10, 20, 30]
        pose.orientation_wxyz = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        release = model.fire("launcher", pose, [1, 2, 3], [0, 0, 2],
                             reference_frame="reference", water_density_kg_m3=1000.,
                             killed=False).release
        # Mount is 1m along reference X. Point velocity adds [0,2,0], launch is
        # 2m/s along X; COM offset adds [0,.4,0]. Rotate all by +90deg about Z.
        np.testing.assert_allclose(release.position_world_m, [10, 21, 30], atol=1e-12)
        np.testing.assert_allclose(release.velocity_com_world_m_s, [-4.4, 3, 3], atol=1e-12)
        np.testing.assert_allclose(release.angular_velocity_world_rad_s, [0, 0, 2], atol=1e-12)
        np.testing.assert_allclose(release.orientation_wxyz, pose.orientation_wxyz, atol=1e-12)

    def test_equivalent_reference_frames_give_identical_world_release(self):
        config = robot()
        edge = config["frames"]["transforms"][0]
        edge["orientation_wxyz"] = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        first, second = Mechanisms(config), Mechanisms(config)
        first.set_armed(True, killed=False)
        second.set_armed(True, killed=False)
        center_pose = native.Pose()
        center_pose.translation = [3, 4, 5]
        reference_pose = native.Pose()
        reference_pose.translation = [4, 4, 5]
        reference_pose.orientation_wxyz = edge["orientation_wxyz"]
        a = first.fire("launcher", center_pose, [1, 2, 3], [0, 0, 2],
                       reference_frame="center", water_density_kg_m3=1000., killed=False).release
        # Reference origin is center+[1,0,0]; its world velocity [1,4,3]
        # expressed in +90deg axes is [4,-1,3].
        b = second.fire("launcher", reference_pose, [4, -1, 3], [0, 0, 2],
                        reference_frame="reference", water_density_kg_m3=1000., killed=False).release
        for field in ("position_world_m", "orientation_wxyz", "velocity_com_world_m_s",
                      "angular_velocity_world_rad_s"):
            np.testing.assert_allclose(getattr(a, field), getattr(b, field), atol=1e-12)

    def test_claw_clamps_to_limits_and_open_initial_state(self):
        config = robot()
        config["mechanisms"][2]["parameters"]["initial_state"] = "open"
        model = Mechanisms(config)
        model.set_armed(True, killed=False)
        self.assertEqual(model.snapshot(killed=False).claws["claw"].state, "opened")
        model.command_claw("claw", False, killed=False)
        model.advance(10_000_000_000, killed=False)
        state = model.snapshot(killed=False)
        self.assertFalse(state.any_busy)
        self.assertEqual(state.claws["claw"].state, "closed")
        self.assertAlmostEqual(state.claws["claw"].gap_m, .01)
        model.command_claw("claw", True, killed=False)
        model.advance(10_000_000_000, killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .21)

    def test_slot_rotation_and_neutral_mass(self):
        config = robot()
        p = config["mechanisms"][0]["parameters"]
        p["slots"][0]["orientation_wxyz"] = [math.sqrt(.5), 0, math.sqrt(.5), 0]
        p["projectile"]["neutral_buoyancy"] = True
        model = Mechanisms(config)
        model.set_armed(True, killed=False)
        release = fire(model).release
        np.testing.assert_allclose(release.velocity_com_world_m_s, [0, 0, -math.sqrt(3)],
                                   atol=1e-12)

    def test_insertion_failure_and_invalid_inputs_do_not_consume(self):
        model = self.armed()
        before = model.snapshot(killed=False)

        def fail(_):
            raise RuntimeError("body capacity exhausted")

        with self.assertRaisesRegex(RuntimeError, "capacity"):
            fire(model, insert=fail)
        for override in ({"water_density_kg_m3": float("nan")}, {"reference_frame": "unknown"}):
            with self.assertRaises((ValueError, RuntimeError)):
                fire(model, **override)
        invalid = native.Pose()
        invalid.orientation_wxyz = [2, 0, 0, 0]
        with self.assertRaises(ValueError):
            model.fire("launcher", invalid, [0, 0, 0], [0, 0, 0],
                       reference_frame="reference", water_density_kg_m3=1000., killed=False)
        with self.assertRaises(ValueError):
            model.fire("launcher", native.Pose(), [math.inf, 0, 0], [0, 0, 0],
                       reference_frame="reference", water_density_kg_m3=1000., killed=False)
        self.assertEqual(model.snapshot(killed=False), before)
        received = []
        release = fire(model, insert=received.append).release
        self.assertIs(received[0], release)
        self.assertEqual(model.snapshot(killed=False).releases["launcher"].available, 1)

    def test_owned_immutable_config_release_and_snapshot(self):
        config = robot()
        model = Mechanisms(config)
        config["mechanisms"][0]["parameters"]["projectile"]["mass_kg"] = 1000
        model.set_armed(True, killed=False)
        release = fire(model).release
        self.assertEqual(release.projectile["mass_kg"], 2.)
        with self.assertRaises(TypeError):
            release.projectile["mass_kg"] = 10
        with self.assertRaises(dataclasses.FrozenInstanceError):
            release.slot_index = 4
        snapshot = model.snapshot(killed=False)
        with self.assertRaises(TypeError):
            snapshot.releases["launcher"] = None
        model.reload_all(killed=False)
        self.assertEqual(snapshot.releases["launcher"].available, 1)

    def test_claw_travel_timing_stop_kill_reload_and_reset(self):
        model = self.armed()
        self.assertTrue(model.command_claw("claw", True, killed=False).accepted)
        self.assertTrue(model.snapshot(killed=False).any_busy)
        model.advance(1_000_000_000, killed=False)
        state = model.snapshot(killed=False).claws["claw"]
        self.assertAlmostEqual(state.gap_m, .11)
        self.assertEqual(state.joint_positions_m, (.05, .05))
        self.assertFalse(state.grasp_supported)
        model.reload_all(killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .11)
        self.assertFalse(model.move_claw("claw", 0, killed=False).accepted)
        model.set_armed(True, killed=False)
        model.move_claw("claw", -.5, killed=False)
        model.advance(250_000_000, killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .085)
        # Legacy decrements timer before drive: the expiry step does not travel.
        model.advance(250_000_000, killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .085)
        model.command_claw("claw", True, killed=False)
        model.move_claw("claw", 0, killed=False)
        model.advance(1_000_000_000, killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .085)
        model.command_claw("claw", True, killed=False)
        model.advance(0, killed=True)
        model.advance(1_000_000_000, killed=False)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .085)
        model.reset(killed=False)
        self.assertEqual(model.time_ns, 0)
        self.assertAlmostEqual(model.snapshot(killed=False).claws["claw"].gap_m, .01)

    def test_invalid_commands_and_config_are_rejected(self):
        model = self.armed()
        before = model.snapshot(killed=False)
        for value in (math.nan, math.inf, 1e308, "1", True):
            with self.assertRaises(ValueError):
                model.move_claw("claw", value, killed=False)
        for value in (-1, .1, True, 2**63):
            with self.assertRaises(ValueError):
                model.advance(value, killed=False)
        self.assertEqual(model.snapshot(killed=False), before)
        for mutate in (
                lambda c: c["mechanisms"].append(c["mechanisms"][0]),
                lambda c: c["mechanisms"][0]["parameters"].update(capacity=3),
                lambda c: c["mechanisms"][2]["parameters"].pop("completion_tolerance_m"),
                lambda c: c["mechanisms"][2]["parameters"].update(completion_tolerance_m=.2)):
            config = robot()
            mutate(config)
            with self.assertRaises((ValueError, KeyError)):
                Mechanisms(config)


if __name__ == "__main__":
    unittest.main()
