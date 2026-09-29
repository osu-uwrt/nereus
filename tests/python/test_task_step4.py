"""Slalom, bins, magnet lights, surface and table rows through TaskRuntime and the pack hook.

Scripted scenarios use the real pack. Table events come from a physical owner, so the real
pack has no table task yet; tests add an empty table task to a copy of the resolved pack.
Comparisons against the original RunScore/CourseJudge run when the original checkout exists
(override with RP_SCORING_REFERENCE) and are skipped otherwise.
"""

import copy
import importlib.util
import math
import os
import unittest
from pathlib import Path

import numpy as np
import yaml
from robotics_platform.packs import resolve_scenario
from robotics_platform.task_runtime import TaskRuntime, _envelope, _placement, _pose
from robotics_platform.task_zones import OpenCrate

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt/scenario.yaml"
ORIGINAL_ROOT = Path("/home/ubuntu/osu-uwrt/release/src/riptide_simulator/c_simulator")
ORIGINAL = Path(os.environ.get("RP_SCORING_REFERENCE",
                               ORIGINAL_ROOT / "tasks/2026/behavior/scoring.py"))
ORIGINAL_TASKS = ORIGINAL.parents[1] / "config/tasks.yaml"
TABLE = {"task": "table", "position_m": [19.290761, 9.007, -1.54], "yaw_deg": 89.039804}
STEP_NS = 125_000_000   # 0.125 s: exact in binary, so dwell sums match the original floats
UP = [1.0, 0, 0, 0]


def quaternion(yaw=0.0, roll=0.0):
    h, r = yaw / 2, roll / 2
    return [math.cos(h) * math.cos(r), math.cos(h) * math.sin(r),
            -math.sin(h) * math.sin(r), math.sin(h) * math.cos(r)]


def plain(value):
    from collections.abc import Mapping
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


def load_resolved():
    resolved = resolve_scenario(SCENARIO)
    # Scoring cases start at boot; the pack default (operator "start") is covered by
    # test_session_run_control.
    resolved.scenario["run"]["auto_start"] = True
    return resolved


class Course:
    """A TaskRuntime plus pose helpers in task/frame coordinates and an advancing clock."""

    def __init__(self, resolved, tasks, **options):
        self.resolved = copy.deepcopy(resolved)
        self.resolved.run_options.update(options)
        self.runtime = TaskRuntime(self.resolved, task_ids=tasks)
        self.place = {p["task"]: _placement(p) for p in self.resolved.scenario["task_placements"]}
        self.frames = {(t["id"], f["id"]): f for t in self.resolved.task_definitions
                       for f in t["frames"]}
        self.t = 0
        self.events = []

    def advance(self, steps=1):
        self.t += steps * STEP_NS
        return self.t

    def pose(self, task, xyz, yaw=0.0, frame=None, roll=0.0):
        base = self.place[task]
        if frame is not None:
            f = self.frames[task, frame]
            base = base.compose(_pose(f["position_m"], f["orientation_wxyz"]))
        return base.compose(_pose(xyz, quaternion(yaw, roll)))

    def observe(self, pose, **kwargs):
        events = self.runtime.observe(self.advance(), pose, **kwargs)
        self.events.extend(plain(events))
        return plain(events)

    def submit(self, *events):
        events = self.runtime.observe_events(self.t, list(events))
        self.events.extend(plain(events))
        return plain(events)

    def gate(self, y=-0.75):
        for x in (3, 2, 1, 0.5, 0, -0.5, -1, -2):
            self.observe(self.pose("gate", [x, y, -0.2]))

    @property
    def scores(self):
        return dict(self.runtime.snapshot()["scores"])

    def kinds(self, kind):
        return [e for e in self.events if e["type"] == kind]


def table_event(kind, identifier, prop, region="", **data):
    return {"id": identifier, "type": kind, "task": "table", "region": region,
            "data": {"prop_id": prop, "mechanism_id": "claw", **data}}


def grasp(prop):
    return table_event("attach", "grasp", prop)


def release(prop, **data):
    return table_event("detach", "release", prop, **data)


def basket(prop, into, expected):
    return table_event("drop_into", "basket_drop", prop, into, basket=into,
                       expected_basket=expected)


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.resolved = load_resolved()
        cls.envelope = _envelope(cls.resolved.robot)


class SlalomTests(Base):
    def cross(self, course, row, y, z=-0.3, forward=True):
        xs = (1, 0.5, -0.5, -1) if forward else (-1, -0.5, 0.5, 1)
        for x in xs:
            course.observe(course.pose("slalom", [x, y, z], frame=row))

    def test_side_and_depth_points_follow_the_gate_side(self):
        course = Course(self.resolved, ["gate", "slalom"])
        course.gate(y=-0.75)   # repair half: side -1
        self.assertEqual(course.scores["gate"], 550)
        self.cross(course, "slalom_front", -0.5)                 # same side, in band
        self.cross(course, "slalom_middle", 0.5)                 # other side, in band
        self.cross(course, "slalom_back", -0.5, z=-2.0)          # same side, too deep
        self.assertEqual({k: v for k, v in course.scores.items() if k.startswith("slalom")},
                         {"slalom_front": 600, "slalom_middle": 400, "slalom_back": 400})
        passes = [e for e in course.events if e["task"] == "slalom" and e["type"] == "pass_through"]
        self.assertEqual([e["data"]["depth_overlap"] for e in passes], [True, True, False])

    def test_rows_reject_edges_reverse_early_and_only_raise(self):
        course = Course(self.resolved, ["gate", "slalom"])
        self.cross(course, "slalom_front", -0.5)                 # before the gate: not scored
        self.assertEqual(course.scores, {})
        course.gate(y=0.75)                                      # rescue half: side +1
        self.cross(course, "slalom_front", 1.6)                  # |y| must be < 1.55
        self.cross(course, "slalom_front", 0.5, forward=False)   # reverse direction
        self.assertNotIn("slalom_front", course.scores)
        self.cross(course, "slalom_front", 1.5, z=0.9)           # above the band: side only
        self.assertEqual(course.scores["slalom_front"], 400)
        self.cross(course, "slalom_front", -1.5)                 # other side never lowers a row
        self.cross(course, "slalom_front", 0.2, z=-0.2)          # depth raises it
        self.assertEqual(course.scores["slalom_front"], 600)
        self.cross(course, "slalom_front", -0.2)
        self.assertEqual(course.scores["slalom_front"], 600)

    def test_row_frames_are_offset_and_reset_clears(self):
        course = Course(self.resolved, ["gate", "slalom"])
        course.gate()
        # A pass through the middle row's plane at the FRONT row's lateral centre is |y| 0.69 there.
        for x in (1, 0.5, -0.5, -1):
            course.observe(course.pose("slalom", [-2.127361 + x, 0.693981 - 0.4, -0.3]))
        self.assertEqual(course.scores["slalom_middle"], 600)
        self.assertNotIn("slalom_front", course.scores)
        course.runtime.reset()
        self.assertEqual(course.scores, {})


class LightsAndBinsTests(Base):
    def crate_pose(self, course, crate):
        return course.place["bins"].compose(_pose(
            course.frames["bins", crate]["position_m"], UP))

    def light(self, course, index):
        target = f"magnet_target{index}"
        runtime = course.runtime
        sensor = runtime._targets["bins", target]._sensor
        probe = runtime._targets["bins", target]._probe
        return sensor - probe, target   # identity-orientation reference position

    def test_light_needs_continuous_half_second_and_latches(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        position, _ = self.light(course, 1)
        near = _pose(position, UP)
        off = _pose(position + [0.16, 0, 0], UP)
        for pose in (near, near, near, off, near, near, near):     # 0.375 s, broken, 0.375 s
            course.observe(pose)
        self.assertNotIn("lights", course.scores)
        course.observe(near)                                        # 0.5 s continuous
        self.assertEqual(course.scores["lights"], 500)
        self.assertEqual(course.runtime.snapshot()["latched"],
                         {"bins/magnet_target1": True, "bins/magnet_target2": False})
        course.observe(off)                                         # latched: leaving changes nothing
        position2, _ = self.light(course, 2)
        for _ in range(5):
            course.observe(_pose(position2, UP))
        self.assertEqual(course.scores["lights"], 1000)
        self.assertEqual(len(course.kinds("activate")), 2)

    def test_light_latched_before_the_gate_scores_nothing(self):
        course = Course(self.resolved, ["gate", "bins"])
        position, _ = self.light(course, 1)
        for _ in range(6):
            course.observe(_pose(position, UP))
        self.assertTrue(course.runtime.snapshot()["latched"]["bins/magnet_target1"])
        course.gate()
        self.assertNotIn("lights", course.scores)

    def test_light_range_is_six_inches_from_the_sensor(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        position, _ = self.light(course, 1)
        for _ in range(8):
            course.observe(_pose(position + [0.1525, 0, 0], UP))   # just outside 0.1524
        self.assertNotIn("lights", course.scores)
        for _ in range(5):
            course.observe(_pose(position + [0.1523, 0, 0], UP))
        self.assertEqual(course.scores["lights"], 500)

    def drop(self, course, ident, crate, offset=(0.0, 0.0), *, kind="dropper", top=0.6):
        origin = self.crate_pose(course, crate)
        rotation = origin.compose(_pose([0, 0, 0], UP))

        def world(local):
            return origin.apply(local)

        axis = rotation.apply([0, 0, -1]) - origin.translation
        x, y = offset
        tip = world([x, y, top])
        course.t += 0     # no time step for release
        course.events.extend(plain(course.runtime.release_projectile(
            course.advance(), ident, kind, tip, 0.013, 0.083)))
        step = None
        heights = [top, 0.3, 0.05, -0.1, -0.5]
        for start, end in zip(heights, heights[1:]):
            step = course.runtime.step_projectile(
                course.advance(), ident, world([x, y, start]), world([x, y, end]), axis,
                axis * 1.0)
            course.events.extend(plain(step.events))
            if step.stop:
                break
        return step

    def test_dropper_scores_good_and_unique_correct_crates(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()   # repair: fire crates are bin_vinyl2 / bin_vinyl3
        step = self.drop(course, 1, "bin_vinyl2")
        self.assertTrue(step.stop)
        landed = course.kinds("payload_landing")[-1]
        self.assertEqual((landed["id"], landed["region"], landed["data"]["outcome"],
                          landed["data"]["region_class"]), ("bin_vinyl2_inside", "bin_vinyl2",
                                                            "inside", "fire"))
        self.assertEqual(course.scores["bins"], 300 + 500)
        self.drop(course, 2, "bin_vinyl2")            # same crate again: 300 more, class only once
        self.assertEqual(course.scores["bins"], 600 + 500)
        self.drop(course, 3, "bin_vinyl3")            # third shot exceeds max_shots
        self.assertEqual(course.scores["bins"], 600 + 500)

    def test_wrong_class_and_unique_crates(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        self.drop(course, 1, "bin_vinyl1")            # blood crate for a repair robot
        self.assertEqual(course.scores["bins"], 300)
        self.drop(course, 2, "bin_vinyl3")            # fire crate
        self.assertEqual(course.scores["bins"], 600 + 500)

    def test_two_distinct_correct_crates_and_rescue_role(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        self.drop(course, 1, "bin_vinyl2")
        self.drop(course, 2, "bin_vinyl3")
        self.assertEqual(course.scores["bins"], 600 + 1000)
        rescue = Course(self.resolved, ["gate", "bins"])
        rescue.gate(y=0.75)                            # rescue: blood crates are correct
        self.drop(rescue, 1, "bin_vinyl1")
        self.assertEqual(rescue.scores["bins"], 800)

    def test_negatives_release_before_gate_torpedo_rim_wall_and_miss(self):
        early = Course(self.resolved, ["gate", "bins"])
        self.drop(early, 1, "bin_vinyl2")              # released before the gate: ineligible
        early.gate()
        self.assertNotIn("bins", early.scores)

        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        self.drop(course, 1, "bin_vinyl2", kind="launcher")   # lands inside, scores nothing (T15)
        self.assertEqual(course.kinds("payload_landing")[-1]["data"]["mechanism_type"], "launcher")
        self.assertNotIn("bins", course.scores)
        step = self.drop(course, 2, "bin_vinyl2", offset=(0.16, 0.0))   # rim: |x| + r > inner
        landed = course.kinds("payload_landing")[-1]
        self.assertEqual((landed["id"], landed["data"]["detail"]), ("bin_vinyl2_blocked", "rim"))
        self.assertTrue(step.stop)
        self.assertNotIn("bins", course.scores)
        self.assertEqual(len(course.kinds("payload_released")), 2)

        outside = Course(self.resolved, ["gate", "bins"])
        outside.gate()
        step = self.drop(outside, 1, "bin_vinyl2", offset=(1.5, 0.0))
        self.assertTrue(step.stop)
        miss = outside.kinds("miss")[0]
        self.assertEqual((miss["task"], miss["data"]["reason"]), ("bins", "pool_floor"))
        floor = outside.runtime.snapshot()["environment"]["floor_z_m"]
        self.assertAlmostEqual(step.position_world[2], floor + 0.0415)
        np.testing.assert_array_equal(step.velocity_world, [0, 0, 0])
        self.assertEqual(outside.kinds("payload_landing"), [])
        # A stopped payload is inert; further steps report the stop without events.
        again = outside.runtime.step_projectile(
            outside.advance(), 1, [0, 0, 0], [0, 0, -1], [0, 0, -1], [0, 0, 0])
        self.assertTrue(again.stop)
        self.assertEqual((again.events, again.position_world), ((), None))

    def test_payload_timeout_and_pool_wall_are_misses(self):
        course = Course(self.resolved, ["gate", "bins"])
        course.gate()
        origin = course.place["bins"].translation
        course.runtime.release_projectile(course.advance(), 7, "dropper", origin + [0, 0, 0.5],
                                          0.013, 0.083)
        axis = np.array([0.0, 0, -1])
        step = course.runtime.step_projectile(course.advance(), 7, origin + [0, 0, 0.5],
                                              origin + [0, 0, 0.49], axis, [0, 0, -1.0])
        self.assertFalse(step.stop)
        self.assertIsNone(step.position_world)
        course.t += 31_000_000_000
        step = course.runtime.step_projectile(course.t, 7, origin + [0, 0, 0.49],
                                              origin + [0, 0, 0.48], axis, [0, 0, -1.0])
        self.assertTrue(step.stop)
        self.assertEqual(plain(step.events)[0]["data"]["reason"], "pool_wall_or_timeout")
        course.runtime.release_projectile(course.advance(), 8, "dropper", [0, 0, -1], 0.013)
        step = course.runtime.step_projectile(course.advance(), 8, [0.5, 0, -1], [-0.5, 0, -1],
                                              axis, [-1.0, 0, 0])
        self.assertEqual(plain(step.events)[0]["data"]["reason"], "pool_wall_or_timeout")

    def test_crate_walls_clip_and_damp_velocity_and_entered_payloads_stay_inside(self):
        origin = _pose([0, 0, 0], UP)
        crate = self.resolved.task_definitions[[t["id"] for t in self.resolved.task_definitions]
                                               .index("bins")]["regions"][0]["parameters"]
        crate = OpenCrate(crate, origin)
        axis = np.array([0.0, 0, -1])
        # From outside into the outer wall (x = outer + radius) at mid height.
        step = crate.step(np.array([0.4, 0.0, 0.1]), np.array([0.0, 0.0, 0.1]),
                          np.array([-2.0, 0.0, -1.0]), axis, 0.013, 0.083, False)
        self.assertAlmostEqual(step.position[0], 0.333248 / 2 + 0.013)
        np.testing.assert_allclose(step.velocity, [0.0, 0.0, -0.15], atol=1e-12)
        self.assertIsNone(step.outcome)
        # Once entered the inner liner face contains the payload.
        step = crate.step(np.array([0.0, 0.0, 0.1]), np.array([0.3, 0.0, 0.1]),
                          np.array([2.0, 0.0, 0.0]), axis, 0.013, 0.083, True)
        self.assertAlmostEqual(step.position[0], 0.3048 / 2 - 0.004 - 0.013)
        self.assertTrue(step.entered)

    def test_step_reports_panel_block_correction(self):
        course = Course(self.resolved, ["torpedo"])
        panel = course.runtime._panels["torpedo", "panel"]
        start, end = panel.world_point([1, 0, 0]), panel.world_point([-1, 0, 0])
        course.runtime.release_projectile(course.advance(), 1, "launcher", start, 0.013)
        axis = (panel.world_point([-1, 0, 0]) - panel.world_point([0, 0, 0]))
        axis /= np.linalg.norm(axis)
        step = course.runtime.step_projectile(course.advance(), 1, start, end, axis, axis * 3)
        self.assertTrue(step.stop)
        np.testing.assert_allclose(step.position_world, panel.world_point([0, 0, 0]), atol=1e-12)
        self.assertEqual(step.velocity_world, (0.0, 0.0, 0.0))
        self.assertEqual(step.events[0]["id"], "panel_blocked")


class SurfaceTests(Base):
    top = float(np.max(_envelope(load_resolved().robot)[:, 2]))

    def dive_and_surface(self, course, *, inside=True, yaw=None, count=1, dwell_ticks=5,
                         x=0.0, y=0.0):
        """Submerge in the octagon, then rise to the water line at (x, y) in octagon axes."""
        z0 = course.place["surface"].translation[2]
        octagon_yaw = math.radians(-45.273547)
        heading = octagon_yaw if yaw is None else yaw
        low = course.pose("surface", [x, y, -z0 - 0.5], yaw=heading - octagon_yaw)
        course.observe(low)
        rise = course.pose("surface", [x, y, -z0 - self.top + 0.01], yaw=heading - octagon_yaw)
        for _ in range(dwell_ticks):
            course.observe(rise)
        return rise

    def test_surfacing_needs_gate_submersion_inside_and_half_a_second(self):
        course = Course(self.resolved, ["gate", "surface"])
        course.gate()
        rise = self.dive_and_surface(course, dwell_ticks=3)
        self.assertNotIn("surface", course.scores)         # 0.375 s of dwell
        course.observe(rise)
        self.assertEqual(course.scores["surface"], 800)
        self.assertEqual([e["id"] for e in course.kinds("surface_reached")], ["surfaced"])
        for _ in range(3):                                   # edge-triggered: no repeats
            course.observe(rise)
        self.assertEqual(len(course.kinds("surface_reached")), 1)
        low = course.pose("surface", [0, 0, -course.place["surface"].translation[2] - 0.5])
        course.observe(low)
        self.assertEqual(len(course.kinds("surface_lost")), 1)

    def test_never_submerged_or_outside_octagon_does_not_surface(self):
        never = Course(self.resolved, ["gate", "surface"])
        never.gate()
        z0 = never.place["surface"].translation[2]
        pose = never.pose("surface", [0, 0, -z0 - self.top + 0.01])
        for _ in range(8):
            never.observe(pose)
        self.assertNotIn("surface", never.scores)
        outside = Course(self.resolved, ["gate", "surface"])
        outside.gate()
        self.dive_and_surface(outside, x=2.0, y=0.0)        # submerged inside? no: outside, at the water
        self.assertNotIn("surface", outside.scores)

    def test_breach_outside_the_octagon_ends_scoring(self):
        course = Course(self.resolved, ["gate", "slalom", "surface"])
        course.gate()
        rise = self.dive_and_surface(course)
        self.assertEqual(course.scores["surface"], 800)
        z0 = course.place["surface"].translation[2]
        course.observe(course.pose("surface", [0, 0, -z0 - 0.5]))
        course.observe(course.pose("surface", [1.5, 0, -z0 - 0.5]))
        course.observe(course.pose("surface", [2.5, 0, -z0 + 0.5]))   # top above margin outside
        self.assertEqual([e["type"] for e in course.events if e["id"] == "surface:scoring_ended"],
                         ["scoring_ended"])
        before = dict(course.scores)
        for x in (1, 0.5, -0.5, -1):
            course.observe(course.pose("slalom", [x, -0.5, -0.3], frame="slalom_front"))
        for _ in range(6):
            course.observe(rise)
        self.assertEqual(course.scores, before)
        self.assertEqual(len(course.kinds("breach")), 1)

    def test_breach_needs_prior_submersion(self):
        course = Course(self.resolved, ["gate", "surface"])
        course.gate()
        z0 = course.place["surface"].translation[2]
        for _ in range(3):
            course.observe(course.pose("surface", [2.5, 0, -z0 + 0.5]))
        self.assertEqual(course.kinds("breach"), [])

    def facing(self, course, icon, dwell_ticks=6):
        target = course.frames["surface", icon]["position_m"]
        yaw = math.atan2(target[1], target[0]) + math.radians(-45.273547)
        return self.dive_and_surface(course, yaw=yaw, dwell_ticks=dwell_ticks)

    def test_facing_rows_and_basket_count_bonus(self):
        expected = {"compass": 400, "hammer_and_wrench": 400, "buoy": 200, "sos": 200}
        for icon, value in expected.items():
            with self.subTest(icon=icon):
                course = Course(self.resolved, ["gate", "surface"])
                course.gate()
                self.facing(course, icon, dwell_ticks=10)
                self.assertEqual(course.scores["facing"], value)
                self.assertEqual(course.kinds("facing_reached")[0]["data"]["target"], icon)
        # Facing needs 0.5 s of steady facing after the 0.5 s surface dwell (the first sighting restarts it).
        course = Course(self.resolved, ["gate", "surface"])
        course.gate()
        self.facing(course, "compass", dwell_ticks=7)
        self.assertNotIn("facing", course.scores)
        self.assertEqual(course.scores["surface"], 800)
        course.observe(course.pose("surface", [0.0, 0.0, 0.0]))    # lost sight: nothing to award
        self.assertNotIn("facing", course.scores)

    def test_facing_count_uses_baskets_and_rescue_icons(self):
        for count, icon, value in ((1, "compass", 700), (1, "hammer_and_wrench", 400),
                                   (2, "hammer_and_wrench", 700), (3, "hammer_and_wrench", 700),
                                   (2, "compass", 400)):
            with self.subTest(count=count, icon=icon):
                course = Course(self.resolved, ["gate", "surface", "table"])
                course.gate()
                for index in range(count):
                    course.submit(basket(f"p{index}", "helmet_basket", "helmet_basket"))
                self.facing(course, icon, dwell_ticks=10)
                self.assertEqual(course.scores["facing"], value)
        rescue = Course(self.resolved, ["gate", "surface", "table"])
        rescue.gate(y=0.75)
        rescue.submit(basket("p0", "warning_basket", "warning_basket"))
        self.facing(rescue, "buoy", dwell_ticks=10)
        self.assertEqual(rescue.scores["facing"], 700)

    def test_facing_target_must_be_within_fifteen_degrees(self):
        course = Course(self.resolved, ["gate", "surface"])
        course.gate()
        target = course.frames["surface", "compass"]["position_m"]
        yaw = math.atan2(target[1], target[0]) + math.radians(-45.273547) + math.radians(16)
        self.dive_and_surface(course, yaw=yaw, dwell_ticks=12)
        self.assertEqual(course.scores["surface"], 800)
        self.assertNotIn("facing", course.scores)

    def test_object_held_while_surfacing(self):
        course = Course(self.resolved, ["gate", "surface", "table"])
        course.gate()
        course.submit(grasp("pill"))
        self.dive_and_surface(course, dwell_ticks=6)
        self.assertEqual(course.scores["objects_surface"], 400)
        course.submit(release("pill"))
        course.submit(grasp("plug"))
        self.assertEqual(course.scores["objects_surface"], 800)
        # Released (or slipped) props no longer count.
        other = Course(self.resolved, ["gate", "surface", "table"])
        other.gate()
        other.submit(grasp("pill"))
        other.submit(release("pill", reason="slipped"))
        self.dive_and_surface(other, dwell_ticks=6)
        self.assertNotIn("objects_surface", other.scores)
        self.assertEqual(other.scores["surface"], 800)
        # A prop grasped before the gate never counts.
        early = Course(self.resolved, ["gate", "surface", "table"])
        early.submit(grasp("pill"))
        early.gate()
        self.dive_and_surface(early, dwell_ticks=6)
        self.assertNotIn("objects_surface", early.scores)


class TurnTests(Base):
    def spin(self, course, total_deg, *, step_deg=30, radius=1.0, start_yaw=0.0):
        """Rotate the reference frame in place near the table origin, then hold 1.25 s."""
        table = course.place["table"]
        yaw = start_yaw
        position = table.translation + [radius, 0, 0.5]
        course.observe(_pose(position, quaternion(yaw)))
        steps = int(round(abs(total_deg) / step_deg))
        for _ in range(steps):
            yaw += math.copysign(math.radians(step_deg), total_deg)
            course.observe(_pose(position, quaternion(yaw)))
        for _ in range(10):
            course.observe(_pose(position, quaternion(yaw)))
        return position, yaw

    def prepare(self, count=2):
        course = Course(self.resolved, ["gate", "surface", "table"])
        course.gate()
        for index in range(count):
            course.submit(basket(f"p{index}", "helmet_basket", "helmet_basket"))
        return course

    def test_full_turns_match_basket_count(self):
        for count, degrees, expected in ((2, 720, 1000), (2, 360, 500), (3, 360, 0),
                                         (1, 360, 1000), (2, 700, 1000), (2, -720, 1000),
                                         (2, 630, 500), (1, 60, 0)):
            with self.subTest(count=count, degrees=degrees):
                course = self.prepare(count)
                self.spin(course, degrees, step_deg=20 if degrees == 700 else 30)
                self.assertEqual(course.scores.get("basket_count", 0), expected)
                judged = course.kinds("rotation_judged")
                self.assertEqual(judged[0]["data"]["reason"], "stopped")

    def test_zero_baskets_small_travel_and_gate_gating(self):
        empty = self.prepare(0)
        self.spin(empty, 720)
        self.assertNotIn("basket_count", empty.scores)
        small = self.prepare(1)
        self.spin(small, 15, step_deg=15)
        self.assertEqual(small.kinds("rotation_judged"), [])
        ungated = Course(self.resolved, ["gate", "surface", "table"])
        ungated.submit(basket("p0", "helmet_basket", "helmet_basket"))
        self.spin(ungated, 360)
        self.assertNotIn("basket_count", ungated.scores)

    def test_basket_change_discards_travel_and_reversal_restarts_count(self):
        course = self.prepare(1)
        position, yaw = self.spin(course, 360)
        self.assertEqual(course.scores["basket_count"], 1000)
        # A new basket while turning restarts from the current heading (travel so far is dropped).
        changed = self.prepare(1)
        pos = changed.place["table"].translation + [1.0, 0, 0.5]
        heading = 0.0
        changed.observe(_pose(pos, quaternion(heading)))
        for _ in range(6):
            heading += math.radians(30)
            changed.observe(_pose(pos, quaternion(heading)))
        changed.submit(basket("late", "warning_basket", "warning_basket"))
        for _ in range(3):
            changed.observe(_pose(pos, quaternion(heading)))
        for _ in range(10):
            changed.observe(_pose(pos, quaternion(heading)))
        judged = changed.kinds("rotation_judged")
        self.assertEqual(judged, [])                       # 0 deg of travel after the restart
        # Reversal judges the one-direction travel at the peak, then counts from there.
        reversal = self.prepare(1)
        pos = reversal.place["table"].translation + [1.0, 0, 0.5]
        heading = 0.0
        reversal.observe(_pose(pos, quaternion(heading)))
        for step in [30] * 12 + [-30] * 6:
            heading += math.radians(step)
            reversal.observe(_pose(pos, quaternion(heading)))
        self.assertEqual([e["data"]["reason"] for e in reversal.kinds("rotation_judged")],
                         ["reversed"])
        self.assertEqual(reversal.scores["basket_count"], 1000)

    def test_leaving_the_zone_judges_and_pose_jump_resets(self):
        course = self.prepare(2)
        table = course.place["table"]
        position = table.translation + [1.0, 0, 0.5]
        course.observe(_pose(position, quaternion(0.0)))
        for k in range(1, 25):
            course.observe(_pose(position, quaternion(math.radians(30 * k))))
        self.assertNotIn("basket_count", course.scores)   # still moving: not settled
        course.observe(_pose(table.translation + [2.8, 0, 0.5], quaternion(math.radians(720))))
        course.observe(_pose(table.translation + [4.0, 0, 0.5], quaternion(math.radians(720))))
        self.assertEqual(course.kinds("rotation_judged")[0]["data"]["reason"], "left")
        self.assertEqual(course.scores["basket_count"], 1000)
        fresh = self.prepare(2)
        base = fresh.place["table"].translation + [1.0, 0, 0.5]
        fresh.observe(_pose(base, quaternion(0.0)))
        for k in range(1, 13):
            fresh.observe(_pose(base, quaternion(math.radians(30 * k))))
        fresh.observe(_pose(base + [3.5, 0, 0], quaternion(math.radians(360))))   # > 2 m jump
        self.assertEqual(fresh.kinds("rotation_judged"), [])


class TableRowTests(Base):
    def test_objects_drop_and_baskets(self):
        course = Course(self.resolved, ["gate", "table"])
        course.gate()
        course.submit(grasp("pill"))
        course.submit(release("pill"))
        self.assertEqual(course.scores["objects_drop"], 200)
        course.submit(release("nut_and_bolt"))                       # never grasped
        course.submit(grasp("plug"))
        course.submit(release("plug", reason="slipped"))             # slip is not a release
        self.assertEqual(course.scores["objects_drop"], 200)
        course.submit(grasp("plug"))
        course.submit(release("plug"))
        self.assertEqual(course.scores["objects_drop"], 400)
        course.submit(basket("pill", "helmet_basket", "helmet_basket"))
        self.assertEqual(course.scores["baskets"], 700)
        course.submit(basket("plug", "helmet_basket", "warning_basket"))
        self.assertEqual(course.scores["baskets"], 700 + 500)
        course.submit(basket("plug", "warning_basket", "warning_basket"))          # re-sorted: prop keeps its best
        self.assertEqual(course.scores["baskets"], 700 + 700)
        course.submit(basket("bandage", "table", "helmet_basket"))          # not a basket
        self.assertEqual(course.scores["baskets"], 1400)

    def test_table_rows_need_the_gate_and_reject_bad_events(self):
        course = Course(self.resolved, ["gate", "table"])
        course.submit(grasp("pill"))
        course.submit(release("pill"))
        course.submit(basket("pill", "helmet_basket", "helmet_basket"))
        self.assertEqual(course.scores, {})
        course.gate()
        course.submit(release("pill"))                               # grasped before the gate
        self.assertEqual(course.scores, {"gate": 550})
        with self.assertRaises(ValueError):
            course.runtime.observe_events(course.t + 1, [{"id": "grasp", "type": "attach",
                                                          "task": "ghost", "region": "", "data": {}}])
        with self.assertRaises(ValueError):
            course.runtime.observe_events(course.t + 1, [{"id": "grasp", "type": "attach"}])
        with self.assertRaises(ValueError):
            course.runtime.observe_events(0, [grasp("pill")])        # time went backwards


def matrix(pose):
    rotation = np.column_stack([pose.apply(axis) - pose.translation for axis in np.eye(3)])
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, pose.translation
    return result


@unittest.skipUnless(ORIGINAL.exists() and ORIGINAL_TASKS.exists(),
                     "original simulator checkout not available")
class OriginalComparisonTests(Base):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        spec = importlib.util.spec_from_file_location("original_scoring_step4", ORIGINAL)
        cls.original = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.original)
        config = yaml.safe_load(ORIGINAL_TASKS.read_text())
        cls.geometry = dict(config["scoring"], floor_z=-2.1336,
                            surface_z=config["octagon"]["surface_z"],
                            octagon_apothem=config["octagon"]["apothem"]
                            - config["octagon"]["pipe_radius"])

    def start(self, role="repair", tasks=("gate", "slalom", "surface", "table")):
        course = Course(self.resolved, list(tasks), role=role)
        ledger = self.original.RunScore()
        ledger.start(0, role=role, heading_coin=True, role_coin=True)
        frames = {"gate_repair": matrix(course.pose("gate", [0, -0.75, 0.5]))}
        for name in ("gate", "slalom", "surface", "table"):
            frames[name] = matrix(course.place[name])
        frames["slalom_front"] = matrix(course.pose("slalom", [0, 0, 0], frame="slalom_front"))
        frames["slalom_middle"] = matrix(course.pose("slalom", [0, 0, 0], frame="slalom_middle"))
        frames["slalom_back"] = matrix(course.pose("slalom", [0, 0, 0], frame="slalom_back"))
        frames["octagon"] = frames["surface"]
        for icon in ("compass", "hammer_and_wrench", "buoy", "sos"):
            frames[icon] = matrix(course.pose("surface", [0, 0, 0], frame=icon))
        judge = self.original.CourseJudge(ledger, self.geometry, frames, self.envelope)
        return course, ledger, judge

    def tick(self, course, ledger, judge, pose, held=None):
        course.observe(pose)
        judge.update(matrix(pose), 0.125, held)
        self.compare(course, ledger)

    def compare(self, course, ledger, rows=("gate", "slalom_front", "slalom_middle",
                                           "slalom_back", "surface", "facing", "basket_count",
                                           "objects_surface", "home", "baskets", "objects_drop",
                                           "lights", "bins")):
        for row in rows:
            self.assertEqual(course.scores.get(row, 0), ledger.points[row],
                             (row, course.t, course.scores, ledger.points))

    def test_gate_and_slalom_trajectories(self):
        rng = np.random.default_rng(4)
        for role, gate_y in (("repair", -0.75), ("rescue", 0.75), ("repair", 0.4)):
            course, ledger, judge = self.start(role)
            for x in (3, 2, 1, 0.5, 0, -0.5, -1, -2):
                self.tick(course, ledger, judge, course.pose("gate", [x, gate_y, -0.2]))
            self.assertGreater(ledger.points["gate"], 0)
            for row in ("slalom_front", "slalom_middle", "slalom_back") * 3:
                y, z = rng.uniform(-1.7, 1.7), rng.uniform(-1.6, 1.0)
                roll = rng.uniform(-1, 1)
                xs = (1, 0.5, -0.5, -1) if rng.random() < 0.75 else (-1, -0.5, 0.5, 1)
                for x in xs:
                    self.tick(course, ledger, judge,
                              course.pose("slalom", [x, y, z], frame=row, roll=roll))
            self.assertEqual(sum(course.scores.get(r, 0) for r in
                                 ("slalom_front", "slalom_middle", "slalom_back")),
                             sum(ledger.points[r] for r in
                                 ("slalom_front", "slalom_middle", "slalom_back")))
            self.assertGreater(ledger.points["slalom_front"] + ledger.points["slalom_back"], 0)

    def test_surface_facing_turns_and_breach_trajectory(self):
        for role, icon, count in (("repair", "compass", 1), ("repair", "hammer_and_wrench", 2),
                                  ("rescue", "sos", 1), ("rescue", "buoy", 3),
                                  ("repair", "buoy", 0)):
            with self.subTest(role=role, icon=icon, count=count):
                course, ledger, judge = self.start(role)
                gate_y = -0.75 if role == "repair" else 0.75
                for x in (3, 2, 1, 0.5, 0, -0.5, -1, -2):
                    self.tick(course, ledger, judge, course.pose("gate", [x, gate_y, -0.2]))
                table = course.place["table"]
                base = table.translation + [1.0, 0, 0.5]
                # Turn near the table with a basket change part way, then move to the octagon.
                yaw = 0.0
                self.tick(course, ledger, judge, _pose(base, quaternion(yaw)))
                for k in range(count):
                    ledger.object_event(f"p{k}", "success", "helmet", "helmet", role)
                    course.submit(basket(f"p{k}", "helmet_basket", "helmet_basket"))
                    judge.update(matrix(_pose(base, quaternion(yaw))), 0.0)
                    course.runtime.observe(course.t, _pose(base, quaternion(yaw)))
                for k in range(1, 26):
                    yaw += math.radians(30)
                    self.tick(course, ledger, judge, _pose(base, quaternion(yaw)))
                for _ in range(10):
                    self.tick(course, ledger, judge, _pose(base, quaternion(yaw)))
                target = course.frames["surface", icon]["position_m"]
                heading = math.atan2(target[1], target[0]) + math.radians(-45.273547)
                z0 = course.place["surface"].translation[2]
                held = "pill" if count else None
                if held:
                    ledger.object_event(held, "grasped", "", "helmet", role)
                    course.submit(grasp(held))
                self.tick(course, ledger, judge,
                          course.pose("surface", [0, 0, -z0 - 0.5], yaw=heading - math.radians(-45.273547)),
                          held)
                rise = course.pose("surface", [0.1, 0.1, -z0 - 0.376],
                                   yaw=heading - math.radians(-45.273547))
                for _ in range(12):
                    self.tick(course, ledger, judge, rise, held)
                self.assertEqual(course.scores["surface"], ledger.points["surface"])
                self.assertEqual(course.scores["surface"], 800)
                self.assertGreater(ledger.points["facing"], 0)
                if count:
                    self.assertGreater(ledger.points["basket_count"], 0)
                if count:
                    self.assertEqual(course.scores["objects_surface"], 400)
                # Breach outside the octagon ends scoring in both.
                self.tick(course, ledger, judge, course.pose("surface", [1.4, 0, -z0 - 0.5]), held)
                self.tick(course, ledger, judge,
                          course.pose("surface", [2.6, 0, -z0 + 0.5]), held)
                self.assertTrue(ledger.ended_reason)
                self.tick(course, ledger, judge, rise, held)

    def test_bins_lights_and_table_rows_match_runscore(self):
        traces = [
            [("gate", "repair"), ("drop", 1, "bin_vinyl2"), ("drop", 2, "bin_vinyl1"),
             ("drop", 3, "bin_vinyl3"), ("light", "magnet_target1"),
             ("light", "magnet_target2"), ("light", "magnet_target1")],
            [("drop", 1, "bin_vinyl2"), ("gate", "repair"), ("drop", 2, "bin_vinyl2"),
             ("drop", 3, "bin_vinyl1"), ("light", "magnet_target1")],
            [("gate", "rescue"), ("drop", 1, "bin_vinyl1"), ("drop", 2, "bin_vinyl4"),
             ("drop", 2, "bin_vinyl4")],
            [("gate", "repair"), ("grasp", "pill"), ("release", "pill"), ("grasp", "pill"),
             ("basket", "pill", "helmet_basket", "helmet_basket"), ("grasp", "plug"),
             ("release", "plug", "slipped"), ("release", "plug"),
             ("basket", "plug", "helmet_basket", "warning_basket"), ("basket", "plug", "warning_basket", "warning_basket"),
             ("grasp", "plug"), ("basket", "pill", "warning_basket", "helmet_basket")],
            [("grasp", "pill"), ("gate", "repair"), ("release", "pill"),
             ("basket", "pill", "helmet_basket", "helmet_basket"), ("release", "nut_and_bolt")],
        ]
        classes = {"bin_vinyl1": "blood", "bin_vinyl2": "fire", "bin_vinyl3": "fire",
                   "bin_vinyl4": "blood"}
        for index, trace in enumerate(traces):
            with self.subTest(trace=index):
                role = "rescue" if ("gate", "rescue") in trace else "repair"
                ledger = self.original.RunScore()
                ledger.start(0, role=role, heading_coin=True, role_coin=True)
                hook = Course(self.resolved, ["gate", "table"], role=role)
                for step in trace:
                    kind = step[0]
                    if kind == "gate":
                        ledger.gate(step[1], -1 if step[1] == "repair" else 1)
                        hook.gate(y=-0.75 if step[1] == "repair" else 0.75)
                    elif kind == "drop":
                        ledger.release_payload("dropper", step[1])
                        ledger.payload_result("dropper", step[1], "success" if classes[step[2]]
                                              == ledger.target_class else "wrong_target", step[2],
                                              classes[step[2]])
                        self.feed_bins(hook, step[1], step[2], classes[step[2]])
                    elif kind == "light":
                        ledger.light(step[1])
                        hook.events.extend(self.feed_light(hook, step[1]))
                    elif kind == "grasp":
                        ledger.object_event(step[1], "grasped", "", "", role)
                        hook.submit(grasp(step[1]))
                    elif kind == "release":
                        ledger.object_event(step[1], "released" if len(step) < 3 else "slipped",
                                            "", "", role)
                        hook.submit(release(step[1], **({"reason": step[2]} if len(step) > 2 else {})))
                    else:
                        short = [name.removesuffix("_basket") for name in step[2:4]]
                        ledger.object_event(step[1], "success" if short[0] == short[1]
                                            else "wrong_target", short[0], short[1], role)
                        hook.submit(basket(step[1], step[2], step[3]))
                    self.compare(hook, ledger)

    @staticmethod
    def feed_bins(course, ident, crate, crate_class):
        runtime = course.runtime
        runtime._evaluate([
            {"id": "payload_released", "type": "payload_released", "task": "bins",
             "region": crate, "time_ns": course.t,
             "data": {"projectile_id": ident, "mechanism_type": "dropper"}}])
        runtime._evaluate([
            {"id": f"{crate}_inside", "type": "payload_landing", "task": "bins", "region": crate,
             "time_ns": course.t, "data": {"projectile_id": ident, "mechanism_type": "dropper",
                                           "outcome": "inside", "detail": "floor",
                                           "region_class": crate_class}}])

    @staticmethod
    def feed_light(course, target):
        return plain(course.runtime._evaluate([
            {"id": f"{target}_on", "type": "activate", "task": "bins", "region": target,
             "time_ns": course.t, "data": {}}]))


if __name__ == "__main__":
    unittest.main()
