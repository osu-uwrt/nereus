"""Portal geometry against independent original traces and standalone spatial contracts.

REFERENCE_TRACES were captured by running CourseJudge.update/finish_gate_attempt from
riptide_simulator 07647eebe706f96ea7b76db3cc9802735a146698, with the scripted local poses
below. Source: c_simulator/tasks/2026/behavior/scoring.py, SHA256
60f6450c64830f6390d0a9eb272e87f543e4cc740dfcf41952e76a8ece177d19.
Capture recorded initial gate() calls, home awards, and gate_attempt.turns before closure.
The original judge used the eight vertices below, gate at world Z=-.75, repair sign Y=-.75,
floor=-2.1336, surface=0, its configured 3m approach/2m jump bounds, and an active run.
Unrelated slalom/table frames were at [100,100,0], octagon apothem100 to isolate passage.
These checked traces require neither the original repository nor its Python modules.
"""

import copy
import dataclasses
import itertools
import math
import unittest

import numpy as np
from robotics_platform import _native as native
from robotics_platform.task_regions import PortalTracker

PARAMETERS = {
    "plane": {"axis": "x", "offset_m": 0.0},
    "bounds_local": {"abs_y_lt_m": 1.5, "z_lt_m": .75},
    "world_floor_clearance": True,
    "crossing_reference": "robot_reference_origin",
    "fit_checks": ["envelope_at_crossing_point_with_current_orientation",
                   "envelope_at_completion"],
    "traversal": "full_envelope", "approach_radius_m": 3., "max_pose_step_m": 2.,
}
VERTICES = np.array(list(itertools.product((-.2, .2), (-.25, .25), (-.25, .25))))
ZERO = (0., 0., 0.)


def pose(position=(0, 0, 0), rpy=(0, 0, 0)):
    roll, pitch, yaw = (math.radians(v) / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    value = native.Pose()
    value.translation = position
    value.orientation_wxyz = [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                              cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]
    return value


def line(xs, y=-.5, z=0):
    return [(x, y, z, 0, 0, 0) for x in xs]


BASE = line((1, .3, 0, -.1, -.3))
CASES = {
    "forward": BASE + line((-1.6, -3.1)),
    "return": BASE + line((-.1, 0, .1, .3, 1.6, 3.1)),
    "wide": line((1, .3, 0, -.1, -.3, -1.6, -3.1), y=1.3),
    "top": line((1, .3, 0, -.1, -.3, -1.6, -3.1), y=0, z=.6),
    "floor": line((1, .3, 0, -.1, -.3, -1.6, -3.1), y=0, z=-1.2),
    "oscillate": line((1, .1, -.1, .1, -.1, .3)),
    "jump": line((1, -1.1, -2, -3.1)),
    "yaw": BASE + [(-.5, -.5, 0, 0, 0, y) for y in (45, 90, 135, 180)] +
    [(x, -.5, 0, 0, 0, 180) for x in (-1.6, -3.1)],
    "reversal": BASE + [(-.5, -.5, 0, 0, 0, y) for y in (45, 90, 45, 0)] + line((-1.6, -3.1)),
    "roll": BASE + [(-.5, -.5, 0, r, 0, 0) for r in (30, 60, 90)] +
    [(x, -.5, 0, 90, 0, 0) for x in (-1.6, -3.1)],
    "jump_after_pass": BASE + [(-.5, -.5, 0, 0, 0, y) for y in (45, 90)] +
    [(10, -.5, 0, 0, 0, 90)],
    "initial_overlap": line((.1, -.1, -.3, -1.6, -3.1)),
}
REFERENCE_TRACES = {
    "forward": [(4, "pass", None), (6, "finish", ZERO)],
    "return": [(4, "pass", None), (8, "home", None), (10, "finish", ZERO)],
    "wide": [], "top": [], "floor": [], "oscillate": [], "jump": [],
    "yaw": [(4, "pass", None), (10, "finish", (0., 0., math.pi))],
    "reversal": [(4, "pass", None), (10, "finish", ZERO)],
    "roll": [(4, "pass", None), (9, "finish", (math.pi / 2, 0., 0.))],
    "jump_after_pass": [(4, "pass", None), (7, "finish", (0., 0., math.pi / 2))],
    "initial_overlap": [(2, "pass", None), (4, "finish", ZERO)],
}


def tracker(parameters=None, vertices=VERTICES, placement=None, floor=-2.1336):
    return PortalTracker(parameters or PARAMETERS, placement or pose([0, 0, -.75]), vertices, floor)


def feed(model, path, placement=None):
    world_task = placement or pose([0, 0, -.75])
    events = []
    for i, item in enumerate(path):
        events.extend(model.observe((i + 1) * 20_000_000,
                                    world_task.compose(pose(item[:3], item[3:]))))
    return events


class OriginalPortalTraceTests(unittest.TestCase):
    def test_original_traversal_rejection_reset_and_net_rotation_traces(self):
        for name, path in CASES.items():
            with self.subTest(name=name):
                records = []
                for event in feed(tracker(), path):
                    index = event.time_ns // 20_000_000 - 1
                    if event.kind == "pass_through":
                        kind = "pass" if event.from_side == "positive" else "home"
                        records.append((index, kind, None))
                        np.testing.assert_allclose(event.crossing_point_local, [0, -.5, 0], atol=1e-12)
                        self.assertEqual(event.attempt_id, 1)
                        self.assertEqual(event.to_side, "negative" if kind == "pass" else "positive")
                    elif event.from_side == "positive":
                        records.append((index, "finish", event.rotation_vector_body))
                expected = REFERENCE_TRACES[name]
                self.assertEqual(len(records), len(expected))
                for actual, reference in zip(records, expected):
                    self.assertEqual(actual[:2], reference[:2])
                    if reference[2] is not None:
                        np.testing.assert_allclose(actual[2], reference[2], rtol=0, atol=1e-12)


class PortalContractTests(unittest.TestCase):
    def test_both_directions_preserve_their_last_passage_for_attempt_closure(self):
        events = feed(tracker(), CASES["return"])
        ended = [event for event in events if event.kind == "attempt_finished"]
        self.assertEqual([event.from_side for event in ended], ["positive", "negative"])
        self.assertEqual({event.attempt_id for event in events}, {1})
        with self.assertRaises(dataclasses.FrozenInstanceError):
            ended[0].kind = "changed"

    def test_explicit_finish_flushes_once_and_reset_discards_motion_and_time(self):
        model = tracker()
        events = feed(model, CASES["yaw"][:-2])
        ended = model.finish_attempt(500_000_000)
        self.assertEqual(len(ended), 1)
        self.assertEqual(ended[0].attempt_id, events[0].attempt_id)
        np.testing.assert_allclose(ended[0].rotation_vector_body, [0, 0, math.pi], atol=1e-12)
        self.assertEqual(model.finish_attempt(500_000_000), ())
        model.reset()
        self.assertEqual(feed(model, BASE)[0].attempt_id, 1)
        model.reset()
        self.assertEqual(model.finish_attempt(0), ())

    def test_spatial_placement_and_nonzero_plane_offset_preserve_local_passage(self):
        placement = pose([11, -4, -10], [0, 0, 37])
        events = feed(tracker(placement=placement, floor=-20), CASES["forward"], placement)
        np.testing.assert_allclose(events[0].crossing_point_local, [0, -.5, 0], atol=1e-12)
        self.assertAlmostEqual(events[0].envelope_top_world, -9.75)
        p = copy.deepcopy(PARAMETERS)
        p["plane"]["offset_m"] = 4
        shifted = [(x + 4, y, z, r, pitch, yaw) for x, y, z, r, pitch, yaw in CASES["forward"]]
        offset_events = feed(tracker(p), shifted)
        self.assertEqual([event.kind for event in offset_events], ["pass_through", "attempt_finished"])
        np.testing.assert_allclose(offset_events[0].crossing_point_local, [4, -.5, 0], atol=1e-12)

    def test_strict_bounds_and_optional_floor_clearance(self):
        for y, z in ((1.25, 0), (0, .5), (0, -1.0)):
            with self.subTest(y=y, z=z):
                self.assertEqual(feed(tracker(floor=-2), line((.5, 0, -.3), y=y, z=z)), [])
        p = copy.deepcopy(PARAMETERS)
        p["world_floor_clearance"] = False
        self.assertEqual(feed(tracker(p), line((.5, 0, -.3), z=-1.2))[0].kind, "pass_through")

    def test_fit_check_selection_and_completion_orientation_are_explicit(self):
        crossing_good = [(.5, 0, 0, 0, 0, 0), (-.1, 1.4, 0, 0, 0, 0), (-.3, 1.4, 0, 0, 0, 0)]
        completion_good = [(.5, 1.6, 0, 0, 0, 0), (-.1, 1.6, 0, 0, 0, 0), (-.3, 0, 0, 0, 0, 0)]
        for path, check in ((crossing_good, "envelope_at_crossing_point_with_current_orientation"),
                            (completion_good, "envelope_at_completion")):
            self.assertEqual(feed(tracker(), path), [])
            p = copy.deepcopy(PARAMETERS)
            p["fit_checks"] = [check]
            self.assertEqual(len(feed(tracker(p), path)), 1)
        long = np.array(list(itertools.product((-.7, .7), (-.05, .05), (-.1, .1))))
        rotated = [(1.2, 1., 0, 0, 0, 0), (0, 1., 0, 0, 0, 0), (-.2, 1., 0, 0, 0, 90)]
        p = copy.deepcopy(PARAMETERS)
        p["fit_checks"] = ["envelope_at_crossing_point_with_current_orientation"]
        self.assertEqual(feed(tracker(p, long), rotated), [])

    def test_reference_origin_traversal_does_not_wait_for_envelope_completion(self):
        path = line((.5, 0, -.1))
        self.assertEqual(feed(tracker(), path), [])
        p = copy.deepcopy(PARAMETERS)
        p["traversal"] = "reference_origin"
        self.assertEqual(len(feed(tracker(p), path)), 1)

    def test_pass_outside_approach_has_no_style_attempt_and_jump_ids_do_not_repeat(self):
        long = np.array(list(itertools.product((-4., 4.), (-.1, .1), (-.1, .1))))
        events = feed(tracker(vertices=long), line((5, 4, 3, 2, 1, 0, -1, -2, -3, -4.1)))
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].kind, "pass_through")
        self.assertEqual(events[0].attempt_id, 0)
        model = tracker()
        first = feed(model, BASE)[0]
        model.observe(200_000_000, pose([10, -.5, -.75]))
        model.observe(220_000_000, pose([1, -.5, -.75]))
        second = model.observe(240_000_000, pose([-.3, -.5, -.75]))[0]
        self.assertGreater(second.attempt_id, first.attempt_id)

    def test_configuration_pose_and_envelope_are_owned(self):
        p, vertices, placement = copy.deepcopy(PARAMETERS), VERTICES.copy(), pose([0, 0, -.75])
        model = tracker(p, vertices, placement)
        p["fit_checks"].clear()
        p["bounds_local"]["abs_y_lt_m"] = .1
        vertices[:] = 1e6
        placement.translation = [100, 0, 0]
        observed = pose([.5, -.5, -.75])
        model.observe(1, observed)
        observed.translation = [-100, 0, 0]
        self.assertEqual(len(model.observe(2, pose([-.3, -.5, -.75]))), 1)

    def test_invalid_observations_leave_time_crossing_and_rotation_unchanged(self):
        model, reference = tracker(), tracker()
        first = pose([.5, -.5, -.75])
        model.observe(20, first)
        reference.observe(20, first)
        bad_pose = pose([float("nan"), 0, 0])
        for time, sample in ((19, first), (True, first), (20., first), (-1, first), (30, bad_pose)):
            with self.assertRaises(ValueError):
                model.observe(time, sample)
        with self.assertRaises(ValueError):
            model.finish_attempt(19)
        for time, x in ((30, 0), (40, -.3)):
            self.assertEqual(model.observe(time, pose([x, -.5, -.75])),
                             reference.observe(time, pose([x, -.5, -.75])))
        self.assertEqual(model.finish_attempt(50), reference.finish_attempt(50))

    def test_rejects_unsupported_or_invalid_geometry_before_observing(self):
        for key, value in (("traversal", "unknown"), ("crossing_reference", "mesh"),
                           ("world_floor_clearance", 1), ("max_pose_step_m", 0),
                           ("approach_radius_m", float("inf")), ("fit_checks", []),
                           ("fit_checks", ["envelope_at_completion"] * 2)):
            p = copy.deepcopy(PARAMETERS)
            p[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                tracker(p)
        p = copy.deepcopy(PARAMETERS)
        p["plane"]["axis"] = "y"
        with self.assertRaisesRegex(ValueError, "only x"):
            tracker(p)
        for vertices in ([], [[1, 2]], [[float("nan"), 0, 0]], [1, 2, 3]):
            with self.assertRaises(ValueError):
                tracker(vertices=vertices)


if __name__ == "__main__":
    unittest.main()
