"""Swept panel geometry, pack placement and physical-release scoring integration."""

import importlib.util
import os
import unittest
from pathlib import Path

import numpy as np
from robotics_platform.packs import resolve_scenario
from robotics_platform.task_projectiles import PerforatedPanel
from robotics_platform.task_runtime import TaskRuntime, _placement, _pose

ROOT = Path(__file__).resolve().parents[2]


class PanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.resolved = resolve_scenario(ROOT / 'content/packs/scenarios/talos_uwrt/scenario.yaml')
        cls.task = next(t for t in cls.resolved.task_definitions if t['id'] == 'torpedo')
        cls.parameters = cls.task['regions'][0]['parameters']
        cls.placement = _placement(next(p for p in cls.resolved.scenario['task_placements']
                                        if p['task'] == 'torpedo'))

    def setUp(self):
        self.panel = PerforatedPanel(self.parameters, self.placement)

    def point(self, x, y=0, z=0):
        return self.placement.apply([x, y, z])

    def axis(self, vector=(-1, 0, 0)):
        return self.placement.apply(vector) - self.placement.translation

    def intersect(self, start, end, radius=.013, axis=(-1, 0, 0)):
        return self.panel.intersect(self.point(*start), self.point(*end), self.axis(axis), radius)

    def test_every_pack_hole_and_reverse_crossing(self):
        for hole in self.parameters['holes']:
            y, z = (np.asarray(hole['uv']) - .5) * 2*self.parameters['half_size_m']
            for start, end in ((2, -2), (-2, 2)):
                hit = self.intersect((start, y, z), (end, y, z))
                self.assertEqual((hit.outcome, hit.hole_id, hit.hole_class, hit.hole_size),
                                 ('pass', hole['id'], hole['class'], hole['size']))
                np.testing.assert_allclose(hit.point_local, [0, y, z], atol=1e-14)

    def test_blocked_miss_oblique_clearance_and_no_repeat_plane_start(self):
        self.assertEqual(self.intersect((1, 0, 0), (-1, 0, 0)).outcome, 'blocked')
        self.assertIsNone(self.intersect((1, 2, 0), (-1, 2, 0)))
        self.assertIsNone(self.intersect((1, 0, 0), (.1, 0, 0)))
        # Test exact plane start in local/identity frame: no second hit on same crossing.
        local = PerforatedPanel(self.parameters, _pose([0, 0, 0], [1, 0, 0, 0]))
        self.assertIsNone(local.intersect([0, 0, 0], [-1, 0, 0], [-1, 0, 0], .013))
        hole = self.parameters['holes'][0]
        y, z = (np.asarray(hole['uv']) - .5) * 2*self.parameters['half_size_m']
        self.assertEqual(self.intersect((1, y, z), (-1, y, z), axis=(0, 1, 0)).outcome, 'blocked')
        self.assertAlmostEqual(self.panel.release_distance(self.point(.4572, y, z)), .4572)

    def test_invalid_inputs_and_offset_plane(self):
        for axis, radius in (((0, 0, 0), .01), ((1, 0, 0), 0), ((1, 0, 0), float('nan'))):
            with self.assertRaises(ValueError):
                self.intersect((1, 0, 0), (-1, 0, 0), axis=axis, radius=radius)
        p = {**self.parameters, 'plane': {'axis': 'x', 'offset_m': 2}}
        panel = PerforatedPanel(p, _pose([0, 0, 0], [1, 0, 0, 0]))
        self.assertEqual(panel.intersect([3, 0, 0], [1, 0, 0], [-1, 0, 0], .01).point_local,
                         (2, 0, 0))
        self.assertEqual(panel.release_distance([3, 0, 0]), 1)

    @unittest.skipUnless(os.environ.get('RP_PAYLOAD_REFERENCE'), 'optional pinned original model')
    def test_random_geometry_matches_original_model(self):
        path = os.environ['RP_PAYLOAD_REFERENCE']
        spec = importlib.util.spec_from_file_location('original_payload_geometry', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cfg = {'panel_half_size': self.parameters['half_size_m'], 'radius': .013,
               'holes': [{**h, 'name': h['id']} for h in self.parameters['holes']]}
        panel = PerforatedPanel(self.parameters, _pose([0, 0, 0], [1, 0, 0, 0]))
        rng = np.random.default_rng(387)
        for _ in range(2000):
            start, end = rng.uniform(-.5, .5, (2, 3))
            axis = rng.normal(size=3)
            axis /= np.linalg.norm(axis)
            expected = module.torpedo_contact(start, end, axis, cfg)
            actual = panel.intersect(start, end, axis, .013)
            if expected is None:
                self.assertIsNone(actual)
            else:
                self.assertEqual(actual.outcome, expected[0])
                if actual.outcome == 'pass':
                    self.assertEqual((actual.hole_id, actual.hole_class), expected[1:3])
                np.testing.assert_allclose(actual.point_local, expected[3], atol=1e-14)

    def test_runtime_hit_binding_repeat_physical_collision_and_reset(self):
        runtime = TaskRuntime(self.resolved)
        hole = self.parameters['holes'][0]
        y, z = (np.asarray(hole['uv']) - .5) * 2*self.parameters['half_size_m']
        for _ in range(2):
            release = runtime.release_projectile(0, 1, 'launcher', self.point(1, y, z), .013)
            self.assertAlmostEqual(release[0]['data']['release_distance_m'], 1)
            events = runtime.observe_projectile(1, 1, self.point(1, y, z), self.point(-1, y, z), self.axis())
            hit = next(e for e in events if e['type'] == 'hit')
            self.assertEqual(hit['id'], 'hole_pass')
            self.assertFalse(hit['data']['stop_projectile'])
            again = runtime.observe_projectile(2, 1, self.point(-1, y, z),
                                               self.point(1, y, z), self.axis())
            self.assertEqual([e['type'] for e in again], ['hit'])
            blocked = runtime.observe_projectile(2, 1, self.point(1), self.point(-1), self.axis())
            self.assertTrue(blocked[0]['data']['stop_projectile'])
            for identifier, end in ((True, self.point(-1)), (1.0, self.point(-1)),
                                    (1, [float('nan'), 0, 0])):
                with self.assertRaises(ValueError):
                    runtime.observe_projectile(3, identifier, self.point(1), end, self.axis())
            # Released before gate: the physical hit cannot award competition points.
            self.assertEqual(dict(runtime.snapshot()['scores']), {})
            with self.assertRaises(ValueError):
                runtime.release_projectile(2, 1, 'launcher', self.point(1), .013)
            runtime.reset()


if __name__ == '__main__':
    unittest.main()
