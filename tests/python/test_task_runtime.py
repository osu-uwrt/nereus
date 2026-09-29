"""Pack geometry/scoring composition, source ownership, and deterministic replay."""

import copy
import hashlib
import json
import math
import tempfile
import unittest
from pathlib import Path

from robotics_platform.packs import resolve_scenario
from robotics_platform.task_runtime import TaskRuntime, _placement, _pose

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / 'content/packs/scenarios/talos_uwrt/scenario.yaml'


def plain(value):
    from collections.abc import Mapping
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    return value


class TaskRuntimeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.resolved = resolve_scenario(SCENARIO)
        # These tests exercise observation/scoring from boot; the pack default (a run starts on the
        # operator's "start") is covered by test_session_run_control.
        cls.resolved.scenario["run"]["auto_start"] = True

    def setUp(self):
        self.r = copy.deepcopy(self.resolved)
        self.task = TaskRuntime(self.r, task_ids=['gate'])
        self.gate = _placement(self.r.scenario['task_placements'][0])

    def pose(self, x, y=-.75, z=-.2, roll=0):
        return self.gate.compose(_pose([x, y, z], [math.cos(roll/2), math.sin(roll/2), 0, 0]))

    def trace(self, *, y=-.75, z=-.2):
        events = []
        for i, x in enumerate([4, 3, 2, 1, .5, 0, -.5, -1, -2, -3, -4,
                                -3, -2, -1, -.5, 0, .5, 1, 2, 3, 4]):
            events.extend(self.task.observe(i * 100_000_000, self.pose(x, y, z)))
        return plain(events)

    def test_real_pack_crossing_home_and_exact_reset_replay(self):
        first = self.trace()
        self.assertEqual(dict(self.task.snapshot()['scores']), {'gate': 550, 'home': 300})
        self.assertEqual([e['id'] for e in first if e['type'] == 'pass_through'],
                         ['forward_pass', 'reverse_pass'])
        self.task.reset()
        self.assertEqual(dict(self.task.snapshot()['scores']), {})
        self.assertEqual(self.trace(), first)

    def test_bounds_floor_and_wrong_role_are_observed(self):
        for y, z in ((1.5, -.2), (-.75, .75), (-.75, -1.4)):
            with self.subTest(y=y, z=z):
                self.task.reset()
                self.trace(y=y, z=z)
                self.assertEqual(dict(self.task.snapshot()['scores']), {})
        self.task.reset()
        self.trace(y=.75)
        self.assertEqual(dict(self.task.snapshot()['scores']), {'gate': 400, 'home': 300})

    def test_stop_flushes_style_and_start_clears_previous_run(self):
        for i, x in enumerate([2, 1, 0, -1]):
            self.task.observe(i, self.pose(x))
        for i in range(1, 21):
            self.task.observe(3+i, self.pose(-1, roll=math.pi*i/20))
        self.task.stop(24)
        self.assertEqual(self.task.snapshot()['scores']['gate'], 950)
        self.assertFalse(self.task.snapshot()['run']['running'])
        self.task.start(30)
        self.assertEqual(dict(self.task.snapshot()['scores']), {})
        self.assertEqual(self.task.snapshot()['history'], ())
        self.assertEqual(self.task.snapshot()['run']['started_ns'], 30)
        with self.assertRaises(ValueError):
            self.task.observe(29, self.pose(0))

    def test_snapshot_and_events_are_owned_recursive_readonly_data(self):
        snapshot = self.task.snapshot()
        with self.assertRaises(TypeError):
            snapshot['run']['options']['role'] = 'rescue'
        with self.assertRaises(TypeError):
            snapshot['tasks']['gate']['frames']['gate_repair']['position_m'][0] = 99
        self.trace()
        self.assertEqual(dict(snapshot['scores']), {})
        self.assertEqual(snapshot['history'], ())
        self.r.run_options['role'] = 'rescue'
        self.task.reset()
        self.assertEqual(self.task.snapshot()['run']['options']['role'], 'repair')

    def test_no_implicit_partial_support_or_unknown_contacts(self):
        unsupported = copy.deepcopy(self.r)
        unsupported.task_definitions[1]['regions'][0]['type'] = 'unsupported_region'
        with self.assertRaisesRegex(ValueError, 'unsupported region'):
            TaskRuntime(unsupported)
        for identifiers in ([], ['missing'], ['gate', 'gate']):
            with self.assertRaises(ValueError):
                TaskRuntime(self.r, task_ids=identifiers)
        with self.assertRaises(ValueError):
            self.task.observe(1, self.pose(0), contacts=[('gate', 'unknown')])
        self.task.observe(1, self.pose(2))  # invalid input did not poison the observer

    def test_generic_event_points_contact_entry_limit_and_negative_points(self):
        self.r.tasks['scoring_hooks'] = []
        gate = self.r.task_definitions[0]
        gate['id'] = 'arbitrary_course'
        self.r.scenario['task_placements'][0]['task'] = gate['id']
        gate['scoring'] = [{'id': 'contact_penalty', 'type': 'event_points',
                           'parameters': {'event': 'structure_contact', 'points': -7, 'max_awards': 2}}]
        task = TaskRuntime(self.r, task_ids=[gate['id']])
        pair = (gate['id'], 'gate_structure')
        for i, contacts in enumerate([[pair], [pair], [], [pair], [], [pair]]):
            task.observe(i, self.pose(4), contacts=contacts)
        self.assertEqual(dict(task.snapshot()['scores']), {'arbitrary_course/contact_penalty': -14})
        self.assertEqual(len(task.snapshot()['history']), 3)
        task.reset()
        task.observe(0, self.pose(4), contacts=[pair])
        self.assertEqual(dict(task.snapshot()['scores']), {'arbitrary_course/contact_penalty': -7})

    def hook_runtime(self, directory, body):
        path = Path(directory) / 'custom.py'
        path.write_text('def evaluate(state, events, parameters):\n' + body)
        self.r.scenario['tasks'] = str(path.parent)
        self.r.tasks['scoring_hooks'] = [{'module': 'custom', 'function': 'evaluate', 'parameters': {}}]
        self.r.sources.append(path)
        self.r.source_sha256[path] = hashlib.sha256(path.read_bytes()).hexdigest()
        return TaskRuntime(self.r, task_ids=['gate']), path

    def test_hook_validation_and_failure_require_reset(self):
        bodies = ["    state['run']['options']['role'] = 'x'\n",
                  "    return {'scores': [{'row': 'bad', 'points': True}], 'events': []}\n",
                  "    return {'scores': [], 'events': [{'bad': 1}]}\n",
                  "    return {'scores': [], 'events': [float('nan')]}\n"]
        for body in bodies:
            with self.subTest(body=body), tempfile.TemporaryDirectory() as directory:
                self.r = copy.deepcopy(self.resolved)
                task, _ = self.hook_runtime(directory, body)
                with self.assertRaises((ValueError, TypeError)):
                    task.observe(0, self.pose(2), contacts=[('gate', 'gate_structure')])
                self.assertEqual(dict(task.snapshot()['scores']), {})
                self.assertEqual(task.snapshot()['history'], ())
                with self.assertRaisesRegex(RuntimeError, 'reset'):
                    task.observe(1, self.pose(2))
                task.reset()
                task.observe(0, self.pose(2))

    def test_changed_hook_rejected_and_existing_observer_owns_verified_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            task, path = self.hook_runtime(directory, "    return {'scores': [], 'events': []}\n")
            path.write_text("raise RuntimeError('edited')\n")
            with self.assertRaisesRegex(ValueError, 'sources changed'):
                TaskRuntime(self.r, task_ids=['gate'])
            task.reset()  # recreates only verified source, never executes the edited file
            task.observe(0, self.pose(2), contacts=[('gate', 'gate_structure')])
            json.dumps(plain(task.snapshot()), allow_nan=False)


if __name__ == '__main__':
    unittest.main()
