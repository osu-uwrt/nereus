#!/usr/bin/env python3
"""Scripted pack task acceptance. This first slice checks gate/home and judge replay."""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping
from pathlib import Path

from robotics_platform.packs import resolve_scenario
from robotics_platform.task_runtime import TaskRuntime, _placement, _pose

ROOT = Path(__file__).resolve().parents[3]


def plain(value):
    if isinstance(value, Mapping):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', type=Path,
                        default=ROOT / 'content/packs/scenarios/talos_uwrt/scenario.yaml')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    resolved = resolve_scenario(args.scenario)
    resolved.dump(args.output / 'resolved.json')
    runtime = TaskRuntime(resolved, task_ids=['gate'])
    gate = _placement(next(p for p in resolved.scenario['task_placements'] if p['task'] == 'gate'))
    x_trace = [4, 3, 2, 1, .5, 0, -.5, -1, -2, -3, -4, -3, -2, -1, -.5, 0, .5, 1, 2, 3, 4]
    cases = [('repair_and_home', -.75, -.2, {'gate': 550, 'home': 300}),
             ('wrong_role_and_home', .75, -.2, {'gate': 400, 'home': 300}),
             ('outside_width', 1.5, -.2, {}), ('above_opening', -.75, .75, {}),
             ('below_floor', -.75, -1.4, {})]
    results, traces = {}, {}
    for name, y, z, expected in cases:
        replay = []
        for _ in range(2):
            runtime.reset()
            records = []
            for i, x in enumerate(x_trace):
                pose = gate.compose(_pose([x, y, z], [1, 0, 0, 0]))
                events = runtime.observe(i * 100_000_000, pose)
                records.append({'time_ns': i * 100_000_000,
                                'position_m': pose.translation.tolist(),
                                'orientation_wxyz': pose.orientation_wxyz.tolist(),
                                'events': plain(events)})
            snapshot = plain(runtime.snapshot())
            assert snapshot['scores'] == expected, (name, snapshot['scores'], expected)
            replay.append({'records': records, 'snapshot': snapshot})
        assert replay[0] == replay[1], f'{name}: reset replay differs'
        traces[name] = replay[0]
        results[name] = {'scores': expected, 'reset_replay_equal': True}
    (args.output / 'traces.json').write_text(json.dumps(traces, indent=2, allow_nan=False)+'\n')
    summary = {'passed': True, 'scope': 'gate/home scoring and task-observer reset only',
               'not_proven': ['dynamic mechanism contacts', 'whole-system reset',
                              'remaining 2026 tasks', 'real-stack mission'], 'cases': results}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
