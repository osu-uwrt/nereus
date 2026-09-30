"""Talos UWRT bridge pack: mechanism services/streams and coordinated resets through the session.

Uses real packs and ROS message classes; no rclpy context or middleware. Image streams are
removed so no GPU is needed (camera coordination is covered by the camera tests).
"""

from __future__ import annotations

import copy
import dataclasses
import unittest
from pathlib import Path
from typing import Any

import numpy as np

try:
    from riptide_msgs2.msg import ActuatorStatus
    from nereus.pack_runtime import create_runtime
    from nereus.packs import resolve_scenario
    from nereus_ros.core import BridgeCore
    from nereus_ros.mapping import service_class
    from std_msgs.msg import Bool, Float32
except ImportError as error:  # pragma: no cover - environment dependent
    raise unittest.SkipTest(f"bridge mechanism test dependencies unavailable: {error}") from None

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt"


def _core() -> BridgeCore:
    resolved = resolve_scenario(SCENARIO)
    bridge = copy.deepcopy(resolved.bridge)
    assert bridge is not None
    bridge["streams"] = [s for s in bridge["streams"] if "image" not in s
                         and not s["native"].startswith("sensor:ffc")
                         and not s["native"].startswith("sensor:dfc")]
    resolved = dataclasses.replace(resolved, bridge=bridge)
    sensors = [s["id"] for s in resolved.robot["sensors"] if s["type"] != "stereo_camera"]
    return BridgeCore(resolved, create_runtime(resolved, sensor_ids=sensors), epoch_ns=10**18)


def _call(core: BridgeCore, service: str, **fields: Any) -> Any:
    kind = {"arm": "std_srvs/srv/SetBool", "claw": "std_srvs/srv/SetBool"}.get(
        service, "std_srvs/srv/Trigger")
    request = service_class(kind).Request()
    for key, value in fields.items():
        setattr(request, key, value)
    return core.call(service, request)


def _run(core: BridgeCore, ticks: int) -> list[Any]:
    publications = []
    for _ in range(ticks):
        publications += core.step()[1]
    return publications


class MechanismBridgeTest(unittest.TestCase):
    def test_arming_follows_kill_and_status_is_published_at_50_hz(self) -> None:
        core = _core()
        self.assertTrue(core.killed)
        self.assertFalse(_call(core, "arm", data=True).success)  # rejected while killed
        core.set_killed(False)
        self.assertTrue(_call(core, "arm", data=True).success)
        status = [p.message for p in _run(core, 50) if p.stream == "actuator_status"]
        self.assertEqual(len(status), 5)
        last = status[-1]
        self.assertIsInstance(last, ActuatorStatus)
        self.assertTrue(last.actuators_armed)
        self.assertEqual((last.torpedo_state, last.torpedo_available_count), (3, 2))
        self.assertEqual((last.dropper_state, last.dropper_available_count), (2, 2))
        self.assertEqual(last.claw_state, 4)  # closed
        core.set_killed(True)  # kill disarms (robot safety.kill_disarms_mechanisms)
        status = [p.message for p in _run(core, 10) if p.stream == "actuator_status"]
        self.assertFalse(status[-1].actuators_armed)

    def test_fire_shares_cooldown_and_reload_disarms(self) -> None:
        core = _core()
        core.set_killed(False)
        _call(core, "arm", data=True)
        _run(core, 1)
        self.assertTrue(_call(core, "fire_torpedo").success)
        self.assertFalse(_call(core, "fire_dropper").success)  # shared release group busy
        busy = [p.message.data for p in _run(core, 10) if p.stream == "actuator_busy"]
        self.assertEqual(busy, [True])
        _run(core, 250)
        self.assertTrue(_call(core, "fire_dropper").success)
        self.assertEqual(len(core.session.payloads), 2)
        reply = _call(core, "reload")
        self.assertTrue(reply.success)
        state = core.session.mechanism_state()
        assert state is not None
        self.assertFalse(state.armed)
        self.assertEqual(state.releases["torpedo_launcher"].available, 2)
        self.assertEqual(len(core.session.payloads), 2)  # released bodies are not recalled

    def test_claw_topic_command_replies_and_moves_jaws(self) -> None:
        core = _core()
        replies = core.receive("claw_timed_move", Float32(data=1.0))
        self.assertEqual([(p.stream, p.message.data) for p in replies],
                         [("actuator_cmd_status", False)])  # disarmed and killed
        core.set_killed(False)
        _call(core, "arm", data=True)
        replies = core.receive("claw_timed_move", Float32(data=1.0))
        self.assertIsInstance(replies[0].message, Bool)
        self.assertTrue(replies[0].message.data)
        _run(core, 600)
        state = core.session.mechanism_state()
        assert state is not None
        claw = state.claws["claw"]
        # 1 s at 0.033 m/s per jaw, then the timer stops it: gap = min + 2 * 0.033.
        self.assertAlmostEqual(claw.gap_m, 0.0012 + 0.066, places=3)
        self.assertTrue(_call(core, "claw", data=False).success)
        _run(core, 2000)
        state = core.session.mechanism_state()
        assert state is not None
        self.assertEqual(state.claws["claw"].state, "closed")

    def test_topic_form_commands_reply_on_cmd_status(self) -> None:
        from std_msgs.msg import Empty

        core = _core()
        core.set_killed(False)
        replies = core.receive("arm_topic", Bool(data=True))
        self.assertEqual([(p.stream, p.message.data) for p in replies],
                         [("actuator_cmd_status", True)])
        _run(core, 1)
        self.assertTrue(core.receive("torpedo_topic", Empty())[0].message.data)
        self.assertFalse(core.receive("dropper_topic", Empty())[0].message.data)  # cooldown
        self.assertTrue(core.receive("claw_topic", Bool(data=True))[0].message.data)
        self.assertTrue(core.receive("reload_topic", Empty())[0].message.data)
        state = core.session.mechanism_state()
        assert state is not None
        self.assertFalse(state.armed)
        self.assertEqual(len(core.session.payloads), 1)

    def test_task_reset_clears_payloads_reloads_and_disarms(self) -> None:
        core = _core()
        core.set_killed(False)
        _call(core, "arm", data=True)
        _run(core, 1)
        _call(core, "fire_torpedo")
        _run(core, 10)
        self.assertTrue(_call(core, "reset_tasks").success)
        self.assertEqual(core.session.payloads, {})
        state = core.session.mechanism_state()
        assert state is not None
        self.assertFalse(state.armed)
        self.assertGreater(core.session.time_ns, 0)  # physics time is untouched

    def test_full_reset_restores_start_without_rewinding_ros_time(self) -> None:
        core = _core()
        core.set_killed(False)
        core.receive("thruster_forces", _forces(core, 20.0))
        _call(core, "arm", data=True)
        _run(core, 1)
        _call(core, "fire_torpedo")
        _run(core, 500)
        moved = np.asarray(core.runtime.observe().body.position)
        before_ns = core.clock_ns()
        reply = _call(core, "reset_scenario")
        self.assertTrue(reply.success, reply.message)
        snapshot = core.runtime.observe()
        self.assertEqual(snapshot.elapsed_ns, 0)
        self.assertFalse(np.allclose(moved, snapshot.body.position))
        np.testing.assert_allclose(snapshot.body.position, core.pack.initial.position)
        self.assertTrue(core.killed)  # robot safety initially_killed
        self.assertEqual(core.session.payloads, {})
        self.assertEqual(core.pending_alignment(), None)  # no estimate yet; request is queued
        self.assertEqual(core.alignment_pending, "full_reset")
        clocks = core.step()[0]
        self.assertGreater(clocks[0], before_ns)

    def test_full_reset_replays_identical_sensor_noise(self) -> None:
        def record(core: BridgeCore) -> list[float]:
            return [p.message.linear_acceleration.x for p in _run(core, 200)
                    if p.stream == "imu"]

        core = _core()
        first = record(core)
        _call(core, "reset_scenario")
        self.assertEqual(record(core), first)


def _forces(core: BridgeCore, value: float) -> Any:
    from std_msgs.msg import Float32MultiArray

    return Float32MultiArray(data=[value] * len(core.config["thrusters"]["order"]))


if __name__ == "__main__":
    unittest.main()
