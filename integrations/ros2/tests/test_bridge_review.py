"""Regressions for review findings: sequence indexes, malformed inbound data, scaled overflow,
and declared-but-unexecuted configuration."""

from __future__ import annotations

import copy
import unittest
from typing import Any

try:
    from robotics_platform_ros.core import BridgeError
    from robotics_platform_ros.mapping import (
        SCALAR,
        MappingError,
        compile_reader,
        compile_writer,
        message_class,
    )
    from test_bridge_core import BRIDGE, SET_POSE, _bool, _float_array, _make, _with_services
except ImportError as error:  # also covers test_bridge_core's own skip when ROS is absent
    raise unittest.SkipTest(f"bridge test dependencies unavailable: {error}") from None
except unittest.SkipTest:
    raise


class SequenceIndexTest(unittest.TestCase):
    def test_writer_rejects_element_of_unbounded_output_sequence(self) -> None:
        cls = message_class("std_msgs/msg/Float32MultiArray")
        with self.assertRaisesRegex(MappingError, "sequence 'data'"):
            compile_writer(cls, {"data[0]": {"constant": 1.0}}, {})

    def test_short_inbound_sequence_is_a_mapping_error_not_a_crash(self) -> None:
        cls = message_class("std_msgs/msg/Float32MultiArray")
        reader = compile_reader(cls, {"value": {"from": "data[2]"}}, {"value": SCALAR})
        message = cls()
        message.data = [1.0]
        with self.assertRaisesRegex(MappingError, "shorter than the mapped index"):
            reader(message)
        message.data = [1.0, 2.0, 3.0]
        self.assertEqual(reader(message), {"value": 3.0})

    def test_core_counts_malformed_inbound_messages_without_crashing(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["streams"].append({
            "id": "estimate", "direction": "subscribe", "topic": "/demo/estimate",
            "message_type": "sensor_msgs/msg/JointState", "native": "estimate:latest",
            "rate_hz": 0, "frame_id": "", "qos": bridge["streams"][0]["qos"],
            "fields": {"frame": {"from": "header.frame_id"}, "position_m": {"from": "velocity"},
                       "orientation_w": {"from": "position[0]"},
                       "orientation_x": {"from": "position[1]"},
                       "orientation_y": {"from": "position[2]"},
                       "orientation_z": {"from": "position[3]"}}})
        core = _make(bridge)
        cls = message_class("sensor_msgs/msg/JointState")
        short_index, short_vector, valid = cls(), cls(), cls()
        short_index.position, short_index.velocity = [1.0], [0.0, 0.0, 0.0]
        short_vector.position, short_vector.velocity = [1.0, 0.0, 0.0, 0.0], [0.0, 0.0]
        valid.position, valid.velocity = [1.0, 0.0, 0.0, 0.0], [4.0, 5.0, 6.0]
        for message in (short_index, short_vector):
            self.assertEqual(core.receive("estimate", message), [])
        self.assertEqual(core.counters.rejected_commands, {"estimate:malformed": 2})
        self.assertIsNone(core.latest_estimate)
        core.receive("estimate", valid)
        assert core.latest_estimate is not None
        self.assertEqual(list(core.latest_estimate["position_m"]), [4.0, 5.0, 6.0])

    def test_incompatible_inbound_type_fails_at_preflight(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        for stream in bridge["streams"]:
            if stream["id"] == "unkill_cmd":
                stream["message_type"] = "std_msgs/msg/Float32MultiArray"
                stream["fields"] = {"killed": {"from": "data[0]"}}
                stream.pop("accept_if", None)
        with self.assertRaisesRegex(BridgeError, "does not provide native bool"):
            _make(bridge)


class ScaledOverflowTest(unittest.TestCase):
    def test_finite_input_that_overflows_after_scaling_is_rejected(self) -> None:
        core = _make()
        core.receive("unkill_cmd", _bool(False))
        core.receive("thruster_cmd", _float_array([1e308, 0.0, 0.0]))  # scale 2.0 -> inf
        self.assertEqual(core.runtime.commands, [])
        self.assertEqual(core.counters.rejected_commands, {"thruster_cmd:nonfinite_scaled": 1})


class DeclaredConfigTest(unittest.TestCase):
    def _rejects(self, bridge: dict[str, Any], fragment: str) -> None:
        with self.assertRaisesRegex(BridgeError, fragment):
            _make(bridge)

    def test_declared_converter_is_rejected(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["streams"][0]["converter"] = "custom"
        self._rejects(bridge, "converters are declared")

    def test_kill_block_must_match_executed_streams(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["kill"] = {"command_stream": "thruster_cmd", "state_stream": "kill_event"}
        self._rejects(bridge, "command:robot.set_killed")

    def test_task_reset_binding_is_rejected(self) -> None:
        bridge = _with_services(SET_POSE)
        bridge["reset"] = {"tasks_service": "set_pose"}
        self._rejects(bridge, "task and full resets are not executed")

    def test_frame_names_must_agree_with_stamped_sensor_frame_id(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        for stream in bridge["streams"]:
            if stream["id"] == "altitude_stamped":
                stream["frame_id"] = "elsewhere"
        self._rejects(bridge, "frame_names of sensor frame 'world'")

    def test_never_published_frames_are_enforced(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["tf"] = {"publish": [{"parent": "odom_like_world", "child": "robot/base",
                                     "native": "state:robot.reference_pose", "rate_hz": 50}],
                        "lookup": [], "never_publish": ["robot/*"]}
        self._rejects(bridge, "never_publish")


if __name__ == "__main__":
    unittest.main()
