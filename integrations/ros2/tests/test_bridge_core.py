"""BridgeCore semantics on a generic (non-Talos) layout with a fake, recording runtime.

No rclpy context, node or middleware is created: only message classes are instantiated.
"""

from __future__ import annotations

import copy
import unittest
from types import SimpleNamespace
from typing import Any

import numpy as np

try:
    import rosidl_runtime_py  # noqa: F401
    from robot_localization.srv import SetPose
    from robotics_platform import _native as native
    from robotics_platform_ros.core import BridgeCore, BridgeError, Publication
    from robotics_platform_ros.mapping import message_class, service_class
    from std_msgs.msg import Bool, Float64MultiArray
    from std_srvs.srv import Trigger
except ImportError as error:  # pragma: no cover - environment dependent
    raise unittest.SkipTest(f"bridge core test dependencies unavailable: {error}") from None

EPOCH_NS = 1_700_000_000_123_456_789
STEP_NS = 2_000_000
OFFSET = np.array([0.1, 0.0, -0.05])
WORLD = "odom_like_world"
YAW_90 = np.array([np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)])
QOS = {"reliability": "reliable", "durability": "volatile", "history": "keep_last", "depth": 10}

ROBOT: dict[str, Any] = {
    "reference_frame": "base",
    "frames": {"root": "com", "transforms": [
        {"parent": "com", "child": "base", "position_m": [0.1, 0.0, -0.05],
         "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]}]},
    "thrusters": [{"id": "a"}, {"id": "b"}, {"id": "c"}],
    "safety": {"initially_killed": True, "kill_stops_thrusters": True,
               "commands_while_killed": "zero_force", "kill_disarms_mechanisms": False,
               "arming": {"required": False}},
    "sensors": [{"id": "alt", "type": "reference_altitude", "frame": "world", "mount_frame": "base",
                 "period_ns": 20_000_000, "parameters": {"noise": {"white_stddev": 0.0}}}],
}


def _publish(name: str, message_type: str, native_id: str, fields: dict[str, Any],
             rate_hz: float, frame_id: str = "") -> dict[str, Any]:
    return {"id": name, "direction": "publish", "topic": f"/demo/{name}",
            "message_type": message_type, "native": native_id, "fields": fields,
            "rate_hz": rate_hz, "frame_id": frame_id, "qos": QOS}


def _subscribe(name: str, message_type: str, native_id: str, fields: dict[str, Any],
               **extra: Any) -> dict[str, Any]:
    return {"id": name, "direction": "subscribe", "topic": f"/demo/{name}",
            "message_type": message_type, "native": native_id, "fields": fields,
            "rate_hz": 0, "frame_id": "", "qos": QOS, **extra}


BRIDGE: dict[str, Any] = {
    "namespace": "/demo",
    "clock": {"topic": "/clock", "epoch": "system_time_at_start", "rate_hz": 500,
              "reset_policy": "preserve_ros_epoch_and_time", "publish_before_data": True,
              "real_time_factor": 1.0, "qos": QOS},
    "thrusters": {"order": ["c", "a", "b"], "input_unit": "N", "input_scales": [2.0, 1.0, -1.0],
                  "reject": ["wrong_length", "nonfinite"]},
    "frame_names": {"world": WORLD},
    "streams": [
        _publish("altitude", "std_msgs/msg/Float64", "sensor:alt",
                 {"data": {"from": "reading.target_world_z"}}, 50),
        _publish("altitude_stamped", "geometry_msgs/msg/PointStamped", "sensor:alt",
                 {"header.stamp": {"from": "sample.time"},
                  "point.z": {"from": "reading.target_world_z"}}, 50, WORLD),
        _publish("ticker", "std_msgs/msg/UInt8", "timer", {"data": {"constant": 7}}, 10),
        _publish("pose", "geometry_msgs/msg/PoseStamped", "state:robot",
                 {"header.stamp": {"from": "sim.time"},
                  "pose.position": {"from": "reference_pose.position"},
                  "pose.orientation.w": {"from": "reference_pose.orientation_wxyz[0]"},
                  "pose.orientation.x": {"from": "reference_pose.orientation_wxyz[1]"},
                  "pose.orientation.y": {"from": "reference_pose.orientation_wxyz[2]"},
                  "pose.orientation.z": {"from": "reference_pose.orientation_wxyz[3]"}},
                 100, WORLD),
        _publish("kill_event", "std_msgs/msg/Bool", "event:robot.kill_changed",
                 {"data": {"from": "killed"}}, 0),
        _subscribe("thruster_cmd", "std_msgs/msg/Float64MultiArray", "command:thrusters.set_forces",
                   {"forces_n": {"from": "data"}}),
        _subscribe("kill_cmd", "std_msgs/msg/Bool", "command:robot.set_killed",
                   {"killed": {"from": "data"}}, accept_if=[{"field": "data", "equals": True}]),
        _subscribe("unkill_cmd", "std_msgs/msg/Bool", "command:robot.set_killed",
                   {"killed": {"from": "data"}}, accept_if=[{"field": "data", "equals": False}]),
    ],
}

SET_POSE = {
    "id": "set_pose", "service_type": "robot_localization/srv/SetPose",
    "action": "command:robot.place",
    "request": {"frame": {"from": "pose.header.frame_id"},
                "position_m": {"from": "pose.pose.pose.position"},
                "orientation_w": {"from": "pose.pose.pose.orientation.w"},
                "orientation_x": {"from": "pose.pose.pose.orientation.x"},
                "orientation_y": {"from": "pose.pose.pose.orientation.y"},
                "orientation_z": {"from": "pose.pose.pose.orientation.z"}},
    "response": {},
    "placement": {"pose_source": "request", "keep_velocity": False,
                  "becomes_start_pose": True, "align_estimator": False},
}
RESET = {
    "id": "reset", "service_type": "std_srvs/srv/Trigger",
    "action": "command:robot.reset_to_start", "request": {},
    "response": {"success": {"from": "accepted"}, "message": {"from": "message"}},
}


def _body(position: Any = (1.0, 2.0, 3.0), wxyz: Any = (1.0, 0.0, 0.0, 0.0)) -> SimpleNamespace:
    return SimpleNamespace(position=np.array(position, float),
                           orientation_wxyz=np.array(wxyz, float),
                           linear_velocity=np.zeros(3), angular_velocity=np.zeros(3))


class FakeStream:
    """Scripted sensor stream: drain() returns and forgets the queued samples."""

    def __init__(self) -> None:
        self.queue: list[Any] = []

    def drain(self) -> list[Any]:
        samples, self.queue = self.queue, []
        return samples


def _sample(acquired_ns: int, z: float | None) -> SimpleNamespace:
    value = None if z is None else SimpleNamespace(
        target_world_z=z, mounted_world_z=z + 0.5, variance=0.01)
    return SimpleNamespace(header=SimpleNamespace(acquired_ns=acquired_ns), value=value)


class FakeRuntime:
    """Records every mutating call; time advances one fixed step per advance(1)."""

    def __init__(self, body: SimpleNamespace) -> None:
        self.tick = 0
        self.generation = 0
        self.body = body
        self.commands: list[np.ndarray] = []
        self.stops = 0

    def _snapshot(self) -> SimpleNamespace:
        return SimpleNamespace(generation=self.generation, tick=self.tick,
                               elapsed_ns=self.tick * STEP_NS, body=self.body)

    def observe(self) -> SimpleNamespace:
        return self._snapshot()

    def advance(self, ticks: int) -> SimpleNamespace:
        self.tick += ticks
        return self._snapshot()

    def command(self, forces: Any) -> None:
        self.commands.append(np.array(forces, float))

    def stop_thrusters(self) -> None:
        self.stops += 1


class FakePlacingRuntime(FakeRuntime):
    def __init__(self, body: SimpleNamespace) -> None:
        super().__init__(body)
        self.placed: list[tuple[np.ndarray, np.ndarray, bool]] = []

    def place(self, state: Any, *, clear_actuators: bool) -> SimpleNamespace:
        self.placed.append((np.array(state.position, float),
                            np.array(state.orientation_wxyz, float), clear_actuators))
        self.body = _body(state.position, state.orientation_wxyz)
        return self._snapshot()


def _frames() -> Any:
    pose = native.Pose()
    pose.translation = OFFSET
    pose.orientation_wxyz = [1.0, 0.0, 0.0, 0.0]
    edge = native.FixedFrame()
    edge.parent, edge.child, edge.pose = "com", "base", pose
    return native.FixedFrames("com", [edge])


def _make(bridge: dict[str, Any] | None = None, robot: dict[str, Any] | None = None, *,
          runtime: FakeRuntime | None = None, selected: tuple[str, ...] = ("alt",),
          lookup: Any = None) -> BridgeCore:
    resolved = SimpleNamespace(bridge=copy.deepcopy(bridge or BRIDGE),
                               robot=copy.deepcopy(robot or ROBOT),
                               scenario={"world_frame": "scenario_world", "seed": 0})
    initial = native.BodyState()
    initial.position = [1.0, 2.0, 3.0]
    pack = SimpleNamespace(
        runtime=runtime or FakePlacingRuntime(_body()),
        parameters=SimpleNamespace(timestep_ns=STEP_NS), frames=_frames(), initial=initial,
        streams={name: FakeStream() for name in selected}, deferred_sensor_ids=())
    return BridgeCore(resolved, pack, epoch_ns=EPOCH_NS, lookup=lookup)


def _with_streams(**changes: Any) -> dict[str, Any]:
    """BRIDGE with per-stream-id dict updates applied; a None update removes the stream."""
    bridge = copy.deepcopy(BRIDGE)
    streams = []
    for stream in bridge["streams"]:
        update = changes.get(stream["id"], {})
        if update is not None:
            streams.append({**stream, **update})
    bridge["streams"] = streams
    return bridge


def _stamp_ns(stamp: Any) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _by_stream(publications: list[Publication], stream: str) -> list[Publication]:
    return [item for item in publications if item.stream == stream]


class ConstructionTest(unittest.TestCase):
    def test_sensor_rate_mismatch_raises(self) -> None:
        with self.assertRaisesRegex(BridgeError, "rate_hz"):
            _make(_with_streams(altitude={"rate_hz": 49}))

    def test_unknown_native_endpoint_raises(self) -> None:
        with self.assertRaisesRegex(BridgeError, "state:unknown"):
            _make(_with_streams(ticker={"native": "state:unknown"}))

    def test_unselected_sensor_raises(self) -> None:
        with self.assertRaisesRegex(BridgeError, "not selected"):
            _make(selected=())

    def test_unknown_sensor_raises(self) -> None:
        with self.assertRaisesRegex(BridgeError, "unknown robot sensor"):
            _make(_with_streams(altitude={"native": "sensor:missing"}))

    def test_image_stream_raises(self) -> None:
        image = {"image": {"encoding": "rgb8"}}
        with self.assertRaisesRegex(BridgeError, "image"):
            _make(_with_streams(ticker=image))

    def test_timer_faster_than_step_rate_raises(self) -> None:
        with self.assertRaisesRegex(BridgeError, "physics step rate"):
            _make(_with_streams(ticker={"rate_hz": 1000}))

    def test_clock_faster_than_step_rate_raises(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["clock"]["rate_hz"] = 1000
        with self.assertRaisesRegex(BridgeError, "clock"):
            _make(bridge)

    def test_thruster_order_must_be_a_permutation(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["thrusters"]["order"] = ["c", "a", "a"]
        with self.assertRaisesRegex(BridgeError, "permutation"):
            _make(bridge)

    def test_thruster_reject_must_cover_both_conditions(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["thrusters"]["reject"] = ["wrong_length"]
        with self.assertRaisesRegex(BridgeError, "reject"):
            _make(bridge)

    def test_invalid_field_map_is_reported_as_bridge_error(self) -> None:
        with self.assertRaises(BridgeError):
            _make(_with_streams(altitude={"fields": {"data": {"from": "reading.nope"}}}))

    def test_epoch_and_world_frame_come_from_configuration(self) -> None:
        core = _make()
        self.assertEqual(core.world_frame, WORLD)
        self.assertEqual(core.clock_ns(), EPOCH_NS)


class SteppingTest(unittest.TestCase):
    def test_clock_precedes_data_and_data_uses_acquisition_time(self) -> None:
        core = _make()
        core.pack.streams["alt"].queue = [
            _sample(500_000, -1.5), _sample(1_000_000, None), _sample(1_500_000, -2.5)]
        clocks, publications, transforms = core.step()
        self.assertEqual(clocks, [EPOCH_NS + STEP_NS])
        self.assertEqual(transforms, [])
        plain = _by_stream(publications, "altitude")
        self.assertEqual([item.message.data for item in plain], [-1.5, -2.5])
        stamped = _by_stream(publications, "altitude_stamped")
        self.assertEqual([_stamp_ns(item.message.header.stamp) for item in stamped],
                         [EPOCH_NS + 500_000, EPOCH_NS + 1_500_000])
        self.assertNotIn(EPOCH_NS + STEP_NS, [_stamp_ns(i.message.header.stamp) for i in stamped])
        self.assertEqual([item.message.point.z for item in stamped], [-1.5, -2.5])
        self.assertEqual({item.message.header.frame_id for item in stamped}, {WORLD})
        self.assertEqual(len(publications), 4)

    def test_unavailable_samples_are_counted_not_published(self) -> None:
        core = _make()
        core.pack.streams["alt"].queue = [_sample(500_000, None), _sample(1_000_000, -1.0)]
        _, publications, _ = core.step()
        self.assertEqual(core.counters.unavailable_samples, {"alt": 1})
        self.assertEqual(len(_by_stream(publications, "altitude")), 1)
        self.assertEqual(core.counters.published["altitude"], 1)
        self.assertEqual(core.counters.published["altitude_stamped"], 1)

    def test_clock_stamp_follows_elapsed_time_every_step(self) -> None:
        core = _make()
        stamps = [stamp for _ in range(5) for stamp in core.step()[0]]
        self.assertEqual(stamps, [EPOCH_NS + STEP_NS * n for n in range(1, 6)])
        self.assertEqual(core.clock_ns(), EPOCH_NS + 5 * STEP_NS)

    def test_slow_clock_publishes_at_its_period(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["clock"]["rate_hz"] = 100
        core = _make(bridge)
        stamps = [stamp for _ in range(20) for stamp in core.step()[0]]
        self.assertEqual(stamps, [EPOCH_NS + STEP_NS, EPOCH_NS + 10_000_000,
                                  EPOCH_NS + 20_000_000, EPOCH_NS + 30_000_000,
                                  EPOCH_NS + 40_000_000])

    def test_timer_and_state_publish_at_their_periods(self) -> None:
        core = _make()
        collected: list[Publication] = []
        for _ in range(100):  # 200 ms of simulated time
            collected += core.step()[1]
        ticker = _by_stream(collected, "ticker")
        pose = _by_stream(collected, "pose")
        self.assertEqual(len(ticker), 2)  # 10 Hz -> 100 ms, 200 ms
        self.assertEqual(len(pose), 20)  # 100 Hz -> every 10 ms
        self.assertEqual({item.message.data for item in ticker}, {7})
        self.assertEqual([_stamp_ns(item.message.header.stamp) for item in pose],
                         [EPOCH_NS + 10_000_000 * n for n in range(1, 21)])
        self.assertEqual(core.counters.published["ticker"], 2)
        self.assertEqual(core.counters.published["pose"], 20)
        self.assertNotIn("altitude", core.counters.published)  # no sensor samples were queued

    def test_pose_is_center_of_mass_pose_composed_with_the_reference_offset(self) -> None:
        core = _make()
        core.runtime.body = _body([1.0, 2.0, 3.0], YAW_90)
        published: list[Publication] = []
        for _ in range(5):
            published += _by_stream(core.step()[1], "pose")
        (item,) = published
        message = item.message
        self.assertEqual(message.header.frame_id, WORLD)
        rotated_offset = np.array([0.0, 0.1, -0.05])  # yaw 90 degrees maps x to y
        position = message.pose.position
        np.testing.assert_allclose([position.x, position.y, position.z],
                                   np.array([1.0, 2.0, 3.0]) + rotated_offset, atol=1e-12)
        orientation = message.pose.orientation
        np.testing.assert_allclose([orientation.w, orientation.x, orientation.y, orientation.z],
                                   YAW_90, atol=1e-12)

    def test_reference_pose_helper_matches_frame_tree(self) -> None:
        core = _make()
        pose = core.reference_pose(_body([0.0, 0.0, 0.0]))
        np.testing.assert_allclose(pose.translation, OFFSET, atol=1e-12)

    def test_transforms_are_produced_only_when_configured(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        bridge["tf"] = {"publish": [{"native": "state:robot.reference_pose", "parent": WORLD,
                                     "child": "base", "rate_hz": 100}]}
        core = _make(bridge)
        transforms = [t for _ in range(10) for t in core.step()[2]]
        self.assertEqual(len(transforms), 2)
        self.assertEqual((transforms[0].parent, transforms[0].child), (WORLD, "base"))
        self.assertEqual(transforms[0].stamp_ns, EPOCH_NS + 5 * STEP_NS)
        np.testing.assert_allclose(transforms[0].translation, np.array([1.0, 2.0, 3.0]) + OFFSET)


def _float_array(values: list[float]) -> Any:
    message = Float64MultiArray()
    message.data = values
    return message


def _bool(value: bool) -> Any:
    message = Bool()
    message.data = value
    return message


class CommandTest(unittest.TestCase):
    def _armed(self, robot: dict[str, Any] | None = None) -> BridgeCore:
        core = _make(robot=robot)
        core.receive("unkill_cmd", _bool(False))
        return core

    def test_thruster_order_and_scales_map_ros_data_to_native_order(self) -> None:
        core = self._armed()
        self.assertEqual(core.receive("thruster_cmd", _float_array([1.0, 2.0, 3.0])), [])
        (command,) = core.runtime.commands
        # order [c, a, b] with scales [2, 1, -1]: c=1*2, a=2*1, b=3*-1; native order [a, b, c]
        np.testing.assert_array_equal(command, [2.0, -3.0, 2.0])

    def test_wrong_length_and_nonfinite_commands_are_rejected_and_counted(self) -> None:
        core = self._armed()
        core.receive("thruster_cmd", _float_array([1.0, 2.0]))
        core.receive("thruster_cmd", _float_array([1.0, 2.0, 3.0, 4.0]))
        core.receive("thruster_cmd", _float_array([1.0, float("nan"), 3.0]))
        core.receive("thruster_cmd", _float_array([float("inf"), 0.0, 0.0]))
        self.assertEqual(core.runtime.commands, [])
        self.assertEqual(core.counters.rejected_commands,
                         {"thruster_cmd:wrong_length": 2, "thruster_cmd:nonfinite": 2})

    def test_initially_killed_zero_force_sends_zeros(self) -> None:
        core = _make()
        self.assertTrue(core.killed)
        core.receive("thruster_cmd", _float_array([1.0, 2.0, 3.0]))
        (command,) = core.runtime.commands
        np.testing.assert_array_equal(command, [0.0, 0.0, 0.0])
        self.assertEqual(core.counters.rejected_commands, {})

    def test_initially_killed_rejected_policy_sends_nothing(self) -> None:
        robot = copy.deepcopy(ROBOT)
        robot["safety"]["commands_while_killed"] = "rejected"
        core = _make(robot=robot)
        core.receive("thruster_cmd", _float_array([1.0, 2.0, 3.0]))
        self.assertEqual(core.runtime.commands, [])
        self.assertEqual(core.counters.rejected_commands, {"thruster_cmd:killed": 1})

    def test_kill_stops_thrusters_and_publishes_one_event(self) -> None:
        core = self._armed()
        self.assertFalse(core.killed)
        self.assertEqual(core.counters.published, {"kill_event": 1})  # the unkill event
        events = core.receive("kill_cmd", _bool(True))
        self.assertTrue(core.killed)
        self.assertEqual(core.runtime.stops, 1)
        self.assertEqual([(item.stream, item.message.data) for item in events],
                         [("kill_event", True)])
        self.assertEqual(core.counters.published["kill_event"], 2)
        core.receive("thruster_cmd", _float_array([1.0, 2.0, 3.0]))
        np.testing.assert_array_equal(core.runtime.commands[-1], [0.0, 0.0, 0.0])

    def test_unkill_publishes_event_without_stopping_thrusters(self) -> None:
        core = _make()
        events = core.receive("unkill_cmd", _bool(False))
        self.assertFalse(core.killed)
        self.assertEqual(core.runtime.stops, 0)
        self.assertEqual([(item.stream, item.message.data) for item in events],
                         [("kill_event", False)])

    def test_kill_without_stop_policy_leaves_thrusters_alone(self) -> None:
        robot = copy.deepcopy(ROBOT)
        robot["safety"]["kill_stops_thrusters"] = False
        core = self._armed(robot)
        core.receive("kill_cmd", _bool(True))
        self.assertTrue(core.killed)
        self.assertEqual(core.runtime.stops, 0)

    def test_messages_rejected_by_accept_if_are_counted_and_ignored(self) -> None:
        core = self._armed()
        self.assertEqual(core.receive("kill_cmd", _bool(False)), [])
        self.assertEqual(core.receive("unkill_cmd", _bool(True)), [])
        self.assertFalse(core.killed)
        self.assertEqual(core.runtime.stops, 0)
        self.assertEqual(core.counters.filtered_messages, {"kill_cmd": 1, "unkill_cmd": 1})
        self.assertEqual(core.counters.published, {"kill_event": 1})  # only the initial unkill

    def test_thruster_subscription_requires_a_thruster_block(self) -> None:
        bridge = copy.deepcopy(BRIDGE)
        del bridge["thrusters"]
        with self.assertRaisesRegex(BridgeError, "thrusters block"):
            _make(bridge)


def _pose_request(frame: str, position: list[float], wxyz: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0)
                  ) -> Any:
    request = SetPose.Request()
    request.pose.header.frame_id = frame
    target = request.pose.pose.pose
    target.position.x, target.position.y, target.position.z = position
    w, x, y, z = (float(value) for value in wxyz)
    target.orientation.w, target.orientation.x = w, x
    target.orientation.y, target.orientation.z = y, z
    return request


def _with_services(*services: dict[str, Any]) -> dict[str, Any]:
    bridge = copy.deepcopy(BRIDGE)
    bridge["services"] = [copy.deepcopy(item) for item in services]
    return bridge


class PlacementTest(unittest.TestCase):
    def test_service_types_are_available(self) -> None:
        self.assertIs(service_class("std_srvs/srv/Trigger"), Trigger)
        self.assertIs(message_class("std_msgs/msg/Bool"), Bool)

    def test_world_frame_request_places_center_of_mass_from_reference_pose(self) -> None:
        core = _make(_with_services(SET_POSE, RESET))
        response = core.call("set_pose", _pose_request("", [2.0, 3.0, -1.0]))
        self.assertIsInstance(response, SetPose.Response)
        ((position, orientation, clear),) = core.runtime.placed
        np.testing.assert_allclose(position, np.array([2.0, 3.0, -1.0]) - OFFSET, atol=1e-12)
        np.testing.assert_allclose(orientation, [1.0, 0.0, 0.0, 0.0], atol=1e-12)
        self.assertTrue(clear)
        self.assertEqual(core.counters.service_calls, {"set_pose": 1})

    def test_rotated_world_request_rotates_the_offset(self) -> None:
        core = _make(_with_services(SET_POSE))
        core.call("set_pose", _pose_request(WORLD, [2.0, 3.0, -1.0], tuple(YAW_90)))
        ((position, orientation, _),) = core.runtime.placed
        np.testing.assert_allclose(position, [2.0, 3.0 - 0.1, -1.0 + 0.05], atol=1e-12)
        np.testing.assert_allclose(orientation, YAW_90, atol=1e-12)

    def test_non_world_frame_uses_injected_lookup(self) -> None:
        calls: list[tuple[str, str]] = []

        def lookup(target: str, source: str) -> Any:
            calls.append((target, source))
            pose = native.Pose()
            pose.translation = [10.0, 0.0, 0.0]
            return pose

        core = _make(_with_services(SET_POSE), lookup=lookup)
        core.call("set_pose", _pose_request("map", [1.0, 1.0, 1.0]))
        self.assertEqual(calls, [(WORLD, "map")])
        ((position, _, _),) = core.runtime.placed
        np.testing.assert_allclose(position, np.array([11.0, 1.0, 1.0]) - OFFSET, atol=1e-12)

    def test_missing_transform_is_refused_without_placing(self) -> None:
        core = _make(_with_services(SET_POSE), lookup=lambda target, source: None)
        core.call("set_pose", _pose_request("map", [1.0, 1.0, 1.0]))
        self.assertEqual(core.runtime.placed, [])
        accepted, message = core.place_reference(
            {"frame": "map", "position_m": np.zeros(3), "orientation_w": 1.0,
             "orientation_x": 0.0, "orientation_y": 0.0, "orientation_z": 0.0},
            {"keep_velocity": False, "becomes_start_pose": True})
        self.assertFalse(accepted)
        self.assertIn("map", message)
        self.assertEqual(core.runtime.placed, [])

    def test_unknown_frame_without_any_lookup_is_refused(self) -> None:
        core = _make(_with_services(SET_POSE))
        core.call("set_pose", _pose_request("map", [0.0, 0.0, 0.0]))
        self.assertEqual(core.runtime.placed, [])

    def test_zero_quaternion_and_nonfinite_position_are_refused(self) -> None:
        core = _make(_with_services(SET_POSE))
        core.call("set_pose", _pose_request("", [0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 0.0)))
        core.call("set_pose", _pose_request("", [float("nan"), 0.0, 0.0]))
        self.assertEqual(core.runtime.placed, [])

    def test_reset_returns_to_the_placed_start_pose(self) -> None:
        core = _make(_with_services(SET_POSE, RESET))
        core.call("set_pose", _pose_request("", [2.0, 3.0, -1.0]))
        core.runtime.body = _body([9.0, 9.0, 9.0], YAW_90)  # the vehicle drifts away
        response = core.call("reset", Trigger.Request())
        self.assertTrue(response.success)
        self.assertIn("(2.000, 3.000, -1.000)", response.message)
        self.assertEqual(len(core.runtime.placed), 2)
        expected = np.array([2.0, 3.0, -1.0]) - OFFSET
        np.testing.assert_allclose(core.runtime.placed[1][0], expected, atol=1e-12)
        np.testing.assert_allclose(core.runtime.placed[1][1], [1.0, 0.0, 0.0, 0.0], atol=1e-12)
        self.assertTrue(core.runtime.placed[1][2])

    def test_reset_without_placement_returns_to_the_initial_state(self) -> None:
        core = _make(_with_services(RESET))
        response = core.call("reset", Trigger.Request())
        self.assertTrue(response.success)
        ((position, _, clear),) = core.runtime.placed
        np.testing.assert_allclose(position, [1.0, 2.0, 3.0], atol=1e-12)
        self.assertTrue(clear)

    def test_placement_that_is_not_a_new_start_pose_keeps_the_previous_one(self) -> None:
        service = copy.deepcopy(SET_POSE)
        service["placement"]["becomes_start_pose"] = False
        core = _make(_with_services(service, RESET))
        core.call("set_pose", _pose_request("", [2.0, 3.0, -1.0]))
        core.call("reset", Trigger.Request())
        np.testing.assert_allclose(core.runtime.placed[1][0], [1.0, 2.0, 3.0], atol=1e-12)

    def test_unsupported_action_raises_at_construction(self) -> None:
        service = {**RESET, "id": "teleport", "action": "command:robot.teleport"}
        with self.assertRaisesRegex(BridgeError, "command:robot.teleport"):
            _make(_with_services(service))

    def test_runtime_without_place_raises_at_construction(self) -> None:
        runtime = FakeRuntime(_body())
        self.assertFalse(hasattr(runtime, "place"))
        with self.assertRaisesRegex(BridgeError, "place"):
            _make(_with_services(SET_POSE), runtime=runtime)

    def test_runtime_without_place_is_fine_when_no_services_are_declared(self) -> None:
        core = _make(runtime=FakeRuntime(_body()))
        self.assertEqual(core.services, {})


if __name__ == "__main__":
    unittest.main()
