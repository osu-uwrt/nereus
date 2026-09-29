"""Viewer/operator interfaces of the original simulator on the real Talos bridge pack.

Run control, scorecard, event feed, visual state streams (thruster forces, claw jaws, magnet
lights, task props, projectiles), the latched scenario description and the real_time_factor
hook. Real packs and ROS message classes; no rclpy context or middleware.
"""

from __future__ import annotations

import copy
import dataclasses
import json
import math
import unittest
from pathlib import Path
from typing import Any

import numpy as np

try:
    from robotics_platform.pack_runtime import create_runtime
    from robotics_platform.packs import resolve_scenario
    from robotics_platform_ros.core import BridgeCore, BridgeError
    from robotics_platform_ros.mapping import service_class
    from std_msgs.msg import Empty, Float32MultiArray, String
    from visualization_msgs.msg import Marker
except ImportError as error:  # pragma: no cover - environment dependent
    raise unittest.SkipTest(f"bridge viewer test dependencies unavailable: {error}") from None

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "content/packs/scenarios/talos_uwrt"
ROWS = ["gate", "slalom_front", "slalom_middle", "slalom_back", "bins", "lights", "torpedoes",
        "sequence", "distance", "surface", "facing", "objects_surface", "objects_drop", "baskets",
        "basket_count", "home", "pinger_first", "pinger_second"]


def _core(edit: Any = None) -> BridgeCore:
    resolved = resolve_scenario(SCENARIO)
    bridge = copy.deepcopy(resolved.bridge)
    assert bridge is not None
    bridge["streams"] = [s for s in bridge["streams"] if "image" not in s
                         and not s["native"].startswith("sensor:ffc")
                         and not s["native"].startswith("sensor:dfc")]
    if edit is not None:
        edit(bridge)
    resolved = dataclasses.replace(resolved, bridge=bridge)
    sensors = [s["id"] for s in resolved.robot["sensors"] if s["type"] != "stereo_camera"]
    return BridgeCore(resolved, create_runtime(resolved, sensor_ids=sensors), epoch_ns=10**18)


def _run(core: BridgeCore, ticks: int) -> list[Any]:
    publications = []
    for _ in range(ticks):
        publications += core.step()[1]
    return publications


def _messages(publications: list[Any], stream: str) -> list[Any]:
    return [p.message for p in publications if p.stream == stream]


def _command(core: BridgeCore, **fields: Any) -> None:
    core.receive("run_command", String(data=json.dumps(fields)))


def _score(core: BridgeCore) -> dict[str, Any]:
    return json.loads(_messages(_run(core, 10), "run_score")[-1].data)  # 50 Hz: within 10 ticks


def _fire(core: BridgeCore, service: str) -> None:
    core.call(service, service_class("std_srvs/srv/Trigger").Request())


class ViewerStreamsTest(unittest.TestCase):
    def test_declared_topics_types_and_rates(self) -> None:
        core = _core()
        expected = {
            "run_score": ("simulator/run_score", "std_msgs/msg/String", 50),
            "task_score": ("simulator/task_score", "std_msgs/msg/String", 50),
            "task_events": ("simulator/task_events", "std_msgs/msg/String", 0),
            "actual_thruster_forces": ("simulator/actual_thruster_forces",
                                       "std_msgs/msg/Float32MultiArray", 100),
            "claw_joints": ("simulator/claw_joints", "std_msgs/msg/Float64MultiArray", 50),
            "magnet_lights": ("simulator/magnet_lights",
                              "visualization_msgs/msg/MarkerArray", 50),
            "task_objects": ("simulator/task_objects",
                             "visualization_msgs/msg/MarkerArray", 50),
            "projectiles": ("simulator/projectiles", "visualization_msgs/msg/MarkerArray", 50),
            "scenario": ("simulator/scenario", "std_msgs/msg/String", 0),
            "run_command": ("simulator/run_command", "std_msgs/msg/String", 0),
            "reset_tasks_topic": ("simulator/reset_tasks", "std_msgs/msg/Empty", 0),
        }
        for stream, (topic, kind, rate) in expected.items():
            with self.subTest(stream=stream):
                self.assertEqual((core.stream_config[stream]["topic"],
                                  core.stream_config[stream]["message_type"],
                                  core.stream_config[stream]["rate_hz"]), (topic, kind, rate))
        scenario = core.stream_config["scenario"]
        self.assertEqual((scenario["qos"]["durability"], scenario["qos"]["depth"]),
                         ("transient_local", 1))
        self.assertEqual(core.config["node_name"], "physics_simulator")
        # The Empty topic and the Trigger service of the same name are separate interfaces.
        self.assertIn("reset_tasks", core.services)
        self.assertEqual(core.services["reset_tasks"][2]["action"], "command:tasks.reset")

    def test_streams_publish_at_their_rates(self) -> None:
        core = _core()
        publications = _run(core, 500)  # 1 s
        for stream, count in (("run_score", 50), ("task_score", 50), ("claw_joints", 50),
                              ("magnet_lights", 50), ("task_objects", 50), ("projectiles", 50),
                              ("actual_thruster_forces", 100)):
            with self.subTest(stream=stream):
                self.assertEqual(len(_messages(publications, stream)), count)

    def test_actual_thruster_forces_are_realized_and_in_bridge_order(self) -> None:
        core = _core()
        core.set_killed(False)
        order = core.config["thrusters"]["order"]
        command = Float32MultiArray(data=[0.0] * len(order))
        command.data[order.index("HLP")] = 15.0
        core.receive("thruster_forces", command)
        early = _messages(_run(core, 5), "actual_thruster_forces")  # first 100 Hz sample
        self.assertEqual(list(early[0].data), [0.0] * 8)  # lag: nothing realized yet
        published = _messages(_run(core, 200), "actual_thruster_forces")
        last = published[-1]
        self.assertIsInstance(last, Float32MultiArray)
        self.assertEqual(len(last.data), 8)
        self.assertGreater(last.data[order.index("HLP")], 5.0)
        self.assertLess(last.data[order.index("HLP")], 15.0)
        others = [value for index, value in enumerate(last.data) if index != order.index("HLP")]
        self.assertLess(max(abs(value) for value in others), 1e-6)
        native = core.session.thruster_forces()
        native_ids = [item["id"] for item in core.robot["thrusters"]]
        self.assertAlmostEqual(float(last.data[order.index("HLP")]),
                               float(native[native_ids.index("HLP")]), places=4)

    def test_claw_joints_are_left_and_right_jaw_travel(self) -> None:
        core = _core()
        joints = _messages(_run(core, 10), "claw_joints")[-1]
        self.assertEqual(list(joints.data), [0.0, 0.0])
        core.set_killed(False)
        core.session.set_armed(True)
        core.session.command_claw("claw", True)
        _run(core, 500)
        joints = _messages(_run(core, 10), "claw_joints")[-1]
        self.assertEqual(len(joints.data), 2)
        self.assertGreater(joints.data[0], 0.0)
        self.assertEqual(joints.data[0], joints.data[1])
        self.assertLessEqual(joints.data[0], 0.0694)  # (max_gap - min_gap) / 2 travel

    def test_magnet_lights_are_red_then_green_at_the_face_poses(self) -> None:
        core = _core()
        lights = _messages(_run(core, 10), "magnet_lights")[-1]
        self.assertEqual([m.ns for m in lights.markers], ["magnet_target1", "magnet_target2"])
        first = lights.markers[0]
        self.assertEqual((first.header.frame_id, first.type, first.action, first.id),
                         ("map", Marker.SPHERE, Marker.ADD, 0))
        self.assertEqual((first.color.r, first.color.g, first.color.a), (1.0, 0.0, 1.0))
        self.assertEqual((first.scale.x, first.scale.y, first.scale.z), (0.002, 0.044, 0.044))
        face = core.session.indicators()[0]
        self.assertEqual((first.pose.position.x, first.pose.position.z),
                         (face["position_m"][0], face["position_m"][2]))
        self.assertAlmostEqual(math.sqrt(first.pose.orientation.w ** 2 + first.pose.orientation.x ** 2
                                         + first.pose.orientation.y ** 2
                                         + first.pose.orientation.z ** 2), 1.0)
        core.session.tasks._targets[("bins", "magnet_target1")].latched = True
        lights = _messages(_run(core, 10), "magnet_lights")[-1]
        self.assertEqual([(m.color.r, m.color.g) for m in lights.markers],
                         [(0.0, 1.0), (1.0, 0.0)])

    def test_task_objects_are_pack_meshes_in_world_or_robot_frame_when_held(self) -> None:
        core = _core()
        objects = _messages(_run(core, 10), "task_objects")[-1]
        self.assertEqual([m.ns for m in objects.markers],
                         ["pill", "bandage", "nut_and_bolt", "plug"])
        pill = objects.markers[0]
        self.assertEqual((pill.header.frame_id, pill.type, pill.action, pill.color.a),
                         ("map", Marker.MESH_RESOURCE, Marker.ADD, 1.0))
        self.assertTrue(pill.mesh_use_embedded_materials)
        self.assertEqual((pill.scale.x, pill.scale.y, pill.scale.z), (1.0, 1.0, 1.0))
        self.assertTrue(pill.mesh_resource.startswith("file:///"))
        self.assertTrue(pill.mesh_resource.endswith("assets/visual/table_pill/model.dae"))
        self.assertTrue(Path(pill.mesh_resource.removeprefix("file://")).is_file())
        world = core.session.prop_visuals()[0]["position_m"]
        self.assertAlmostEqual(pill.pose.position.x, world[0])
        core.session.prop_worlds[0].held = "pill"  # attached to the claw: robot-relative pose
        objects = core.publishers["task_objects"](core._state_values["task_objects"]())
        held, other = objects.markers[0], objects.markers[1]
        self.assertEqual((held.header.frame_id, other.header.frame_id),
                         ("talos/base_link", "map"))
        body = core.reference_pose(core.session.last_step.snapshot.body)
        rebuilt = body.apply(np.array([held.pose.position.x, held.pose.position.y,
                                       held.pose.position.z]))
        np.testing.assert_allclose(rebuilt, core.session.prop_visuals()[0]["position_m"],
                                   atol=1e-6)

    def test_projectiles_clear_then_show_flight_and_loaded_rounds(self) -> None:
        core = _core()
        array = _messages(_run(core, 10), "projectiles")[-1]
        self.assertEqual(array.markers[0].action, Marker.DELETEALL)
        loaded = array.markers[1:]
        self.assertEqual([(m.ns, m.id) for m in loaded],
                         [("torpedo_loaded", 0), ("torpedo_loaded", 1),
                          ("dropper_loaded", 0), ("dropper_loaded", 1)])
        first = loaded[0]
        self.assertEqual((first.header.frame_id, first.type, first.mesh_use_embedded_materials),
                         ("map", Marker.MESH_RESOURCE, False))
        self.assertTrue(first.mesh_resource.startswith("file:///"))
        self.assertTrue(first.mesh_resource.endswith("assets/visual/payloads/projectile.glb"))
        self.assertAlmostEqual(first.scale.x, 0.08299993)
        self.assertAlmostEqual(first.scale.y, 0.026)
        self.assertEqual((first.color.r, first.color.g, first.color.b), (0.65, 0.025, 0.035))
        core.set_killed(False)
        core.session.set_armed(True)
        _run(core, 1)
        self.assertTrue(core.session.fire("torpedo_launcher").accepted)
        array = _messages(_run(core, 10), "projectiles")[-1]
        self.assertEqual(array.markers[0].action, Marker.DELETEALL)
        self.assertEqual([(m.ns, m.id) for m in array.markers[1:]],
                         [("torpedo", 0), ("torpedo_loaded", 1),
                          ("dropper_loaded", 0), ("dropper_loaded", 1)])
        self.assertLess(array.markers[1].pose.position.z, 0.0)


class RunControlBridgeTest(unittest.TestCase):
    def test_run_score_format_and_start_stop_adjustment_commands(self) -> None:
        core = _core()
        score = _score(core)
        self.assertEqual([row["key"] for row in score["rows"]], ROWS)
        self.assertTrue(score["running"])  # scenario auto_start
        _command(core, action="start", role="rescue", heading_coin=False, role_coin=True)
        rejected = _score(core)
        self.assertEqual(rejected["message"], "Command rejected: Stop the current run first")
        self.assertEqual(rejected["intended_role"], "repair")  # nothing changed
        _command(core, action="stop")
        stopped = _score(core)
        self.assertEqual((stopped["running"], stopped["message"]), (False, "Run stopped"))
        _command(core, action="start", role="rescue", heading_coin=False, role_coin=True)
        started = _score(core)
        self.assertEqual((started["running"], started["intended_role"], started["target_class"],
                          started["message"]),
                         (True, "rescue", "blood", "Run started; pass the gate first"))
        _run(core, 250)
        self.assertAlmostEqual(_score(core)["elapsed"], 0.5, delta=0.06)
        _command(core, action="adjustment", points=125.5)
        adjusted = _score(core)
        self.assertEqual((adjusted["adjustment"], adjusted["total"]), (125.5, 125.5))

    def test_bad_commands_are_reported_in_the_run_message(self) -> None:
        core = _core()
        cases = [("not json", "Command rejected: "), (json.dumps({"x": 1}), "Command rejected: "),
                 (json.dumps({"action": "dance"}), "Command rejected: Unknown run command"),
                 (json.dumps({"action": "adjustment", "points": "lots"}),
                  "Command rejected: Adjustment must be a number"),
                 (json.dumps({"action": "adjustment"}), "Command rejected: "),
                 (json.dumps({"action": "pinger_switch"}), "Command rejected: Unknown run command")]
        for text, prefix in cases:
            with self.subTest(text=text):
                core.receive("run_command", String(data=text))
                self.assertTrue(_score(core)["message"].startswith(prefix))
        self.assertEqual(core.counters.rejected_commands["run_command"], len(cases))

    def test_reset_topic_and_service_publish_the_reset_event_and_counters(self) -> None:
        core = _core()
        _command(core, action="adjustment", points=50)
        core.receive("reset_tasks_topic", Empty())
        events = [json.loads(m.data) for m in _messages(_run(core, 1), "task_events")]
        self.assertEqual([(e["kind"], e["result"], e["target"]) for e in events],
                         [("tasks", "reset", "")])
        self.assertEqual(_score(core)["adjustment"], 0.0)
        _fire(core, "reset_tasks")
        events = [json.loads(m.data) for m in _messages(_run(core, 1), "task_events")]
        self.assertEqual(len(events), 1)
        self.assertEqual(json.loads(_messages(_run(core, 10), "task_score")[-1].data),
                         {"success": 0, "wrong_target": 0, "blocked": 0, "miss": 0})

    def test_payload_events_use_the_original_kinds_and_results(self) -> None:
        core = _core()
        core.set_killed(False)
        core.session.set_armed(True)
        _run(core, 1)
        self.assertTrue(core.session.fire("dropper").accepted)
        events = [json.loads(m.data) for m in _messages(_run(core, 1200), "task_events")]
        self.assertEqual([(e["kind"], e["result"]) for e in events],
                         [("dropper", "released"), ("dropper", "miss")])
        self.assertEqual((events[0]["id"], events[0]["slot"], events[1]["target"]),
                         (0, 0, "pool_floor"))
        self.assertGreater(events[1]["time"], events[0]["time"])
        counts = json.loads(_messages(_run(core, 10), "task_score")[-1].data)
        self.assertEqual(counts, {"success": 0, "wrong_target": 0, "blocked": 0, "miss": 1})

    def test_flush_delivers_operator_events_without_stepping(self) -> None:
        core = _core()
        core.receive("reset_tasks_topic", Empty())
        publications = core.flush()  # paused: no tick needed
        self.assertEqual([p.stream for p in publications], ["task_events"])
        self.assertEqual(core.flush(), [])
        refreshed = {p.stream for p in core.refresh()}
        self.assertEqual({"run_score", "task_score", "claw_joints", "magnet_lights",
                          "task_objects", "projectiles", "actual_thruster_forces",
                          "actuator_status", "actuator_busy"}, refreshed)


class ScenarioAndRateTest(unittest.TestCase):
    def test_scenario_is_the_resolved_document_with_absolute_asset_paths(self) -> None:
        core = _core()
        (publication,) = core.startup_publications()
        self.assertEqual(publication.stream, "scenario")
        document = json.loads(publication.message.data)
        manifest = core.resolved.manifest()
        self.assertEqual(set(document), set(manifest) | {"asset_paths"})
        for key, value in manifest.items():
            with self.subTest(key=key):
                self.assertEqual(document[key], json.loads(json.dumps(value)))
        self.assertEqual(document["tasks"]["ui"]["focus"][0], "Course")
        self.assertEqual(set(document["asset_paths"]), {"robot", "pool", "tasks"})
        self.assertGreater(len(document["asset_paths"]["robot"]), 15)
        for role, assets in document["asset_paths"].items():
            declared = {a["id"] for a in document[role].get("assets", []) if a["status"] == "present"}
            self.assertEqual(set(assets), declared)
            for path in assets.values():
                self.assertTrue(Path(path).is_absolute() and Path(path).is_file(), path)
        self.assertIn("robot_magnet_mesh", document["asset_paths"]["robot"])
        self.assertIn("projectile_mesh", document["asset_paths"]["robot"])
        self.assertIn("pill_visual", document["asset_paths"]["tasks"])

    def test_real_time_factor_accepts_zero_and_rejects_bad_values(self) -> None:
        core = _core()
        self.assertEqual(core.real_time_factor, 1.0)
        self.assertIsNone(core.set_real_time_factor(0))
        self.assertEqual(core.real_time_factor, 0.0)
        self.assertIsNone(core.set_real_time_factor(2.5))
        for bad in (-1.0, math.nan, math.inf, "fast", True, None):
            with self.subTest(value=bad):
                self.assertIsNotNone(core.set_real_time_factor(bad))
                self.assertEqual(core.real_time_factor, 2.5)


class ValidationTest(unittest.TestCase):
    def _reject(self, edit: Any, text: str) -> None:
        with self.assertRaises(BridgeError) as caught:
            _core(edit)
        self.assertIn(text, str(caught.exception))

    @staticmethod
    def _stream(bridge: dict[str, Any], name: str) -> dict[str, Any]:
        return next(s for s in bridge["streams"] if s["id"] == name)

    def test_encoder_declarations_are_validated_at_construction(self) -> None:
        self._reject(lambda b: self._stream(b, "run_score").update(format="marker_array"),
                     "cannot use format")
        self._reject(lambda b: self._stream(b, "run_score").update(fields={"data": {"from": "json"}}),
                     "take no field map")
        self._reject(lambda b: self._stream(b, "run_score").update(options={"x": 1}),
                     "options: keys must be")
        self._reject(lambda b: self._stream(b, "run_score").update(frame_id="map"), "unstamped")
        self._reject(lambda b: self._stream(b, "run_score").update(message_type="std_msgs/msg/Bool"),
                     "string 'data'")
        self._reject(lambda b: self._stream(b, "magnet_lights").update(frame_id="odom"),
                     "world frame")
        self._reject(lambda b: self._stream(b, "task_events").update(rate_hz=5), "rate_hz 0")
        self._reject(lambda b: self._stream(b, "run_command").update(format="json"),
                     "publish streams")

    def test_marker_options_must_match_the_pack(self) -> None:
        self._reject(lambda b: self._stream(b, "magnet_lights")["options"]["colors"].pop("green"),
                     "no color for ['green']")
        self._reject(lambda b: self._stream(b, "projectiles")["options"].update(
            mesh_asset="launcher_mesh_nope"), "not present in the robot pack")
        self._reject(lambda b: self._stream(b, "projectiles")["options"]["namespaces"].pop("dropper"),
                     "no namespace for mechanism types ['dropper']")
        self._reject(lambda b: self._stream(b, "task_objects")["options"].pop("held_frame_id"),
                     "missing ['held_frame_id']")
        self._reject(lambda b: self._stream(b, "magnet_lights")["options"].update(
            scale_m=[1, 2]), "3 finite numbers")


if __name__ == "__main__":
    unittest.main()
