"""Alignment coalescing and service acknowledgements are independently observable."""

import copy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest import SkipTest, TestCase
from unittest.mock import Mock

try:
    from rclpy.task import Future
    from robot_localization.srv import SetPose
    from robotics_platform import Pose
    from robotics_platform.pack_runtime import create_runtime
    from robotics_platform.packs import resolve_scenario
    from robotics_platform_ros.core import BridgeCore, Counters
    from robotics_platform_ros.node import BridgeNode
except ImportError as error:
    raise SkipTest(f"ROS alignment dependencies unavailable: {error}") from None


class AlignmentTests(TestCase):
    def test_placement_supersedes_unsent_startup_and_uses_latest_pose(self):
        root = Path(__file__).resolve().parents[3]
        config = resolve_scenario(root / "content/packs/scenarios/talos_uwrt")
        # Alignment exercises navigation independently of optional camera rendering.
        bridge = copy.deepcopy(config.bridge)
        bridge["streams"] = [
            stream for stream in bridge["streams"]
            if not stream["native"].startswith(("sensor:ffc.", "sensor:dfc."))
        ]
        config = replace(config, bridge=bridge)
        core = BridgeCore(
            config,
            create_runtime(config, sensor_ids=["imu", "fog", "dvl", "depth"]),
            epoch_ns=1_000_000_000,
            lookup=lambda target, source: Pose(),
        )
        self.assertIsNone(core.pending_alignment())  # no estimate frame yet
        request = SetPose.Request()
        request.pose.header.frame_id = "map"
        pose = request.pose.pose.pose
        pose.position.x, pose.position.y, pose.position.z = 8.0, 8.0, -1.0
        pose.orientation.w = 1.0
        core.call("set_sim_pose", request)
        self.assertEqual(core.counters.alignments_superseded, {"startup": 1})
        core.latest_estimate = {"frame": "odom"}
        alignment = core.pending_alignment()
        self.assertIsNotNone(alignment)
        self.assertEqual(alignment.trigger, "placement")
        self.assertEqual(alignment.request.pose.header.frame_id, "odom")
        self.assertAlmostEqual(alignment.request.pose.pose.pose.position.x, 8.0)
        self.assertAlmostEqual(alignment.request.pose.pose.pose.position.z, -1.0)
        self.assertEqual(core.counters.alignments, {"placement": 1})
        self.assertIsNone(core.pending_alignment())  # never resends an old pose

    def test_service_failure_is_not_counted_as_acknowledgement(self):
        counters = Counters()
        logger = Mock()
        node = SimpleNamespace(core=SimpleNamespace(counters=counters), get_logger=lambda: logger)
        good, failed = Future(), Future()
        good.set_result(SetPose.Response())
        failed.set_exception(RuntimeError("transport failure"))
        BridgeNode.alignment_finished(node, "placement", good)
        BridgeNode.alignment_finished(node, "startup", failed)
        self.assertEqual(counters.alignments_acknowledged, {"placement": 1})
        self.assertEqual(counters.alignments_failed, {"startup": 1})
        logger.error.assert_called_once()
