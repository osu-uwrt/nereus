"""Fixed robot frames are data-driven and persist for late ROS subscribers."""

import copy
import os
import time
import unittest

import numpy as np
from robotics_platform_ros.core import BridgeError
from test_bridge_core import BRIDGE, EPOCH_NS, OFFSET, WORLD, _make


def configured():
    bridge = copy.deepcopy(BRIDGE)
    bridge["frame_names"].update(com="vehicle/root", base="vehicle/mount")
    bridge["tf"] = {"publish": [], "lookup": [], "never_publish": [], "static": [{
        "parent": "vehicle/root", "child": "vehicle/mount",
        "from_frame": "com", "to_frame": "base"}]}
    return bridge


class StaticTransformTests(unittest.TestCase):
    def test_pose_comes_from_robot_frame_tree_and_is_not_dynamic(self):
        core = _make(configured())
        transform, = core.static_transforms
        self.assertEqual((transform.parent, transform.child), ("vehicle/root", "vehicle/mount"))
        self.assertEqual(transform.stamp_ns, EPOCH_NS)
        np.testing.assert_allclose(transform.translation, OFFSET)
        np.testing.assert_allclose(transform.orientation_wxyz, [1, 0, 0, 0])
        self.assertEqual(core.step()[2], [])

    def test_unknown_native_frames_and_wrong_ros_alias_are_rejected(self):
        for key, value in (("from_frame", "ghost"), ("to_frame", "ghost"),
                           ("parent", "wrong_parent"), ("child", "wrong_child")):
            with self.subTest(key=key):
                bridge = configured()
                bridge["tf"]["static"][0][key] = value
                with self.assertRaises(BridgeError):
                    _make(bridge)

    def test_duplicate_ownership_cycles_and_forbidden_children_are_rejected(self):
        bridge = configured()
        bridge["tf"]["lookup"] = [{"parent": "external", "child": "vehicle/mount"}]
        with self.assertRaisesRegex(BridgeError, "duplicate"):
            _make(bridge)
        bridge = configured()
        bridge["tf"]["publish"] = [{"parent": WORLD, "child": "vehicle/mount",
                                    "native": "state:robot.reference_pose", "rate_hz": 10}]
        with self.assertRaisesRegex(BridgeError, "duplicate"):
            _make(bridge)
        bridge = configured()
        bridge["tf"]["lookup"] = [{"parent": "vehicle/mount", "child": "vehicle/root"}]
        with self.assertRaisesRegex(BridgeError, "cycle"):
            _make(bridge)
        bridge = configured()
        bridge["tf"]["never_publish"] = ["vehicle/*"]
        with self.assertRaisesRegex(BridgeError, "never_publish"):
            _make(bridge)

    @unittest.skipUnless(os.environ.get("RP_TEST_ROS_LIVE") == "1", "live ROS test is opt-in")
    def test_late_listener_receives_static_transform_with_no_simulation_step(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from robotics_platform_ros.node import BridgeNode
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        bridge = listener = None
        try:
            core = _make(configured())
            bridge = BridgeNode(lambda lookup: core, "/static_test")
            listener = Node("late_static_listener")
            received = []
            listener.create_subscription(
                TFMessage, "/tf_static", received.append,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            deadline = time.monotonic() + 5
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(bridge, timeout_sec=0)
                rclpy.spin_once(listener, timeout_sec=0.05)
            self.assertTrue(received, "late subscriber did not receive the static transform")
            transform = next(t for msg in received for t in msg.transforms
                             if t.child_frame_id == "vehicle/mount")
            self.assertEqual(transform.header.frame_id, "vehicle/root")
            self.assertAlmostEqual(transform.transform.translation.x, OFFSET[0])
            self.assertEqual(core.runtime.tick, 0)
        finally:
            if listener is not None:
                listener.destroy_node()
            if bridge is not None:
                bridge.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
