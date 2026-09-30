"""Camera stream preflight, physics ownership and ROS delivery on a generic layout."""

import copy
import os
import time
import unittest
from types import SimpleNamespace

from nereus_ros.__main__ import select_sensors
from nereus_ros.camera_bridge import CameraBridge
from nereus_ros.core import BridgeCore, BridgeError
from test_bridge_core import BRIDGE, EPOCH_NS, QOS, RESET, ROBOT, _make
from test_camera_bridge import Provider, resolved


def configured():
    camera = resolved()
    robot = copy.deepcopy(ROBOT)
    robot["sensors"] += camera.robot["sensors"]
    bridge = copy.deepcopy(BRIDGE)
    bridge["frame_names"].update(camera.bridge["frame_names"])
    stream = camera.bridge["streams"][0]
    stream.update(topic="camera/depth", qos=QOS)
    bridge["streams"].append(stream)
    bridge["services"] = [{**RESET, "service": "reset"}]
    return SimpleNamespace(robot=robot, bridge=bridge, scenario={"world_frame": "scenario_world", "seed": 0})


class CameraIntegrationTests(unittest.TestCase):
    def test_selection_rejects_unknown_duplicate_disabled_and_separates_cameras(self):
        data = configured()
        self.assertEqual(select_sensors(data, None), (["alt"], ["cam"]))
        self.assertEqual(select_sensors(data, ["cam"]), ([], ["cam"]))
        self.assertEqual(select_sensors(data, []), ([], []))
        for selection in (["unknown"], ["cam", "cam"]):
            with self.assertRaises(ValueError):
                select_sensors(data, selection)
        data.robot["sensors"][-1]["enabled"] = False
        with self.assertRaises(ValueError):
            select_sensors(data, ["cam"])
        self.assertEqual(select_sensors(data, None), (["alt"], []))

    def test_camera_streams_require_selected_provider_and_placement_discards_old_pixels(self):
        from std_srvs.srv import Trigger

        data, pack = configured(), _make().pack
        with self.assertRaises(BridgeError):
            BridgeCore(data, pack, epoch_ns=EPOCH_NS)
        provider, published = Provider(), []
        worker = CameraBridge(data, provider, published.extend)
        self.addCleanup(worker.close)
        self.addCleanup(provider.release.set)
        core = BridgeCore(data, pack, epoch_ns=EPOCH_NS, cameras=worker)
        worker.start()
        for _ in range(20):
            core.step()  # No rendering is started by advancing physics alone.
        self.assertEqual(provider.calls, [])
        worker.acquire(pack.runtime.observe(), core.clock_ns())
        self.assertTrue(provider.entered.wait(2))
        result = core.call("reset", Trigger.Request())
        self.assertTrue(result.success)
        provider.release.set()
        worker.close()
        self.assertEqual(published, [])
        self.assertEqual(worker.stats()["cam"]["discarded_stale"], 1)

    def test_uncoordinated_native_reset_fails_before_camera_seed_can_diverge(self):
        data, pack = configured(), _make().pack
        worker = CameraBridge(data, Provider(), lambda _: None)
        core = BridgeCore(data, pack, epoch_ns=EPOCH_NS, cameras=worker)
        pack.runtime.generation += 1
        with self.assertRaisesRegex(BridgeError, "coordinated full reset"):
            core.step()
        worker.close()

    @unittest.skipUnless(os.environ.get("NEREUS_TEST_ROS_LIVE") == "1", "live ROS test is opt-in")
    def test_live_node_publishes_camera_acquisition_stamp_frame_and_owned_depth(self):
        import rclpy
        from rclpy.node import Node
        from nereus_ros.node import BridgeNode
        from sensor_msgs.msg import Image

        data, pack = configured(), _make().pack
        provider = Provider()
        provider.release.set()
        worker = CameraBridge(data, provider, lambda _: None)
        rclpy.init()
        node = listener = None
        try:
            node = BridgeNode(lambda lookup: BridgeCore(
                data, pack, epoch_ns=EPOCH_NS, lookup=lookup, cameras=worker), "/camera_test")
            listener = Node("camera_contract_listener")
            received = []
            listener.create_subscription(Image, "/camera_test/camera/depth", received.append, 10)
            node.start_cameras()
            deadline = time.monotonic() + 5
            while not received and time.monotonic() < deadline:
                node.tick()
                rclpy.spin_once(node, timeout_sec=0)
                rclpy.spin_once(listener, timeout_sec=0.01)
            self.assertTrue(received)
            stamp = received[0].header.stamp
            self.assertGreater(stamp.sec * 1_000_000_000 + stamp.nanosec, EPOCH_NS)
            self.assertLessEqual(stamp.sec * 1_000_000_000 + stamp.nanosec, node.core.clock_ns())
            self.assertEqual(received[0].header.frame_id, "auv/optical")
            self.assertEqual(received[0].encoding, "32FC1")
            self.assertEqual(len(received[0].data), 24)
        finally:
            worker.close()
            if listener is not None:
                listener.destroy_node()
            if node is not None:
                node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
