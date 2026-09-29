"""Camera ROS wire formats: acquisition stamps, calibrated metadata and bulk-owned pixels."""

import copy
import sys
import unittest
from types import SimpleNamespace

import numpy as np

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from robotics_platform_ros.images import compile_image_writer, compile_info_writer
    from robotics_platform_ros.mapping import MappingError
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
except ImportError as error:
    raise unittest.SkipTest(f"camera message test dependencies unavailable: {error}") from None

STAMP = 1_700_000_000_123_456_789


def stream(output="depth_left", encoding="32FC1"):
    return {"id": "camera", "native": f"sensor:rig.{output}", "direction": "publish",
            "frame_id": "vehicle/optical", "message_type": "sensor_msgs/msg/Image",
            "image": {"encoding": encoding}, "fields": {"header.stamp": {"from": "sample.time"}}}


def jpeg_stream():
    config = stream("rgb_left", "rgb8")
    config["message_type"] = "sensor_msgs/msg/CompressedImage"
    config["image"] = {"compression": "jpeg", "quality": 93,
                       "source_encoding": "rgb8", "compressed_encoding": "bgr8"}
    return config


class CameraImageTests(unittest.TestCase):
    def assert_stamp(self, message):
        self.assertEqual(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                         STAMP)
        self.assertEqual(message.header.frame_id, "vehicle/optical")

    def test_depth_wire_data_keeps_metres_nan_rows_and_acquisition_stamp(self):
        depth = np.array([[1, np.nan, 3], [4, 5, 6]], dtype=np.float32)
        message = compile_image_writer(stream())(SimpleNamespace(width=3, height=2, depth=depth),
                                                STAMP)
        self.assert_stamp(message)
        self.assertEqual((message.width, message.height, message.step), (3, 2, 12))
        self.assertEqual(message.encoding, "32FC1")
        self.assertEqual(message.is_bigendian, sys.byteorder == "big")
        decoded = deserialize_message(serialize_message(message), Image)
        np.testing.assert_array_equal(np.frombuffer(decoded.data, dtype=np.float32).reshape(2, 3),
                                      depth)
        depth[:] = 42
        self.assertEqual(np.frombuffer(message.data, dtype=np.float32)[0], 1)

    def test_rgb_and_bgr_rows_are_tight_owned_and_channels_explicit(self):
        rgb = np.array([[[255, 0, 5], [0, 255, 6]], [[7, 0, 255], [8, 9, 10]]], np.uint8)
        frame = SimpleNamespace(width=2, height=2, rgb=rgb)
        for encoding in ("rgb8", "bgr8"):
            with self.subTest(encoding=encoding):
                message = compile_image_writer(stream("rgb_right", encoding))(frame, STAMP)
                self.assert_stamp(message)
                self.assertEqual(message.step, 6)
                decoded = deserialize_message(serialize_message(message), Image)
                expected = rgb if encoding == "rgb8" else rgb[:, :, ::-1]
                np.testing.assert_array_equal(np.frombuffer(decoded.data, np.uint8).reshape(2, 2, 3),
                                              expected)

    def test_jpeg_preserves_native_payload_and_original_transport_format(self):
        payload = b"\xff\xd8encoded-test-payload\xff\xd9"
        writer = compile_image_writer(jpeg_stream())
        self.assertEqual(writer.jpeg_quality, 93)
        message = writer(SimpleNamespace(jpeg=payload), STAMP)
        self.assert_stamp(message)
        self.assertEqual(message.format, "rgb8; jpeg compressed bgr8")
        decoded = deserialize_message(serialize_message(message), CompressedImage)
        self.assertEqual(bytes(decoded.data), payload)
        with self.assertRaises(MappingError):
            writer(SimpleNamespace(jpeg=b""), STAMP)

    def test_bad_stream_types_encodings_and_metadata_overrides_fail_before_capture(self):
        cases = []
        for output, encoding in (("depth_left", "16UC1"), ("rgb_left", "32FC1"),
                                 ("camera_info", "rgb8")):
            cases.append(stream(output, encoding))
        config = jpeg_stream()
        config["image"]["compressed_encoding"] = "rgb8"
        cases.append(config)
        config = stream()
        config["message_type"] = "sensor_msgs/msg/CompressedImage"
        cases.append(config)
        config = stream()
        config["fields"]["width"] = {"constant": 7}
        cases.append(config)
        config = stream()
        config["fields"]["header.frame_id"] = {"constant": "wrong"}
        cases.append(config)
        config = stream()
        config["direction"] = "subscribe"
        cases.append(config)
        for config in cases:
            with self.subTest(config=config), self.assertRaises(MappingError):
                compile_image_writer(config)
        for quality in (0, 101, True, 93.5):
            config = jpeg_stream()
            config["image"]["quality"] = quality
            with self.subTest(quality=quality), self.assertRaises(MappingError):
                compile_image_writer(config)

    def test_invalid_frame_shapes_and_scalar_types_are_rejected(self):
        writer = compile_image_writer(stream())
        for depth in (np.zeros((2, 3), np.float64), np.zeros((3, 2), np.float32)):
            with self.assertRaises(MappingError):
                writer(SimpleNamespace(width=3, height=2, depth=depth), STAMP)

    def test_camera_info_maps_rectified_projection_including_right_baseline(self):
        config = stream("camera_info_right")
        config.pop("image")
        config["message_type"] = "sensor_msgs/msg/CameraInfo"
        config["fields"].update({
            "width": {"from": "info.width"}, "height": {"from": "info.height"},
            "k": {"from": "info.k"}, "p": {"from": "info.p"},
            "d": {"constant": [0., 0., 0., 0., 0.]},
            "r": {"constant": [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
            "distortion_model": {"constant": "plumb_bob"},
        })
        info = {"width": 640, "height": 480, "k": [500., 0., 320., 0., 500., 240., 0., 0., 1.],
                "p": [500., 0., 320., -25., 0., 500., 240., 0., 0., 0., 1., 0.]}
        message = compile_info_writer(config)({"sample": {"time": STAMP}, "info": info})
        decoded = deserialize_message(serialize_message(message), CameraInfo)
        self.assert_stamp(decoded)
        self.assertEqual(decoded.p[3], -25.)
        np.testing.assert_array_equal(decoded.d, np.zeros(5))
        wrong = copy.deepcopy(config)
        wrong["fields"]["header.stamp"] = {"constant": 0}
        with self.assertRaises(MappingError):
            compile_info_writer(wrong)


if __name__ == "__main__":
    unittest.main()
