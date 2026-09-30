"""Field-map compilation tests using only generated ROS message classes (no middleware)."""

from __future__ import annotations

import unittest
from types import SimpleNamespace
from typing import Any

import numpy as np

try:
    import rosidl_runtime_py  # noqa: F401
    from nereus_ros.mapping import (
        BOOLEAN,
        INTEGER,
        MATRIX3,
        QUATERNION_WXYZ,
        SCALAR,
        STRING,
        TIME,
        VECTOR3,
        MappingError,
        Spec,
        compile_reader,
        compile_writer,
        message_class,
        parse_path,
        parse_type,
        ros_field,
        service_class,
    )
except ImportError as error:  # MappingError is undefined when the import itself fails
    raise unittest.SkipTest(f"ROS interfaces unavailable: {error}") from None

try:
    IMU = message_class("sensor_msgs/msg/Imu")
    TWIST = message_class("geometry_msgs/msg/TwistWithCovarianceStamped")
    CAMERA_INFO = message_class("sensor_msgs/msg/CameraInfo")
    FLOAT64 = message_class("std_msgs/msg/Float64")
    UINT8 = message_class("std_msgs/msg/UInt8")
    BOOL = message_class("std_msgs/msg/Bool")
    FLOAT_ARRAY = message_class("std_msgs/msg/Float32MultiArray")
    ODOMETRY = message_class("nav_msgs/msg/Odometry")
    TRIGGER = service_class("std_srvs/srv/Trigger")
    SET_POSE = service_class("robot_localization/srv/SetPose")
except (ImportError, MappingError) as error:
    raise unittest.SkipTest(f"ROS interfaces unavailable: {error}") from None

try:
    KILL_REPORT: Any = message_class("riptide_msgs2/msg/KillSwitchReport")
except (ImportError, MappingError):
    KILL_REPORT = None

STAMP_NS = 1790000000123456789


class PathAndTypeTest(unittest.TestCase):
    def test_parse_path_indexes(self) -> None:
        self.assertEqual(parse_path("twist.covariance[35]"),
                         [("twist", ()), ("covariance", (35,))])
        self.assertEqual(parse_path("esc_telemetry[0].thruster_ready"),
                         [("esc_telemetry", (0,)), ("thruster_ready", ())])
        self.assertEqual(parse_path("m[1][2]"), [("m", (1, 2))])

    def test_parse_path_invalid(self) -> None:
        for text in ("a+b", "", "a..b", "a[x]", "1a"):
            with self.subTest(text=text), self.assertRaises(MappingError):
                parse_path(text)

    def test_parse_type_fixed_array(self) -> None:
        parsed = parse_type("double[9]")
        self.assertEqual((parsed.base, parsed.length, parsed.sequence), ("double", 9, False))
        self.assertTrue(parsed.is_array)
        self.assertFalse(parsed.is_message)

    def test_parse_type_sequences(self) -> None:
        for text, base in (("sequence<float>", "float"), ("sequence<double, 5>", "double")):
            with self.subTest(text=text):
                parsed = parse_type(text)
                self.assertEqual((parsed.base, parsed.length, parsed.sequence),
                                 (base, None, True))

    def test_parse_type_bounded_string_and_message(self) -> None:
        parsed = parse_type("string<=10")
        self.assertEqual((parsed.base, parsed.is_array), ("string", False))
        message = parse_type("sequence<geometry_msgs/Vector3>")
        self.assertTrue(message.is_message and message.is_array)

    def test_ros_field_types(self) -> None:
        self.assertEqual(ros_field(TWIST, "twist.covariance[35]").base, "double")
        self.assertFalse(ros_field(TWIST, "twist.covariance[35]").is_array)
        self.assertEqual(ros_field(TWIST, "twist.covariance").length, 36)
        self.assertEqual(ros_field(TWIST, "header.stamp").base, "builtin_interfaces/Time")
        with self.assertRaises(MappingError):
            ros_field(TWIST, "twist.covariance[36]")


class ImuWriterTest(unittest.TestCase):
    FIELDS = {
        "header.stamp": {"from": "reading.stamp_ns"},
        "orientation.w": {"from": "reading.attitude.orientation_wxyz[0]"},
        "orientation.x": {"from": "reading.attitude.orientation_wxyz[1]"},
        "orientation.y": {"from": "reading.attitude.orientation_wxyz[2]"},
        "orientation.z": {"from": "reading.attitude.orientation_wxyz[3]"},
        "orientation_covariance": {"from": "reading.attitude.covariance"},
        "angular_velocity": {"from": "reading.gyro"},
    }
    SOURCES = {"reading": {
        "stamp_ns": TIME,
        "attitude": {"orientation_wxyz": QUATERNION_WXYZ, "covariance": MATRIX3},
        "gyro": VECTOR3,
    }}

    def values(self) -> Any:
        return {"reading": SimpleNamespace(
            stamp_ns=STAMP_NS,
            attitude={"orientation_wxyz": np.array([0.5, 0.1, 0.2, 0.3]),
                      "covariance": np.arange(1.0, 10.0).reshape(3, 3).T},
            gyro=np.array([0.01, -0.02, 0.03]))}

    def test_imu_message(self) -> None:
        writer = compile_writer(IMU, self.FIELDS, self.SOURCES, frame_id="imu_link", where="imu")
        message = writer(self.values())
        self.assertIsInstance(message, IMU)
        self.assertEqual(message.header.frame_id, "imu_link")
        self.assertEqual(message.header.stamp.sec, 1790000000)
        self.assertEqual(message.header.stamp.nanosec, 123456789)
        self.assertEqual(
            (message.orientation.w, message.orientation.x, message.orientation.y,
             message.orientation.z), (0.5, 0.1, 0.2, 0.3))
        # The transposed arange matrix is [[1,4,7],[2,5,8],[3,6,9]]; row-major flatten.
        self.assertEqual(list(message.orientation_covariance),
                         [1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0])
        self.assertEqual((message.angular_velocity.x, message.angular_velocity.y,
                          message.angular_velocity.z), (0.01, -0.02, 0.03))

    def test_frame_id_optional_when_not_given(self) -> None:
        writer = compile_writer(IMU, self.FIELDS, self.SOURCES)
        self.assertEqual(writer(self.values()).header.frame_id, "")

    def test_writer_returns_fresh_message(self) -> None:
        writer = compile_writer(IMU, self.FIELDS, self.SOURCES, frame_id="imu_link")
        self.assertIsNot(writer(self.values()), writer(self.values()))

    def test_source_index_out_of_range(self) -> None:
        fields = {"orientation.w": {"from": "reading.attitude.orientation_wxyz[4]"}}
        with self.assertRaises(MappingError):
            compile_writer(IMU, fields, self.SOURCES)

    def test_matrix_needs_nine_elements(self) -> None:
        sources = {"m": Spec("float", (2, 2))}
        with self.assertRaises(MappingError):
            compile_writer(IMU, {"orientation_covariance": {"from": "m"}}, sources)


class IndexedDestinationTest(unittest.TestCase):
    def test_covariance_element(self) -> None:
        sources = {"reading": {"covariance": MATRIX3}}
        writer = compile_writer(
            TWIST, {"twist.covariance[35]": {"from": "reading.covariance[0][0]"}}, sources)
        matrix = np.zeros((3, 3))
        matrix[0][0] = 4.5
        message = writer({"reading": {"covariance": matrix}})
        self.assertEqual(message.twist.covariance[35], 4.5)
        self.assertEqual(sum(message.twist.covariance), 4.5)

    def test_destination_index_out_of_range(self) -> None:
        sources = {"reading": {"covariance": MATRIX3}}
        with self.assertRaises(MappingError):
            compile_writer(
                TWIST, {"twist.covariance[36]": {"from": "reading.covariance[0][0]"}}, sources)


class ConstantTest(unittest.TestCase):
    def test_int_constant_range(self) -> None:
        message = compile_writer(UINT8, {"data": {"constant": 255}}, {})({})
        self.assertEqual(message.data, 255)
        for value in (256, -1):
            with self.subTest(value=value), self.assertRaises(MappingError):
                compile_writer(UINT8, {"data": {"constant": value}}, {})

    def test_float_array_constants(self) -> None:
        distortion = [0.1, -0.2, 0.0, 0.0, 0.05]
        rectification = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        writer = compile_writer(
            CAMERA_INFO,
            {"d": {"constant": distortion}, "r": {"constant": rectification}}, {})
        message = writer({})
        self.assertEqual(list(message.d), distortion)
        self.assertEqual(list(message.r), rectification)

    def test_mixed_int_float_array_constant(self) -> None:
        message = compile_writer(
            CAMERA_INFO, {"r": {"constant": [1, 0, 0, 0, 1.0, 0, 0, 0, 1]}}, {})({})
        self.assertEqual(list(message.r), [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])

    def test_wrong_length_fixed_array_rejected(self) -> None:
        with self.assertRaises(MappingError):
            compile_writer(CAMERA_INFO, {"r": {"constant": [1.0] * 8}}, {})

    def test_bool_constant(self) -> None:
        self.assertIs(compile_writer(BOOL, {"data": {"constant": True}}, {})({}).data, True)
        self.assertIs(compile_writer(BOOL, {"data": {"constant": False}}, {})({}).data, False)

    def test_bool_constant_not_a_number(self) -> None:
        with self.assertRaises(MappingError):
            compile_writer(FLOAT64, {"data": {"constant": True}}, {})

    def test_int_constant_into_float(self) -> None:
        self.assertEqual(compile_writer(FLOAT64, {"data": {"constant": 3}}, {})({}).data, 3.0)


class RejectionTest(unittest.TestCase):
    SOURCES = {"a": SCALAR, "v": VECTOR3, "q": QUATERNION_WXYZ, "s": STRING, "t": TIME}

    def assertRejected(self, cls: Any, fields: dict[str, Any], **kwargs: Any) -> str:
        with self.assertRaises(MappingError) as caught:
            compile_writer(cls, fields, self.SOURCES, **kwargs)
        return str(caught.exception)

    def test_unknown_destination_field(self) -> None:
        self.assertRejected(FLOAT64, {"nonsense": {"from": "a"}})
        self.assertRejected(IMU, {"orientation.w2": {"from": "a"}})

    def test_unknown_native_source(self) -> None:
        self.assertRejected(FLOAT64, {"data": {"from": "missing"}})
        self.assertRejected(FLOAT64, {"data": {"from": "v.x"}})

    def test_quaternion_whole_assignment(self) -> None:
        message = self.assertRejected(IMU, {"orientation": {"from": "q"}})
        self.assertIn("component", message)

    def test_vector_from_scalar(self) -> None:
        self.assertRejected(IMU, {"angular_velocity": {"from": "a"}})

    def test_string_into_double(self) -> None:
        self.assertRejected(FLOAT64, {"data": {"from": "s"}})

    def test_float_into_int(self) -> None:
        self.assertRejected(UINT8, {"data": {"from": "a"}})

    def test_vector_into_scalar(self) -> None:
        self.assertRejected(FLOAT64, {"data": {"from": "v"}})

    def test_stamped_requires_frame_id(self) -> None:
        self.assertRejected(IMU, {"header.stamp": {"from": "t"}}, frame_id="")

    def test_unstamped_rejects_frame_id(self) -> None:
        self.assertRejected(FLOAT64, {"data": {"from": "a"}}, frame_id="base_link")

    def test_unstamped_accepts_empty_frame_id(self) -> None:
        compile_writer(FLOAT64, {"data": {"from": "a"}}, self.SOURCES, frame_id="")

    def test_stamped_requires_header_stamp_mapping(self) -> None:
        message = self.assertRejected(IMU, {"angular_velocity": {"from": "v"}},
                                      frame_id="imu_link")
        self.assertIn("header.stamp", message)

    def test_time_needs_time_source(self) -> None:
        self.assertRejected(IMU, {"header.stamp": {"from": "a"}}, frame_id="imu_link")

    def test_whole_message_assignment(self) -> None:
        self.assertRejected(TWIST, {"twist": {"from": "v"}})


class EnumMapTest(unittest.TestCase):
    SOURCES = {"state": {"mode": STRING, "count": INTEGER}}

    def writer(self) -> Any:
        return compile_writer(UINT8, {"data": {"from": "state.mode",
                                               "enum_map": {"idle": 0, "run": 2}}},
                              self.SOURCES)

    def test_mapped_values(self) -> None:
        writer = self.writer()
        self.assertEqual(writer({"state": {"mode": "run"}}).data, 2)
        self.assertEqual(writer({"state": SimpleNamespace(mode="idle")}).data, 0)

    def test_unmapped_state_raises_at_call(self) -> None:
        writer = self.writer()
        with self.assertRaises(MappingError) as caught:
            writer({"state": {"mode": "fault"}})
        self.assertIn("fault", str(caught.exception))

    def test_enum_map_on_non_string_source(self) -> None:
        with self.assertRaises(MappingError):
            compile_writer(UINT8, {"data": {"from": "state.count", "enum_map": {"1": 1}}},
                           self.SOURCES)

    def test_enum_value_out_of_range(self) -> None:
        with self.assertRaises(MappingError):
            compile_writer(UINT8, {"data": {"from": "state.mode",
                                            "enum_map": {"run": 256}}}, self.SOURCES)


class ReaderTest(unittest.TestCase):
    def test_float_array(self) -> None:
        reader = compile_reader(FLOAT_ARRAY, {"forces_n": {"from": "data"}},
                                {"forces_n": Spec("float", (None,))})
        message = FLOAT_ARRAY(data=[1.0, 2.5, -3.0])
        result = reader(message)
        self.assertEqual(set(result), {"forces_n"})
        np.testing.assert_array_equal(result["forces_n"], [1.0, 2.5, -3.0])

    def test_float_array_wrong_rank(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(FLOAT_ARRAY, {"forces_n": {"from": "data"}}, {"forces_n": SCALAR})

    def test_odometry(self) -> None:
        arguments = {"position": VECTOR3, "qw": SCALAR, "qx": SCALAR, "qy": SCALAR,
                     "qz": SCALAR, "frame": STRING}
        fields = {
            "position": {"from": "pose.pose.position"},
            "qw": {"from": "pose.pose.orientation.w"},
            "qx": {"from": "pose.pose.orientation.x"},
            "qy": {"from": "pose.pose.orientation.y"},
            "qz": {"from": "pose.pose.orientation.z"},
            "frame": {"from": "header.frame_id"},
        }
        reader = compile_reader(ODOMETRY, fields, arguments, where="odom")
        message = ODOMETRY()
        message.header.frame_id = "world"
        message.pose.pose.position.x = 1.0
        message.pose.pose.position.y = 2.0
        message.pose.pose.position.z = 3.0
        message.pose.pose.orientation.w = 0.5
        message.pose.pose.orientation.x = 0.1
        message.pose.pose.orientation.y = 0.2
        message.pose.pose.orientation.z = 0.3
        result = reader(message)
        self.assertIsInstance(result["position"], np.ndarray)
        np.testing.assert_array_equal(result["position"], [1.0, 2.0, 3.0])
        self.assertEqual([result[key] for key in ("qw", "qx", "qy", "qz")],
                         [0.5, 0.1, 0.2, 0.3])
        self.assertEqual(result["frame"], "world")

    def test_int_field_reads_as_int(self) -> None:
        reader = compile_reader(UINT8, {"n": {"from": "data"}}, {"n": INTEGER})
        value = reader(UINT8(data=7))["n"]
        self.assertEqual(value, 7)
        self.assertIsInstance(value, int)

    def test_missing_and_extra_arguments(self) -> None:
        arguments = {"a": SCALAR, "b": SCALAR}
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"from": "data"}}, arguments)
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"from": "data"}, "b": {"from": "data"},
                                     "c": {"from": "data"}}, arguments)

    def test_reader_rejects_non_from_maps(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"constant": 1.0}}, {"a": SCALAR})

    def test_type_mismatch(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"from": "data"}}, {"a": STRING})
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"from": "data"}}, {"a": VECTOR3})

    def test_read_whole_quaternion_rejected(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(ODOMETRY, {"q": {"from": "pose.pose.orientation"}},
                           {"q": QUATERNION_WXYZ})

    def test_read_whole_message_rejected(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(ODOMETRY, {"q": {"from": "pose.pose"}}, {"q": SCALAR})

    def test_unknown_ros_field(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(FLOAT64, {"a": {"from": "nope"}}, {"a": SCALAR})

    def test_no_filter_accepts_everything(self) -> None:
        reader = compile_reader(FLOAT64, {"a": {"from": "data"}}, {"a": SCALAR})
        self.assertTrue(reader.accepts(FLOAT64(data=1.0)))


@unittest.skipUnless(KILL_REPORT is not None, "riptide_msgs2 is not installed")
class AcceptIfTest(unittest.TestCase):
    def reader(self) -> Any:
        return compile_reader(
            KILL_REPORT, {"asserting": {"from": "switch_asserting_kill"}},
            {"asserting": BOOLEAN},
            accept_if=[{"field": "kill_switch_id", "equals": 1}], where="kill")

    def test_filter(self) -> None:
        reader = self.reader()
        wanted = KILL_REPORT(kill_switch_id=1, switch_asserting_kill=True)
        other = KILL_REPORT(kill_switch_id=2, switch_asserting_kill=True)
        self.assertTrue(reader.accepts(wanted))
        self.assertFalse(reader.accepts(other))
        self.assertEqual(reader(wanted), {"asserting": True})

    def test_filter_type_mismatch(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(KILL_REPORT, {"asserting": {"from": "switch_asserting_kill"}},
                           {"asserting": BOOLEAN},
                           accept_if=[{"field": "kill_switch_id", "equals": "one"}])

    def test_filter_string_field(self) -> None:
        reader = compile_reader(
            KILL_REPORT, {"asserting": {"from": "switch_asserting_kill"}},
            {"asserting": BOOLEAN}, accept_if=[{"field": "sender_id", "equals": "left"}])
        self.assertTrue(reader.accepts(KILL_REPORT(sender_id="left")))
        self.assertFalse(reader.accepts(KILL_REPORT(sender_id="right")))
        with self.assertRaises(MappingError):
            compile_reader(KILL_REPORT, {"asserting": {"from": "switch_asserting_kill"}},
                           {"asserting": BOOLEAN},
                           accept_if=[{"field": "sender_id", "equals": 1}])

    def test_filter_unknown_field(self) -> None:
        with self.assertRaises(MappingError):
            compile_reader(KILL_REPORT, {"asserting": {"from": "switch_asserting_kill"}},
                           {"asserting": BOOLEAN},
                           accept_if=[{"field": "missing", "equals": 1}])


class ServiceMapTest(unittest.TestCase):
    def test_trigger_response_writer(self) -> None:
        writer = compile_writer(
            TRIGGER.Response,
            {"success": {"from": "accepted"}, "message": {"from": "message"}},
            {"accepted": BOOLEAN, "message": STRING}, where="trigger")
        response = writer({"accepted": True, "message": "armed"})
        self.assertIsInstance(response, TRIGGER.Response)
        self.assertIs(response.success, True)
        self.assertEqual(response.message, "armed")

    def test_trigger_response_unknown_field(self) -> None:
        with self.assertRaises(MappingError):
            compile_writer(TRIGGER.Response, {"accepted": {"from": "accepted"}},
                           {"accepted": BOOLEAN})

    def test_set_pose_request_field(self) -> None:
        field = ros_field(SET_POSE.Request, "pose")
        self.assertEqual(field.base, "geometry_msgs/PoseWithCovarianceStamped")
        self.assertTrue(field.is_message)
        self.assertEqual(ros_field(SET_POSE.Request, "pose.pose.pose.position").base,
                         "geometry_msgs/Point")

    def test_set_pose_request_reader(self) -> None:
        reader = compile_reader(
            SET_POSE.Request,
            {"position": {"from": "pose.pose.pose.position"},
             "frame": {"from": "pose.header.frame_id"}},
            {"position": VECTOR3, "frame": STRING})
        request = SET_POSE.Request()
        request.pose.header.frame_id = "odom"
        request.pose.pose.pose.position.z = -2.0
        result = reader(request)
        np.testing.assert_array_equal(result["position"], [0.0, 0.0, -2.0])
        self.assertEqual(result["frame"], "odom")

    def test_unknown_service(self) -> None:
        with self.assertRaises(MappingError):
            service_class("std_srvs/srv/NoSuchService")
        with self.assertRaises(MappingError):
            message_class("no_such_pkg/msg/Thing")


if __name__ == "__main__":
    unittest.main()
