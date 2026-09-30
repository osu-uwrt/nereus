// Field-map compiler tests (semantics of integrations/ros2/tests/test_bridge_mapping.py) over
// real ROS message types through rosidl introspection; no middleware.
#include "mapping.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <gtest/gtest.h>

using namespace nereus::ros_bridge;

namespace {
std::shared_ptr<const MessageType> type(const char *name) {
    return MessageType::get(name);
}
SpecTree tree(std::map<std::string, SpecTree> nodes) {
    return SpecTree(std::move(nodes));
}
SpecTree sim() {
    return tree({{"time", timeSpec()}});
}

template <class Fn> std::string errorOf(Fn &&fn) {
    try {
        fn();
    } catch (const MappingError &error) {
        return error.what();
    }
    return "";
}
Writer writerFor(const char *message, const Json &fields, const SpecTree &sources,
                 std::optional<std::string> frame = std::nullopt) {
    return compileWriter(type(message)->members(), fields, sources, frame, "test");
}
} // namespace

TEST(Path, ParsesDottedNamesAndIndexes) {
    const auto tokens = parsePath("a.b[2].c[1][3]");
    ASSERT_EQ(tokens.size(), 3u);
    EXPECT_EQ(tokens[1].indexes, std::vector<int>{2});
    EXPECT_EQ(tokens[2].indexes, (std::vector<int>{1, 3}));
    for (const char *bad : {"", "1a", "a..b", "a.", "a[", "a[x]", "a[-1]", "a b", "a[1]b"})
        EXPECT_THROW(parsePath(bad), MappingError) << bad;
}

TEST(SourceRef, ValidatesAgainstTheSpecTree) {
    const SpecTree sources =
        tree({{"reading", tree({{"m", matrix3Spec()}, {"v", vector3Spec()}, {"s", scalarSpec()}})}});
    Spec spec;
    const auto element = SourceRef::compile(sources, "reading.m[1][2]", spec);
    EXPECT_EQ(spec, scalarSpec());
    const Value value = Value::map({{"reading", Value::map({{"m", Value::array({0, 1, 2, 3, 4, 5, 6, 7, 8})}})}});
    EXPECT_EQ(element.read(value).f, 5.0);
    const auto row = SourceRef::compile(sources, "reading.m[2]", spec);
    EXPECT_EQ(spec, vector3Spec());
    EXPECT_EQ(row.read(value).a, (std::vector<double>{6, 7, 8}));
    EXPECT_NE(errorOf([&] { SourceRef::compile(sources, "reading.x", spec); }).find("unknown native field 'reading.x'"),
              std::string::npos);
    EXPECT_NE(errorOf([&] { SourceRef::compile(sources, "reading.s[0]", spec); }).find("not indexable"),
              std::string::npos);
    EXPECT_NE(errorOf([&] { SourceRef::compile(sources, "reading.v[3]", spec); }).find("out of range"),
              std::string::npos);
    EXPECT_NE(errorOf([&] { SourceRef::compile(sources, "reading", spec); }).find("structure, not a value"),
              std::string::npos);
}

TEST(Writer, StampsHeadersAndAssignsFieldsAndArrays) {
    const SpecTree sources =
        tree({{"sample", sim()}, {"reading", tree({{"cov", matrix3Spec()}, {"f", vector3Spec()}})}});
    const Json fields = {{"header.stamp", {{"from", "sample.time"}}},
                         {"linear_acceleration", {{"from", "reading.f"}}},
                         {"orientation_covariance", {{"from", "reading.cov"}}},
                         {"orientation.w", {{"constant", 1.0}}}};
    const auto writer = writerFor("sensor_msgs/msg/Imu", fields, sources, "imu_link");
    const auto message =
        writer.make(type("sensor_msgs/msg/Imu"),
                    Value::map({{"sample", Value::map({{"time", Value::time(1'500'000'123)}})},
                                {"reading", Value::map({{"cov", Value::array({1, 2, 3, 4, 5, 6, 7, 8, 9})},
                                                        {"f", Value::array({0.1, 0.2, 0.3})}})}}));
    const auto &imu = *static_cast<sensor_msgs::msg::Imu *>(message->data());
    EXPECT_EQ(imu.header.frame_id, "imu_link");
    EXPECT_EQ(imu.header.stamp.sec, 1);
    EXPECT_EQ(imu.header.stamp.nanosec, 500'000'123u);
    EXPECT_DOUBLE_EQ(imu.linear_acceleration.z, 0.3);
    EXPECT_DOUBLE_EQ(imu.orientation_covariance[8], 9.0);
    EXPECT_DOUBLE_EQ(imu.orientation.w, 1.0);
}

TEST(Writer, IndexedFixedArrayElementsAndNestedPaths) {
    const SpecTree sources = tree({{"sample", sim()}, {"r", tree({{"c", scalarSpec()}})}});
    const Json fields = {{"header.stamp", {{"from", "sample.time"}}},
                         {"twist.covariance[35]", {{"from", "r.c"}}},
                         {"twist.twist.angular.z", {{"from", "r.c"}}}};
    const auto writer = writerFor("geometry_msgs/msg/TwistWithCovarianceStamped", fields, sources, "fog");
    const auto message = writer.make(
        type("geometry_msgs/msg/TwistWithCovarianceStamped"),
        Value::map({{"sample", Value::map({{"time", Value::time(0)}})}, {"r", Value::map({{"c", Value::real(4.5)}})}}));
    const auto &twist = *static_cast<geometry_msgs::msg::TwistWithCovarianceStamped *>(message->data());
    EXPECT_DOUBLE_EQ(twist.twist.covariance[35], 4.5);
    EXPECT_DOUBLE_EQ(twist.twist.twist.angular.z, 4.5);
}

TEST(Writer, EnumMapConstantsAndSequences) {
    const SpecTree sources = tree({{"state", stringSpec()}, {"data", floatArray({-1})}});
    const auto writer = writerFor("std_msgs/msg/UInt8",
                                  {{"data", {{"from", "state"}, {"enum_map", {{"on", 3}, {"off", 4}}}}}}, sources);
    const auto message = writer.make(type("std_msgs/msg/UInt8"), Value::map({{"state", Value::text("off")}}));
    EXPECT_EQ(static_cast<std_msgs::msg::UInt8 *>(message->data())->data, 4);
    EXPECT_NE(errorOf([&] {
                  writer.make(type("std_msgs/msg/UInt8"), Value::map({{"state", Value::text("bad")}}));
              }).find("native state 'bad' has no enum_map entry"),
              std::string::npos);
    const auto array = writerFor("std_msgs/msg/Float32MultiArray", {{"data", {{"from", "data"}}}}, sources);
    const auto out =
        array.make(type("std_msgs/msg/Float32MultiArray"), Value::map({{"data", Value::array({1.5, 2.5, 3.5})}}));
    EXPECT_EQ(static_cast<std_msgs::msg::Float32MultiArray *>(out->data())->data,
              (std::vector<float>{1.5f, 2.5f, 3.5f}));
}

TEST(Writer, RejectsInvalidDeclarationsWithClearMessages) {
    const SpecTree sources = tree({{"sample", sim()}, {"x", scalarSpec()}, {"v", vector3Spec()}, {"n", integerSpec()}});
    const auto fail = [&](const char *message, const Json &fields, std::optional<std::string> frame = std::nullopt) {
        return errorOf([&] { writerFor(message, fields, sources, frame); });
    };
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped",
                   {{"header.stamp", {{"from", "sample.time"}}}, {"pose.position.q", {{"from", "x"}}}}, "map")
                  .find("has no field 'q'"),
              std::string::npos);
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped",
                   {{"header.stamp", {{"from", "sample.time"}}}, {"pose.orientation", {{"from", "v"}}}}, "map")
                  .find("quaternions are never assigned whole"),
              std::string::npos);
    EXPECT_NE(fail("std_msgs/msg/Float32MultiArray", {{"data[0]", {{"from", "x"}}}})
                  .find("cannot assign an element of sequence"),
              std::string::npos);
    EXPECT_NE(fail("std_msgs/msg/UInt8", {{"data", {{"constant", 300}}}}).find("300 out of range for uint8"),
              std::string::npos);
    EXPECT_NE(fail("std_msgs/msg/UInt8", {{"data", {{"from", "x"}}}}).find("cannot assign native float to ROS uint8"),
              std::string::npos);
    EXPECT_NE(fail("std_msgs/msg/UInt8", {{"data", {{"from", "x"}}}}, "frame")
                  .find("frame_id 'frame' set on unstamped UInt8"),
              std::string::npos);
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped", {{"header.stamp", {{"from", "sample.time"}}}}, "")
                  .find("requires a frame_id"),
              std::string::npos);
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped", {{"pose.position.x", {{"from", "x"}}}}, "map")
                  .find("must map header.stamp"),
              std::string::npos);
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped", {{"header.stamp", {{"from", "x"}}}}, "map")
                  .find("needs a native time source"),
              std::string::npos);
    EXPECT_NE(fail("geometry_msgs/msg/PoseStamped",
                   {{"header.stamp", {{"from", "sample.time"}}}, {"pose", {{"from", "x"}}}}, "map")
                  .find("whole geometry_msgs/Pose assignment is not supported"),
              std::string::npos);
    EXPECT_NE(
        fail("sensor_msgs/msg/Imu", {{"orientation_covariance", {{"from", "v"}}}}).find("does not fill double[9]"),
        std::string::npos);
    EXPECT_NE(fail("std_msgs/msg/Bool", {{"data", {{"from", "nope"}}}}).find("unknown native field"),
              std::string::npos);
}

TEST(Reader, ExtractsTypedArgumentsAndFilters) {
    const auto reader =
        compileReader(type("std_msgs/msg/Float32MultiArray")->members(), {{"forces_n", {{"from", "data"}}}},
                      {{"forces_n", floatArray({-1})}}, Json(), "s");
    Message message(type("std_msgs/msg/Float32MultiArray"));
    static_cast<std_msgs::msg::Float32MultiArray *>(message.data())->data = {1.f, 2.f};
    EXPECT_TRUE(reader.accepts(message.data()));
    EXPECT_EQ(reader(message.data()).at("forces_n").a, (std::vector<double>{1.0, 2.0}));

    const auto filtered =
        compileReader(type("std_msgs/msg/Bool")->members(), {{"killed", {{"from", "data"}}}},
                      {{"killed", booleanSpec()}}, Json::array({{{"field", "data"}, {"equals", true}}}), "s");
    Message flag(type("std_msgs/msg/Bool"));
    EXPECT_FALSE(filtered.accepts(flag.data()));
    static_cast<std_msgs::msg::Bool *>(flag.data())->data = true;
    EXPECT_TRUE(filtered.accepts(flag.data()));
    EXPECT_TRUE(filtered(flag.data()).at("killed").b);
}

TEST(Reader, PoseVectorsAndDimensionChecks) {
    const auto reader =
        compileReader(type("geometry_msgs/msg/PoseStamped")->members(),
                      {{"frame", {{"from", "header.frame_id"}}},
                       {"position_m", {{"from", "pose.position"}}},
                       {"w", {{"from", "pose.orientation.w"}}}},
                      {{"frame", stringSpec()}, {"position_m", vector3Spec()}, {"w", scalarSpec()}}, Json(), "s");
    Message message(type("geometry_msgs/msg/PoseStamped"));
    auto &pose = *static_cast<geometry_msgs::msg::PoseStamped *>(message.data());
    pose.header.frame_id = "map";
    pose.pose.position.x = 1;
    pose.pose.position.y = 2;
    pose.pose.position.z = 3;
    pose.pose.orientation.w = 1;
    const Value out = reader(message.data());
    EXPECT_EQ(out.at("frame").s, "map");
    EXPECT_EQ(out.at("position_m").a, (std::vector<double>{1, 2, 3}));
    EXPECT_DOUBLE_EQ(out.at("w").f, 1.0);

    const auto fixed = compileReader(type("std_msgs/msg/Float32MultiArray")->members(), {{"v", {{"from", "data"}}}},
                                     {{"v", vector3Spec()}}, Json(), "s");
    Message array(type("std_msgs/msg/Float32MultiArray"));
    static_cast<std_msgs::msg::Float32MultiArray *>(array.data())->data = {1.f, 2.f};
    EXPECT_NE(errorOf([&] { fixed(array.data()); }).find("does not match native float[3]"), std::string::npos);
}

TEST(Reader, RejectsMismatchedDeclarations) {
    const auto members = type("std_msgs/msg/Float32MultiArray")->members();
    const auto fail = [&](const Json &fields, const std::map<std::string, Spec> &arguments) {
        return errorOf([&] { compileReader(members, fields, arguments, Json(), "s"); });
    };
    EXPECT_NE(fail({{"a", {{"from", "data"}}}}, {{"b", scalarSpec()}}).find("native arguments must be exactly ['b']"),
              std::string::npos);
    EXPECT_NE(fail({{"a", {{"constant", 1}}}}, {{"a", scalarSpec()}}).find("inbound maps accept only"),
              std::string::npos);
    EXPECT_NE(fail({{"a", {{"from", "data"}}}}, {{"a", booleanSpec()}}).find("does not provide native bool"),
              std::string::npos);
    EXPECT_NE(fail({{"a", {{"from", "layout"}}}}, {{"a", scalarSpec()}}).find("cannot read whole"), std::string::npos);
    EXPECT_NE(errorOf([&] {
                  compileReader(type("std_msgs/msg/Bool")->members(), {{"a", {{"from", "data"}}}},
                                {{"a", booleanSpec()}}, Json::array({{{"field", "data"}, {"equals", "x"}}}), "s");
              }).find("cannot compare ROS bool with 'x'"),
              std::string::npos);
}

TEST(Reader, ShortSequencesAreMalformed) {
    const auto reader = compileReader(type("std_msgs/msg/Float32MultiArray")->members(), {{"x", {{"from", "data[2]"}}}},
                                      {{"x", scalarSpec()}}, Json(), "s");
    Message message(type("std_msgs/msg/Float32MultiArray"));
    static_cast<std_msgs::msg::Float32MultiArray *>(message.data())->data = {1.f, 2.f};
    EXPECT_NE(errorOf([&] { reader(message.data()); }).find("message sequence is shorter"), std::string::npos);
    static_cast<std_msgs::msg::Float32MultiArray *>(message.data())->data = {1.f, 2.f, 3.f};
    EXPECT_DOUBLE_EQ(reader(message.data()).at("x").f, 3.0);
}

TEST(Types, MissingInterfacesAreReported) {
    EXPECT_NE(errorOf([] { MessageType::get("nonexistent_pkg/msg/Nothing"); }).find("is not installed"),
              std::string::npos);
    EXPECT_NE(errorOf([] { ServiceType::get("std_srvs/srv/Nope"); }).find("is not installed"), std::string::npos);
    const auto trigger = ServiceType::get("std_srvs/srv/Trigger");
    EXPECT_EQ(resolveField(trigger->response, "success").type.base, "boolean");
    EXPECT_EQ(resolveField(type("geometry_msgs/msg/PoseStamped")->members(), "pose.position").type.base,
              "geometry_msgs/Point");
}
