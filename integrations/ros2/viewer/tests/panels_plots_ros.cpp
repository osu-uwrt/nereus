// Plots over ROS: field trees and compiled paths on real serialized messages (nav_msgs, std_msgs), header stamps,
// missing types, and the hub subscribing to a live publisher and filling a channel.
#include "../panels/plots_fields.hpp"
#include "../panels/plots_hub.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <thread>

using namespace nereus::ros_viewer::plots;

namespace {

template <class T> rclcpp::SerializedMessage serialize(const T &message) {
    rclcpp::SerializedMessage out;
    rclcpp::Serialization<T>().serialize_message(&message, &out);
    return out;
}

template <class F> bool throws(F f) {
    try {
        f();
    } catch (const std::invalid_argument &) {
        return true;
    }
    return false;
}

bool has(const std::vector<Field> &fields, const std::string &name) {
    for (const auto &f : fields)
        if (f.name == name)
            return true;
    return false;
}

void fields() {
    const auto odometry = MessageType::get("nav_msgs/msg/Odometry");
    assert(MessageType::get("nav_msgs/Odometry") == odometry); // two-part names, one cached type
    const auto root = fieldsAt(*odometry, "");
    assert(has(root, "header") && has(root, "pose") && has(root, "twist") && has(root, "child_frame_id"));
    const auto orientation = fieldsAt(*odometry, "pose.pose.orientation");
    assert(orientation.size() == 7 && orientation[0].angle && orientation[2].name == "yaw" && has(orientation, "w"));

    nav_msgs::msg::Odometry m;
    m.header.stamp.sec = 412;
    m.header.stamp.nanosec = 600000000;
    m.pose.pose.position.z = -1.25;
    m.pose.pose.orientation.z = std::sin(M_PI / 4); // yaw 90 degrees
    m.pose.pose.orientation.w = std::cos(M_PI / 4);
    m.pose.covariance[35] = 0.5;
    Message decoded(odometry);
    assert(decoded.deserialize(serialize(m)));
    assert(std::abs(FieldReader(*odometry, "pose.pose.position.z").read(decoded.data()) + 1.25) < 1e-9);
    assert(std::abs(FieldReader(*odometry, "pose.pose.orientation.yaw").read(decoded.data()) - 90) < 1e-6);
    assert(std::abs(FieldReader(*odometry, "pose.pose.orientation.roll").read(decoded.data())) < 1e-6);
    assert(FieldReader(*odometry, "pose.covariance[35]").read(decoded.data()) == .5);
    assert(std::abs(*headerStamp(*odometry, decoded.data()) - 412.6) < 1e-9);
    assert(hasHeader(*odometry));

    // Bad paths fail when compiled, with a reason.
    assert(throws([&] { FieldReader(*odometry, "pose.pose.position.q"); }));
    assert(throws([&] { FieldReader(*odometry, "pose.pose.orientation"); })); // a quaternion: pick an angle
    assert(throws([&] { FieldReader(*odometry, "pose.covariance"); }));       // an array: pick an element
    assert(throws([&] { FieldReader(*odometry, "pose.covariance[36]"); }));   // past a fixed array
    assert(throws([&] { FieldReader(*odometry, "child_frame_id"); }));        // text is not a number

    const auto paths = numericPaths(*odometry);
    const auto found = [&](const std::string &p) { return std::find(paths.begin(), paths.end(), p) != paths.end(); };
    assert(found("pose.pose.position.z") && found("pose.pose.orientation.yaw") && found("twist.twist.linear.x"));
    assert(!found("pose.pose.orientation.x")); // a quaternion's raw parts stay out of the index

    // Sequences: elements come from a received message; past its length a read is NaN.
    const auto array = MessageType::get("std_msgs/msg/Float32MultiArray");
    assert(!hasHeader(*array));
    std_msgs::msg::Float32MultiArray forces;
    forces.data = {1.5f, -2.f, 3.f};
    Message decodedForces(array);
    assert(decodedForces.deserialize(serialize(forces)));
    assert(fieldsAt(*array, "data", decodedForces.data()).size() == 3);
    assert(fieldsAt(*array, "data").empty()); // no message: no known length
    assert(FieldReader(*array, "data[1]").read(decodedForces.data()) == -2);
    assert(std::isnan(FieldReader(*array, "data[7]").read(decodedForces.data())));

    bool missing = false;
    try {
        MessageType::get("no_such_package/msg/Nothing");
    } catch (const std::runtime_error &error) {
        missing = std::string(error.what()).find("isn't installed") != std::string::npos;
    }
    assert(missing);
}

void hub() {
    Hub hub("plots_test", false);
    assert(hub.resolve("odom") == "/plots_test/odom" && hub.resolve("/tf") == "/tf");
    assert(hub.relative("/plots_test/odom") == "odom" && hub.relative("/tf") == "/tf");

    auto node = std::make_shared<rclcpp::Node>("plots_test_publisher", "/plots_test");
    auto publisher = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    Source z;
    z.topic = "odom";
    z.field = "pose.pose.position.z";
    Source bad = z;
    bad.field = "pose.pose.position.q";
    const auto channel = hub.channel(z), broken = hub.channel(bad);

    // Publish until the hub has subscribed (graph discovery) and recorded samples.
    nav_msgs::msg::Odometry m;
    const auto start = std::chrono::steady_clock::now();
    std::size_t samples = 0;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(15)) {
        m.header.stamp = node->get_clock()->now();
        m.pose.pose.position.z -= .01;
        publisher->publish(m);
        hub.maintain();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::lock_guard<std::mutex> lock(channel->mutex);
        samples = channel->samples.size();
        if (samples >= 20)
            break;
    }
    assert(samples >= 20);
    {
        std::lock_guard<std::mutex> lock(channel->mutex);
        assert(channel->stamped && channel->error.empty());
        assert(channel->samples.back().value < 0);
    }
    {
        std::lock_guard<std::mutex> lock(broken->mutex);
        assert(broken->error.find("no field") != std::string::npos);
    }
    assert(hub.rate("/plots_test/odom") > 5);
    assert(hub.latest("/plots_test/odom") != nullptr);
    assert(hub.topicType("/plots_test/odom") == "nav_msgs/msg/Odometry");

    // Figures from the UI thread land in their own channel.
    hub.pushFigure("motion.z.actual", -1.5);
    Source figure;
    figure.kind = Source::Kind::Figure;
    figure.figure = "motion.z.actual";
    const auto f = hub.channel(figure);
    std::lock_guard<std::mutex> lock(f->mutex);
    assert(f->samples.size() == 1 && f->samples.back().value == -1.5f);
}

} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    fields();
    hub();
    rclcpp::shutdown();
    std::cout << "plots ros ok\n";
    return 0;
}
