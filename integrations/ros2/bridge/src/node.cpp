#include "node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <robot_localization/srv/set_pose.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

namespace robotics::ros_bridge {
namespace {
using Clock = std::chrono::steady_clock;
using Post = std::function<void(std::function<void()>)>;
using Handler = std::function<void(const void *, void *)>;

rclcpp::QoS qosFrom(const Json &config) {
    rclcpp::QoS qos(rclcpp::KeepLast(config.at("depth").get<std::size_t>()));
    const std::string reliability = config.at("reliability"), durability = config.at("durability");
    if (reliability == "reliable")
        qos.reliable();
    else
        qos.best_effort();
    if (durability == "transient_local")
        qos.transient_local();
    else
        qos.durability_volatile();
    return qos;
}

builtin_interfaces::msg::Time stampOf(std::int64_t ns) {
    std::int64_t sec = ns / 1000000000, rest = ns % 1000000000;
    if (rest < 0) {
        rest += 1000000000;
        --sec;
    }
    builtin_interfaces::msg::Time time;
    time.sec = static_cast<std::int32_t>(sec);
    time.nanosec = static_cast<std::uint32_t>(rest);
    return time;
}

geometry_msgs::msg::TransformStamped transformMessage(const Transform &item) {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stampOf(item.stamp_ns);
    message.header.frame_id = item.parent;
    message.child_frame_id = item.child;
    message.transform.translation.x = item.translation.x();
    message.transform.translation.y = item.translation.y();
    message.transform.translation.z = item.translation.z();
    message.transform.rotation.w = item.orientation.w();
    message.transform.rotation.x = item.orientation.x();
    message.transform.rotation.y = item.orientation.y();
    message.transform.rotation.z = item.orientation.z();
    return message;
}

template <class Srv>
std::shared_ptr<void> createService(rclcpp::Node &node, const std::string &name, Handler handler, Post post) {
    return node.create_service<Srv>(
        name, [handler, post](std::shared_ptr<rclcpp::Service<Srv>> service, std::shared_ptr<rmw_request_id_t> header,
                              std::shared_ptr<typename Srv::Request> request) {
            post([handler, service, header, request] {
                auto response = std::make_shared<typename Srv::Response>();
                handler(request.get(), response.get());
                service->send_response(*header, *response);
            });
        });
}
using ServiceCreator = std::function<std::shared_ptr<void>(rclcpp::Node &, const std::string &, Handler, Post)>;
const std::map<std::string, ServiceCreator> &serviceCreators() {
    static const std::map<std::string, ServiceCreator> table = {
        {"std_srvs/srv/Trigger", createService<std_srvs::srv::Trigger>},
        {"std_srvs/srv/SetBool", createService<std_srvs::srv::SetBool>},
        {"robot_localization/srv/SetPose", createService<robot_localization::srv::SetPose>}};
    return table;
}
} // namespace

struct BridgeNode::Impl {
    std::mutex inbox_mutex;
    std::deque<std::function<void()>> inbox;
    std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers;
    std::map<std::string, std::shared_ptr<rclcpp::SerializationBase>> serializers;
    std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subscriptions;
    std::vector<std::shared_ptr<void>> services;
    std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> clock;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
    rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr alignment_client;
    rclcpp::SerializedMessage buffer_message; // reused by the stepping thread
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread executor_thread;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_handle;
    CameraSink *cameras{nullptr};

    void post(std::function<void()> work) {
        std::lock_guard<std::mutex> lock(inbox_mutex);
        inbox.push_back(std::move(work));
    }
    void drainInbox() {
        std::deque<std::function<void()>> work;
        {
            std::lock_guard<std::mutex> lock(inbox_mutex);
            if (inbox.empty())
                return;
            work.swap(inbox);
        }
        for (auto &item : work)
            item();
    }
};

BridgeNode::BridgeNode(BridgeCore &core, CameraSink *cameras)
    : impl_(std::make_unique<Impl>()), core_(core) {
    impl_->cameras = cameras;
    const Json &config = core.config();
    node_ = std::make_shared<rclcpp::Node>(config.value("node_name", "robotics_platform_bridge"),
                                           config.value("namespace", "/"));
    impl_->buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    impl_->listener = std::make_shared<tf2_ros::TransformListener>(*impl_->buffer, node_, false);
    core.setLookup([buffer = impl_->buffer](const std::string &target,
                                            const std::string &source) -> std::optional<spatial::Pose> {
        try {
            const auto transform = buffer->lookupTransform(target, source, tf2::TimePointZero);
            const auto &t = transform.transform.translation;
            const auto &r = transform.transform.rotation;
            return spatial::Pose{Eigen::Vector3d(t.x, t.y, t.z),
                                 Eigen::Quaterniond(r.w, r.x, r.y, r.z).normalized()};
        } catch (const tf2::TransformException &) {
            return std::nullopt;
        }
    });

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.dynamic_typing = true;
    descriptor.description = "Simulation speed relative to wall time; 0 pauses stepping and /clock.";
    node_->declare_parameter("real_time_factor", core.realTimeFactor(), descriptor);
    impl_->parameter_handle = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto &parameter : parameters) {
                if (parameter.get_name() != "real_time_factor")
                    continue;
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
                    result.successful = false;
                    result.reason = "real_time_factor must be a number";
                    return result;
                }
                const double value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                                         ? parameter.as_double()
                                         : static_cast<double>(parameter.as_int());
                if (const auto problem = core_.setRealTimeFactor(value)) {
                    result.successful = false;
                    result.reason = *problem;
                    return result;
                }
            }
            return result;
        });

    const Json &clock = config.at("clock");
    impl_->clock = node_->create_publisher<rosgraph_msgs::msg::Clock>(clock.at("topic"), qosFrom(clock.at("qos")));
    if (config.value("tf", Json::object()).contains("publish") && !config["tf"]["publish"].empty())
        impl_->broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    if (!core.staticTransforms().empty()) {
        impl_->static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        std::vector<geometry_msgs::msg::TransformStamped> messages;
        for (const auto &item : core.staticTransforms())
            messages.push_back(transformMessage(item));
        impl_->static_broadcaster->sendTransform(messages);
    }

    Impl *impl = impl_.get();
    for (const auto &stream : config.at("streams")) {
        const std::string id = stream.at("id"), topic = stream.at("topic");
        const auto type = core.streamTypes().at(id);
        const rclcpp::QoS qos = qosFrom(stream.at("qos"));
        impl->serializers[id] = std::make_shared<rclcpp::SerializationBase>(type->typeSupport());
        if (stream.at("direction") == "publish") {
            impl->publishers[id] = node_->create_generic_publisher(topic, type->name(), qos);
        } else {
            const auto serializer = impl->serializers[id];
            impl->subscriptions.push_back(node_->create_generic_subscription(
                topic, type->name(), qos, [this, impl, id, type, serializer](std::shared_ptr<rclcpp::SerializedMessage> raw) {
                    impl->post([this, id, type, serializer, raw] {
                        Message message(type);
                        serializer->deserialize_message(raw.get(), message.data());
                        send(core_.receive(id, message.data()));
                    });
                }));
        }
    }
    for (const auto &service : config.value("services", Json::array())) {
        const std::string id = service.at("id"), name = service.at("service");
        const auto &entry = core.services().at(id);
        const auto creator = serviceCreators().find(entry.type->name);
        if (creator == serviceCreators().end())
            throw BridgeError("service type '" + entry.type->name + "' is not supported by this bridge");
        impl->services.push_back(creator->second(
            *node_, name, [this, id](const void *request, void *response) { core_.call(id, request, response); },
            [impl](std::function<void()> work) { impl->post(std::move(work)); }));
    }
    if (core.hasAlignment())
        impl->alignment_client = node_->create_client<robot_localization::srv::SetPose>(
            core.alignmentConfig().at("client").get<std::string>());
}

BridgeNode::~BridgeNode() {
    stopExecutor();
}

void BridgeNode::startExecutor() {
    impl_->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    impl_->executor->add_node(node_);
    impl_->executor_thread = std::thread([this] { impl_->executor->spin(); });
}

void BridgeNode::stopExecutor() {
    if (impl_->executor) {
        impl_->executor->cancel();
        if (impl_->executor_thread.joinable())
            impl_->executor_thread.join();
        impl_->executor.reset();
    }
}

void BridgeNode::send(const std::vector<Publication> &publications) {
    for (const auto &item : publications) {
        impl_->serializers.at(item.stream)->serialize_message(item.message->data(), &impl_->buffer_message);
        impl_->publishers.at(item.stream)->publish(impl_->buffer_message);
    }
}

void BridgeNode::publishClock(std::int64_t ns) {
    rosgraph_msgs::msg::Clock message;
    message.clock = stampOf(ns);
    impl_->clock->publish(message);
}

void BridgeNode::broadcast(const std::vector<Transform> &transforms) {
    if (transforms.empty() || !impl_->broadcaster)
        return;
    std::vector<geometry_msgs::msg::TransformStamped> messages;
    for (const auto &item : transforms)
        messages.push_back(transformMessage(item));
    impl_->broadcaster->sendTransform(messages);
}

void BridgeNode::tick() {
    impl_->drainInbox();
    StepOutput out = core_.step();
    for (const auto stamp : out.clocks) // clock strictly before data carrying that time
        publishClock(stamp);
    send(out.publications);
    broadcast(out.transforms);
    if (impl_->cameras != nullptr) {
        const auto snapshot = core_.session().observe();
        impl_->cameras->acquire(snapshot, core_.rosNs(snapshot.elapsed.count()));
    }
    if (impl_->alignment_client && impl_->alignment_client->service_is_ready()) {
        if (const auto alignment = core_.pendingAlignment()) {
            auto request = std::make_shared<robot_localization::srv::SetPose::Request>();
            auto &stamped = request->pose;
            stamped.header.frame_id = alignment->frame;
            stamped.header.stamp = stampOf(alignment->stamp_ns);
            auto &target = stamped.pose.pose;
            target.position.x = alignment->position.x();
            target.position.y = alignment->position.y();
            target.position.z = alignment->position.z();
            target.orientation.w = alignment->orientation.w();
            target.orientation.x = alignment->orientation.x();
            target.orientation.y = alignment->orientation.y();
            target.orientation.z = alignment->orientation.z();
            for (std::size_t k = 0; k < 6; ++k)
                stamped.pose.covariance[k * 7] = alignment->covariance_diagonal;
            Impl *impl = impl_.get();
            const std::string trigger = alignment->trigger;
            impl_->alignment_client->async_send_request(
                request, [this, impl, trigger](rclcpp::Client<robot_localization::srv::SetPose>::SharedFuture future) {
                    bool ok = true;
                    try {
                        future.get();
                    } catch (const std::exception &error) {
                        RCLCPP_ERROR(node_->get_logger(), "estimator alignment failed (%s): %s", trigger.c_str(), error.what());
                        ok = false;
                    }
                    impl->post([this, trigger, ok] {
                        Counters::bump(ok ? core_.counters().alignments_acknowledged : core_.counters().alignments_failed,
                                       trigger);
                    });
                });
        }
    }
}

std::int64_t BridgeNode::run(std::optional<std::int64_t> duration_ns, int max_catchup_ticks) {
    const double step_s = static_cast<double>(core_.timestepNs()) / 1e9;
    publishClock(core_.clockNs());
    send(core_.startupPublications());
    if (impl_->cameras != nullptr) {
        impl_->cameras->start([this](EncodedImage image) {
            rclcpp::SerializedMessage buffer;
            impl_->serializers.at(image.stream)->serialize_message(image.message->data(), &buffer);
            impl_->publishers.at(image.stream)->publish(buffer);
        });
    }
    std::map<std::string, bool> demand; // camera stream -> has subscribers
    auto last_demand = Clock::now() - std::chrono::seconds(1);
    const auto pollDemand = [&](Clock::time_point now) {
        if (impl_->cameras == nullptr || now - last_demand < std::chrono::milliseconds(500))
            return;
        last_demand = now;
        for (const auto &stream : impl_->cameras->streamIds()) {
            const bool wanted = impl_->publishers.at(stream)->get_subscription_count() > 0;
            const auto known = demand.find(stream);
            if (known == demand.end() || known->second != wanted) {
                demand[stream] = wanted;
                impl_->cameras->setDemand(stream, wanted);
            }
        }
    };
    std::int64_t ticks = 0;
    const auto run_start = Clock::now();
    double owed = 0.0;
    auto previous = Clock::now(), last_refresh = previous;
    constexpr double kPausedRefreshS = 0.02; // viewer state refresh while paused (original 50 Hz timer)
    while (rclcpp::ok() && !stop_) {
        impl_->drainInbox();
        send(core_.flush()); // operator events, also while paused
        const auto now = Clock::now();
        pollDemand(now);
        const double rtf = core_.realTimeFactor();
        if (rtf <= 0) {
            owed = 0.0; // paused: no stepping and no /clock; viewers still get fresh state
            if (std::chrono::duration<double>(now - last_refresh).count() >= kPausedRefreshS) {
                send(core_.refresh());
                last_refresh = now;
            }
            previous = now;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        owed += std::chrono::duration<double>(now - previous).count() * rtf;
        previous = now;
        int steps = 0;
        while (owed >= step_s && steps < max_catchup_ticks) {
            const auto started = Clock::now();
            tick();
            const auto cost = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - started).count();
            performance_.tick_ns_total += cost;
            performance_.tick_ns_max = std::max<std::int64_t>(performance_.tick_ns_max, cost);
            ++performance_.ticks;
            ++ticks;
            owed -= step_s;
            ++steps;
            if (duration_ns && ticks * core_.timestepNs() >= *duration_ns) {
                performance_.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - run_start).count();
                performance_.sim_ns = ticks * core_.timestepNs();
                return ticks;
            }
        }
        if (steps == max_catchup_ticks && owed >= step_s) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "falling behind wall time; dropping %.4f s backlog", owed);
            owed = 0.0;
        }
        const double remaining = (step_s - owed) / std::max(rtf, 1e-9);
        if (remaining > 2e-4)
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining - 1e-4));
    }
    performance_.wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - run_start).count();
    performance_.sim_ns = ticks * core_.timestepNs();
    return ticks;
}

} // namespace robotics::ros_bridge
