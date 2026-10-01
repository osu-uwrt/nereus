// CameraSink over nereus::session_cameras::SessionCameras (built when NEREUS_BUILD_SESSION_CAMERAS
// is on): compiles the camera streams and formats their messages; the scheduling, bounded workers, stale discard and
// seed reset live in SessionCameras.
#ifdef NEREUS_BRIDGE_CAMERAS
#include "camera_sink.hpp"
#include "mapping.hpp"
#include "session_port.hpp"

#include <nereus/session_cameras/session_cameras.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cmath>
#include <cstring>
#include <set>

namespace nereus::ros_bridge {
namespace sc = nereus::session_cameras;
namespace {
struct CameraStream {
    std::string id, camera, output;
    bool info{false}, right_eye{false}, jpeg{false}, depth{false}, bgr{false}, cloud{false};
    std::string encoding, jpeg_format;
    std::shared_ptr<const MessageType> type;
    Writer header;
    std::vector<sc::Output> demand_outputs;
    int stride{1}, every{1}; // point_cloud: pixel step; publish every Nth capture (stream rate = sensor rate / N)
};
// PCL PointXYZRGB layout, as the ZED driver publishes (point_step 32).
struct CloudPoint {
    float x, y, z, padding;
    std::uint8_t b, g, r, alpha;
    std::uint32_t reserved[3];
};
static_assert(sizeof(CloudPoint) == 32 && offsetof(CloudPoint, b) == 16);

SpecTree infoSources() {
    SpecTree sample(std::map<std::string, SpecTree>{{"time", timeSpec()}});
    SpecTree info(std::map<std::string, SpecTree>{
        {"width", integerSpec()}, {"height", integerSpec()}, {"k", floatArray({9})}, {"p", floatArray({12})}});
    return SpecTree(std::map<std::string, SpecTree>{{"sample", sample}, {"info", info}});
}
SpecTree sampleSources() {
    SpecTree sample(std::map<std::string, SpecTree>{{"time", timeSpec()}});
    return SpecTree(std::map<std::string, SpecTree>{{"sample", sample}});
}

class SessionCameraSink final : public CameraSink {
  public:
    SessionCameraSink(const session::ResolvedScenario &resolved, const std::vector<std::string> &camera_ids,
                      SessionPort &session, const CameraSinkOptions &options)
        : session_(session), resolved_(resolved) {
        std::map<std::string, Json> sensors;
        for (const auto &sensor : resolved.robot.at("sensors"))
            if (std::find(camera_ids.begin(), camera_ids.end(), sensor.at("id").get<std::string>()) != camera_ids.end())
                sensors[sensor.at("id")] = sensor;
        if (sensors.size() != camera_ids.size())
            throw MappingError("camera provider must select unique enabled robot cameras");
        for (const auto &[id, sensor] : sensors)
            if (sensor.at("type") != "stereo_camera" || !sensor.value("enabled", true))
                throw MappingError("camera provider must select unique enabled robot cameras");
        std::map<std::string, int> quality;
        const Json names = resolved.bridge.value("frame_names", Json::object());
        for (const auto &stream : resolved.bridge.at("streams")) {
            const std::string endpoint = stream.at("native");
            if (endpoint.rfind("sensor:", 0) != 0)
                continue;
            const std::string rest = endpoint.substr(7);
            const auto dot = rest.find('.');
            const std::string camera = rest.substr(0, dot);
            const std::string output = dot == std::string::npos ? "" : rest.substr(dot + 1);
            const auto found = sensors.find(camera);
            if (found == sensors.end())
                continue;
            const Json &config = found->second;
            const std::string id = stream.at("id");
            const std::string where = "streams/" + id;
            bool listed = false;
            for (const auto &item : config.at("parameters").at("outputs"))
                listed = listed || item == output;
            if (!listed)
                throw MappingError("camera " + repr(camera) + " has no configured output " + repr(output));
            const double rate = stream.at("rate_hz").get<double>();
            // Point clouds may be published at an integer fraction of the camera rate.
            const double ratio = rate > 0 ? 1e9 / (rate * config.at("period_ns").get<double>()) : 0;
            const int every = static_cast<int>(std::lround(ratio));
            if (output == "point_cloud" ? (every < 1 || std::fabs(ratio - every) > 1e-6) : std::fabs(ratio - 1) > 1e-6)
                throw MappingError("camera stream " + repr(id) +
                                   (output == "point_cloud" ? " rate must be the sensor rate divided by an integer"
                                                            : " rate differs from its sensor"));
            const bool right = output.size() >= 5 && output.compare(output.size() - 5, 5, "right") == 0;
            const std::string frame = right ? config.at("parameters").at("right_frame").get<std::string>()
                                            : config.at("frame").get<std::string>();
            if (names.contains(frame) && stream.at("frame_id") != names[frame])
                throw MappingError("camera stream " + repr(id) + " differs from frame_names");
            CameraStream item;
            item.id = id;
            item.camera = camera;
            item.output = output;
            item.right_eye = right;
            item.type = MessageType::get(stream.at("message_type"));
            const std::string frame_id = stream.value("frame_id", "");
            if (output.rfind("camera_info", 0) == 0)
                compileInfo(item, stream, where, frame_id);
            else if (output == "point_cloud")
                compileCloud(item, stream, where, frame_id, every, config);
            else
                compileImage(item, stream, where, frame_id, camera, quality);
            streams_[camera].push_back(item);
            stream_ids_.push_back(id);
        }
        for (const auto &[id, sensor] : sensors) {
            if (sensor.value("latency_ns", 0) != 0)
                throw MappingError("camera " + repr(id) + ": nonzero delivery latency is not implemented");
            if (sensor.value("capacity", 1) < 1 || (sensor.value("overflow", "drop_oldest") != "fail" &&
                                                    sensor.value("overflow", "drop_oldest") != "drop_oldest"))
                throw MappingError("camera " + repr(id) + ": invalid pending queue policy");
        }
        sc::Options camera_options;
        camera_options.sensor_ids = camera_ids;
        camera_options.always = options.always;
        camera_options.supersample = options.supersample;
        camera_options.jpeg_quality = quality;
        cameras_ = std::make_unique<sc::SessionCameras>(resolved, camera_options);
        quality_ = quality;
        // Payload mesh for dynamic content comes from the projectiles stream options, when declared.
        for (const auto &stream : resolved.bridge.at("streams"))
            if (stream.at("native") == "state:payloads" && stream.contains("options") &&
                stream["options"].contains("mesh_asset"))
                payload_asset_ = stream["options"]["mesh_asset"].get<std::string>();
        if (payload_asset_)
            payload_mesh_ = cameras_->scene().mesh("robot", *payload_asset_);
    }
    ~SessionCameraSink() override {
        close();
    }

    std::vector<std::string> streamIds() const override {
        return stream_ids_;
    }

    void start(std::function<void(EncodedImage)> publish) override {
        publish_ = std::move(publish);
        cameras_->start([this](sc::Products &&products) { deliver(products); });
    }

    void acquire(const simulation::Snapshot &snapshot, std::int64_t ros_ns) override {
        const spatial::Pose root{snapshot.body.position, snapshot.body.orientation};
        std::vector<rendering::Instance> dynamic;
        if (anyDemand())
            dynamic = dynamicInstances();
        std::map<std::string, bool> latched; // task indicator latch states for LED visuals
        if (!cameras_->scene().indicatorVisuals().empty())
            for (const auto &item : session_.indicators())
                latched[item.at("region").get<std::string>()] = item.at("latched").get<bool>();
        cameras_->request(snapshot.elapsed.count(), ros_ns, root, dynamic, {}, latched);
    }

    void setDemand(const std::string &stream, bool wanted) override {
        for (const auto &[camera, list] : streams_)
            for (const auto &item : list)
                if (item.id == stream && !item.demand_outputs.empty()) {
                    for (const auto output : item.demand_outputs)
                        cameras_->setDemand(camera, output, wanted, "ros:" + stream);
                    if (wanted)
                        demanded_.insert(stream);
                    else
                        demanded_.erase(stream);
                }
    }

    void invalidate(std::optional<std::uint64_t> seed) override {
        cameras_->invalidate(seed);
    }
    void close() override {
        if (cameras_)
            cameras_->close();
    }

    Json describe() const override {
        Json pending = Json::object();
        for (const auto &sensor : resolved_.robot.at("sensors"))
            if (streams_.count(sensor.at("id").get<std::string>()))
                pending[sensor.at("id").get<std::string>()] = {{"capacity", sensor.value("capacity", 1)},
                                                               {"overflow", sensor.value("overflow", "drop_oldest")}};
        Json jpeg = Json::object();
        for (const auto &[camera, list] : streams_)
            jpeg[camera] = quality_.count(camera) ? Json(quality_.at(camera)) : Json();
        return {{"capture", cameras_->describe()},
                {"worker",
                 {{"count", streams_.size()},
                  {"in_flight_capacity", streams_.size()},
                  {"policy", "one per selected camera; shared GL serialized, on-demand outputs"},
                  {"pending", pending},
                  {"jpeg_quality", jpeg}}}};
    }

    Json stats() const override {
        Json out = Json::object();
        for (const auto &[camera, item] : cameras_->stats())
            out[camera] = {{"requested", item.requested},
                           {"skipped_no_demand", item.skipped_no_demand},
                           {"captured", item.captured},
                           {"published", item.delivered},
                           {"dropped_pending", item.dropped_pending},
                           {"discarded_stale", item.discarded_stale},
                           {"capture_wall_ns", item.capture_wall_ns},
                           {"render_ns", item.render_ns},
                           {"process_ns", item.process_ns}};
        return out;
    }

  private:
    bool anyDemand() const {
        return !demanded_.empty() || always_;
    }

    void compileInfo(CameraStream &item, const Json &stream, const std::string &where, const std::string &frame_id) {
        item.info = true;
        if (stream.at("direction") != "publish" || (item.output != "camera_info" && item.output != "camera_info_right"))
            throw MappingError(where + ": camera_info requires a camera metadata publication");
        if (stream.at("message_type") != "sensor_msgs/msg/CameraInfo" || stream.contains("image"))
            throw MappingError(where + ": camera_info requires sensor_msgs/msg/CameraInfo");
        const Json &fields = stream.at("fields");
        if (!fields.contains("header.stamp") || fields["header.stamp"] != Json{{"from", "sample.time"}})
            throw MappingError(where + ": CameraInfo header.stamp must use sample.time");
        if (fields.contains("header.frame_id"))
            throw MappingError(where + ": use frame_id for CameraInfo frame names");
        item.header = compileWriter(item.type->members(), fields, infoSources(), frame_id, where);
    }

    void compileImage(CameraStream &item, const Json &stream, const std::string &where, const std::string &frame_id,
                      const std::string &camera, std::map<std::string, int> &quality) {
        const std::string &output = item.output;
        if (output != "rgb_left" && output != "rgb_right" && output != "depth_left")
            throw MappingError(where + ": image requires an RGB or depth camera output");
        if (stream.at("direction") != "publish")
            throw MappingError(where + ": camera images must be published");
        if (!stream.contains("image"))
            throw MappingError(where + ": image streams need an image block");
        const Json &fields = stream.at("fields");
        if (fields != Json{{"header.stamp", {{"from", "sample.time"}}}})
            throw MappingError(where + ": image fields must map header.stamp from sample.time; frame_id and "
                                       "image settings supply the other fields");
        const Json &options = stream.at("image");
        item.depth = output == "depth_left";
        const std::string type = stream.at("message_type");
        if (options.contains("compression")) {
            if (item.depth || options["compression"] != "jpeg" ||
                (options.at("source_encoding") != "rgb8" && options.at("source_encoding") != "bgr8") ||
                options.at("compressed_encoding") != "bgr8")
                throw MappingError(where + ": native JPEG uses RGB input and BGR compression");
            if (type != "sensor_msgs/msg/CompressedImage")
                throw MappingError(where + ": JPEG requires sensor_msgs/msg/CompressedImage");
            const Json &q = options.at("quality");
            if (!q.is_number_integer() || q.get<int>() < 1 || q.get<int>() > 100)
                throw MappingError(where + ": JPEG quality must be an integer in [1, 100]");
            item.jpeg = true;
            item.encoding = options.at("source_encoding");
            item.jpeg_format = item.encoding + "; jpeg compressed bgr8";
            const int value = q.get<int>();
            const auto have = quality.find(camera);
            if (have != quality.end() && have->second != value)
                throw MappingError("camera " + repr(camera) + " streams require conflicting JPEG qualities");
            quality[camera] = value;
        } else {
            item.encoding = options.at("encoding");
            const bool allowed =
                item.depth ? item.encoding == "32FC1" : (item.encoding == "rgb8" || item.encoding == "bgr8");
            if (!allowed || type != "sensor_msgs/msg/Image")
                throw MappingError(where + ": " + output + " needs sensor_msgs/msg/Image with " +
                                   (item.depth ? "['32FC1']" : "['bgr8', 'rgb8']"));
            item.bgr = item.encoding == "bgr8";
        }
        item.header = compileWriter(item.type->members(), fields, sampleSources(), frame_id, where);
        item.demand_outputs = {output == "rgb_left"    ? sc::Output::RgbLeft
                               : output == "rgb_right" ? sc::Output::RgbRight
                                                       : sc::Output::DepthLeft};
    }

    // Coloured cloud from the left depth and RGB of one capture, decimated by `stride`, organized
    // (height x width grid, NaN where depth is invalid) in the stream's (optical) frame.
    void compileCloud(CameraStream &item, const Json &stream, const std::string &where, const std::string &frame_id,
                      int every, const Json &sensor) {
        if (stream.at("direction") != "publish" || stream.at("message_type") != "sensor_msgs/msg/PointCloud2")
            throw MappingError(where + ": point_cloud requires a sensor_msgs/msg/PointCloud2 publication");
        bool rgb = false, depth = false;
        for (const auto &output : sensor.at("parameters").at("outputs"))
            rgb = rgb || output == "rgb_left", depth = depth || output == "depth_left";
        if (!rgb || !depth)
            throw MappingError(where + ": point_cloud needs the camera's rgb_left and depth_left outputs");
        const Json &fields = stream.at("fields");
        if (fields != Json{{"header.stamp", {{"from", "sample.time"}}}})
            throw MappingError(where + ": point_cloud fields must map header.stamp from sample.time");
        const Json options = stream.value("point_cloud", Json::object());
        for (const auto &[key, value] : options.items())
            if (key != "stride")
                throw MappingError(where + ": unknown point_cloud option " + repr(key));
        const Json stride = options.value("stride", Json(1));
        if (!stride.is_number_integer() || stride.get<int>() < 1)
            throw MappingError(where + ": point_cloud stride must be a positive integer");
        item.cloud = true;
        item.stride = stride.get<int>();
        item.every = every;
        item.header = compileWriter(item.type->members(), fields, sampleSources(), frame_id, where);
        item.demand_outputs = {sc::Output::RgbLeft, sc::Output::DepthLeft};
    }

    static void fillCloud(const CameraStream &item, const cameras::Frame &frame, const sc::CameraInfo &info,
                          void *message) {
        const int width = frame.width, height = frame.height;
        if (frame.depth.size() != static_cast<std::size_t>(width) * height ||
            frame.rgb.size() != static_cast<std::size_t>(width) * height * 3)
            throw MappingError("point cloud needs matching depth and RGB frames");
        const double fx = info.k[0], cx = info.k[2], fy = info.k[4], cy = info.k[5];
        const int columns = (width - 1) / item.stride + 1, rows = (height - 1) / item.stride + 1;
        auto &cloud = *static_cast<sensor_msgs::msg::PointCloud2 *>(message);
        cloud.height = static_cast<std::uint32_t>(rows);
        cloud.width = static_cast<std::uint32_t>(columns);
        cloud.is_bigendian = false;
        cloud.is_dense = false;
        cloud.point_step = sizeof(CloudPoint);
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.fields.clear();
        for (const auto &[name, offset] :
             {std::pair<const char *, std::uint32_t>{"x", 0}, {"y", 4}, {"z", 8}, {"rgb", 16}}) {
            sensor_msgs::msg::PointField field;
            field.name = name;
            field.offset = offset;
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;
            cloud.fields.push_back(field);
        }
        cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0);
        auto *points = reinterpret_cast<CloudPoint *>(cloud.data.data());
        for (int y = 0; y < rows; ++y)
            for (int x = 0; x < columns; ++x) {
                const int u = x * item.stride, v = y * item.stride;
                const std::size_t pixel = static_cast<std::size_t>(v) * width + u;
                const float d = frame.depth[pixel];
                auto &p = points[static_cast<std::size_t>(y) * columns + x];
                p.x = static_cast<float>((u - cx) * d / fx);
                p.y = static_cast<float>((v - cy) * d / fy);
                p.z = d;
                p.r = frame.rgb[3 * pixel];
                p.g = frame.rgb[3 * pixel + 1];
                p.b = frame.rgb[3 * pixel + 2];
                p.alpha = 255;
            }
    }

    std::vector<rendering::Instance> dynamicInstances() {
        std::vector<rendering::Instance> out;
        const auto &visuals = cameras_->scene().propVisuals();
        for (const auto &prop : session_.propVisuals())
            for (const auto &visual : visuals)
                if (visual.task == prop.task && visual.prop == prop.id && visual.mesh) {
                    rendering::Instance instance;
                    instance.mesh = visual.mesh;
                    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                    pose.topLeftCorner<3, 3>() = prop.orientation.toRotationMatrix();
                    pose.topRightCorner<3, 1>() = prop.position;
                    instance.transform = pose.cast<float>();
                    out.push_back(std::move(instance));
                }
        if (payload_mesh_)
            for (const auto &payload : session_.payloadVisuals()) {
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                pose.topLeftCorner<3, 3>() =
                    payload.orientation.toRotationMatrix() *
                    Eigen::DiagonalMatrix<double, 3>(payload.length_m, 2 * payload.radius_m, 2 * payload.radius_m);
                pose.topRightCorner<3, 1>() = payload.position;
                rendering::Instance instance;
                instance.mesh = payload_mesh_;
                instance.transform = pose.cast<float>();
                out.push_back(std::move(instance));
            }
        return out;
    }

    static void fillImage(const CameraStream &item, const cameras::Frame &frame, void *message) {
        if (item.jpeg) {
            if (frame.jpeg.empty())
                throw MappingError("camera did not produce the configured JPEG");
            auto &image = *static_cast<sensor_msgs::msg::CompressedImage *>(message);
            image.format = item.jpeg_format;
            image.data.assign(frame.jpeg.begin(), frame.jpeg.end());
            return;
        }
        const int width = frame.width, height = frame.height;
        if (width <= 0 || height <= 0)
            throw MappingError("camera frame dimensions must be positive");
        auto &image = *static_cast<sensor_msgs::msg::Image *>(message);
        image.width = static_cast<std::uint32_t>(width);
        image.height = static_cast<std::uint32_t>(height);
        image.encoding = item.encoding;
        image.is_bigendian = 0;
        if (item.depth) {
            if (frame.depth.size() != static_cast<std::size_t>(width) * height)
                throw MappingError("depth frame must be native float32 with shape (height, width)");
            image.step = static_cast<std::uint32_t>(width * 4);
            image.data.resize(frame.depth.size() * sizeof(float));
            std::memcpy(image.data.data(), frame.depth.data(), image.data.size());
        } else {
            if (frame.rgb.size() != static_cast<std::size_t>(width) * height * 3)
                throw MappingError("RGB frame must be uint8 with shape (height, width, 3)");
            image.step = static_cast<std::uint32_t>(width * 3);
            image.data.assign(frame.rgb.begin(), frame.rgb.end());
            if (item.bgr)
                for (std::size_t k = 0; k + 2 < image.data.size(); k += 3)
                    std::swap(image.data[k], image.data[k + 2]);
        }
    }

    void deliver(const sc::Products &products) {
        const auto found = streams_.find(products.camera);
        if (found == streams_.end())
            return;
        for (const auto &item : found->second) {
            auto message = std::make_shared<Message>(item.type);
            if (item.info) {
                const auto &info = item.right_eye ? products.right_info : products.left_info;
                std::vector<double> k(info.k.begin(), info.k.end()), p(info.p.begin(), info.p.end());
                item.header.apply(message->data(),
                                  Value::map({{"sample", Value::map({{"time", Value::time(products.ros_stamp_ns)}})},
                                              {"info", Value::map({{"width", Value::integer(info.width)},
                                                                   {"height", Value::integer(info.height)},
                                                                   {"k", Value::array(k)},
                                                                   {"p", Value::array(p)}})}}));
            } else {
                const auto &frame = item.right_eye ? products.right : products.left;
                if (!frame)
                    continue; // output not produced (no demand)
                const bool produced = item.cloud   ? !frame->depth.empty() && !frame->rgb.empty()
                                      : item.jpeg  ? !frame->jpeg.empty()
                                      : item.depth ? !frame->depth.empty()
                                                   : !frame->rgb.empty();
                if (!produced)
                    continue;
                if (item.cloud && cloud_count_[item.id]++ % item.every != 0)
                    continue;
                item.header.apply(message->data(),
                                  Value::map({{"sample", Value::map({{"time", Value::time(products.ros_stamp_ns)}})}}));
                if (item.cloud)
                    fillCloud(item, *frame, products.left_info, message->data());
                else
                    fillImage(item, *frame, message->data());
            }
            publish_({item.id, message});
        }
    }

    SessionPort &session_;
    const session::ResolvedScenario &resolved_;
    std::map<std::string, std::vector<CameraStream>> streams_;
    std::vector<std::string> stream_ids_;
    std::map<std::string, int> quality_;
    std::unique_ptr<sc::SessionCameras> cameras_;
    std::function<void(EncodedImage)> publish_;
    std::optional<std::string> payload_asset_;
    std::shared_ptr<const rendering::MeshAsset> payload_mesh_;
    std::set<std::string> demanded_;
    std::map<std::string, std::uint64_t> cloud_count_; // deliveries are serialized across cameras
    bool always_{false};

  public:
    void setAlways(bool value) {
        always_ = value;
        cameras_->setAlways(value);
    }
};
} // namespace

std::unique_ptr<CameraSink> createCameraSink(const session::ResolvedScenario &scenario,
                                             const std::vector<std::string> &camera_ids, SessionPort &session,
                                             const CameraSinkOptions &options) {
    auto sink = std::make_unique<SessionCameraSink>(scenario, camera_ids, session, options);
    sink->setAlways(options.always);
    return sink;
}
} // namespace nereus::ros_bridge
#endif
