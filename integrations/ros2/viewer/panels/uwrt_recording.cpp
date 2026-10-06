// "uwrt.recording" provider: per-camera ZED SVO recording and single-image capture over ROS services.
#include "ros_runtime.hpp"
#include <set>
#include <std_srvs/srv/trigger.hpp>
#ifdef NEREUS_VIEWER_HAVE_ZED
#include <zed_msgs/srv/start_svo_rec.hpp>
#endif

namespace nereus::ros_viewer::panels {
namespace {
using Trigger = std_srvs::srv::Trigger;
#ifdef NEREUS_VIEWER_HAVE_ZED
using StartSvo = zed_msgs::srv::StartSvoRec;
#endif
// ZED SVO recording per camera (zed_node start_svo_rec / stop_svo_rec) and the picture taker's capture_image.
// Starting needs zed_msgs at build time; stopping and capturing are std_srvs/Trigger.
class UwrtRecording final : public Recording {
    // One configured camera: its UI state, service clients, and the in-flight request (epoch-guarded).
    struct Camera {
        RecordingCamera value;
#ifdef NEREUS_VIEWER_HAVE_ZED
        rclcpp::Client<StartSvo>::SharedPtr start;
#endif
        rclcpp::Client<Trigger>::SharedPtr stop;
        bool starting = false; // which client holds the pending request
        uint64_t epoch = 0;
        int64_t request = 0;
        std::string file;
        Steady::time_point since{}, startedAt{};
    };

  public:
    UwrtRecording(std::shared_ptr<RosRuntime> runtime, const YAML::Node &cfg, const Context &ctx)
        : runtime(runtime), timeout(cfg["request_timeout"].as<double>(5)) {
        auto node = runtime->node;

        // A start / stop client pair per camera.
        for (const auto &entry : cfg["cameras"]) {
            Camera camera;
            camera.value.id = entry["id"].as<std::string>();
            camera.value.label = entry["label"].as<std::string>(camera.value.id);
#ifdef NEREUS_VIEWER_HAVE_ZED
            camera.start = node->create_client<StartSvo>(expand(entry["start_service"].as<std::string>(), ctx));
#endif
            camera.stop = node->create_client<Trigger>(expand(entry["stop_service"].as<std::string>(), ctx));
            cameras.push_back(std::move(camera));
        }

        captureClient = node->create_client<Trigger>(expand(cfg["capture_service"].as<std::string>(), ctx));
        timer = node->create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    }

    // Snapshot for the UI, with each recording camera's elapsed time filled in.
    RecordingState state() override {
        std::lock_guard<std::mutex> lock(mutex);
        RecordingState value;
#ifdef NEREUS_VIEWER_HAVE_ZED
        value.svoSupported = true;
#endif
        value.captureReady = captureReady;
        value.capturing = capturing;
        value.captureMessage = captureMessage;
        const auto now = Steady::now();
        for (const auto &camera : cameras) {
            value.cameras.push_back(camera.value);
            if (camera.value.recording)
                value.cameras.back().elapsed = std::chrono::duration<double>(now - camera.startedAt).count();
        }
        return value;
    }

    // Starts SVO recording to `file` (a path on the robot) for one camera.
    void start(const std::string &id, const std::string &file) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *camera = find(id);
        if (!camera || camera->value.pending || camera->value.recording || !camera->value.startReady || file.empty())
            return;
#ifdef NEREUS_VIEWER_HAVE_ZED
        // As the RViz mapping panel: lossless, the camera's own bitrate and frame rate.
        auto request = std::make_shared<StartSvo::Request>();
        request->bitrate = 0;
        request->compression_mode = 0;
        request->target_framerate = 0;
        request->input_transcode = false;
        request->svo_filename = file;
        begin(*camera, true, "Starting recording...");
        camera->file = file;
        const auto epoch = camera->epoch;
        camera->request = camera->start
                              ->async_send_request(request,
                                                   [this, id, epoch](rclcpp::Client<StartSvo>::SharedFuture reply) {
                                                       const auto response = reply.get();
                                                       finish(id, epoch, true, response->success, response->message);
                                                   })
                              .request_id;
#else
        (void)file; // start_svo_rec needs zed_msgs at build time
#endif
    }

    void stop(const std::string &id) override {
        std::lock_guard<std::mutex> lock(mutex);
        auto *camera = find(id);
        if (!camera || camera->value.pending || !camera->value.stopReady)
            return;
        begin(*camera, false, "Stopping recording...");
        const auto epoch = camera->epoch;
        camera->request = camera->stop
                              ->async_send_request(std::make_shared<Trigger::Request>(),
                                                   [this, id, epoch](rclcpp::Client<Trigger>::SharedFuture reply) {
                                                       const auto response = reply.get();
                                                       finish(id, epoch, false, response->success, response->message);
                                                   })
                              .request_id;
    }

    // Asks the picture taker for one capture; one request at a time.
    void capture() override {
        std::lock_guard<std::mutex> lock(mutex);
        if (capturing || !captureClient->service_is_ready())
            return;
        capturing = true;
        captureSince = Steady::now();
        captureMessage = "Capturing...";
        const auto epoch = ++captureEpoch;
        captureRequest =
            captureClient
                ->async_send_request(std::make_shared<Trigger::Request>(),
                                     [this, epoch](rclcpp::Client<Trigger>::SharedFuture reply) {
                                         std::lock_guard<std::mutex> lock(mutex);
                                         if (epoch != captureEpoch)
                                             return;
                                         const auto response = reply.get();
                                         capturing = false;
                                         // The picture taker's reply lists the saved files.
                                         captureMessage = response->success ? "Captured" : "Capture failed";
                                         if (!response->message.empty())
                                             captureMessage = response->success
                                                                  ? response->message
                                                                  : captureMessage + ": " + response->message;
                                     })
                .request_id;
    }

  private:
    Camera *find(const std::string &id) {
        for (auto &camera : cameras)
            if (camera.value.id == id)
                return &camera;
        return nullptr;
    }

    // Marks a start / stop request as pending; bumping the epoch voids any earlier reply.
    static void begin(Camera &camera, bool starting, const std::string &message) {
        camera.value.pending = true;
        camera.value.message = message;
        camera.starting = starting;
        camera.since = Steady::now();
        ++camera.epoch;
    }

    // Applies a start / stop reply (runs on the executor thread; takes the lock itself).
    void finish(const std::string &id, uint64_t epoch, bool started, bool success, const std::string &message) {
        std::lock_guard<std::mutex> lock(mutex);
        auto *camera = find(id);
        if (!camera || epoch != camera->epoch)
            return;
        camera->value.pending = false;
        if (success && started) {
            camera->value.recording = true;
            camera->value.file = camera->file;
            camera->startedAt = Steady::now();
        } else if (success)
            camera->value.recording = false;
        camera->value.message =
            started ? (success ? "Recording" : "Start failed") : (success ? "Stopped" : "Stop failed");
        if (!message.empty())
            camera->value.message += ": " + message;
    }

    // 10 Hz: service readiness and request timeouts.
    void tick() {
        std::lock_guard<std::mutex> lock(mutex);
        const auto now = Steady::now();
        auto elapsed = [&](auto since) { return std::chrono::duration<double>(now - since).count(); };
        for (auto &camera : cameras) {
#ifdef NEREUS_VIEWER_HAVE_ZED
            camera.value.startReady = camera.start->service_is_ready();
#endif
            camera.value.stopReady = camera.stop->service_is_ready();
            if (camera.value.pending && elapsed(camera.since) > timeout) {
#ifdef NEREUS_VIEWER_HAVE_ZED
                if (camera.starting)
                    camera.start->remove_pending_request(camera.request);
#endif
                if (!camera.starting)
                    camera.stop->remove_pending_request(camera.request);
                ++camera.epoch;
                camera.value.pending = false;
                camera.value.message = std::string(camera.starting ? "Start" : "Stop") + " timed out; result unknown";
            }
        }

        captureReady = captureClient->service_is_ready();
        if (capturing && elapsed(captureSince) > timeout) {
            captureClient->remove_pending_request(captureRequest);
            ++captureEpoch;
            capturing = false;
            captureMessage = "Capture timed out; result unknown";
        }
    }

    std::shared_ptr<RosRuntime> runtime;
    std::mutex mutex;
    double timeout;
    std::vector<Camera> cameras;

    // Image capture request state.
    rclcpp::Client<Trigger>::SharedPtr captureClient;
    bool captureReady = false, capturing = false;
    std::string captureMessage;
    uint64_t captureEpoch = 0;
    int64_t captureRequest = 0;
    Steady::time_point captureSince{};
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace

void registerUwrtRecording(Registry &registry, const RuntimeFactory &runtime) {
    registry.providers.emplace(
        "uwrt.recording",
        ProviderFactory{Kind::Recording,
                        [](const YAML::Node &cfg) {
                            keys(cfg, {"cameras", "capture_service", "request_timeout"}, "uwrt.recording");
                            required(cfg, {"capture_service"});
                            positive(cfg, "request_timeout", 5);
                            if (!cfg["cameras"].IsSequence())
                                throw std::invalid_argument("cameras must be a sequence");
                            std::set<std::string> ids;
                            for (const auto &entry : cfg["cameras"]) {
                                keys(entry, {"id", "label", "start_service", "stop_service"}, "recording camera");
                                required(entry, {"id", "start_service", "stop_service"});
                                if (!ids.insert(entry["id"].as<std::string>()).second)
                                    throw std::invalid_argument("duplicate recording camera ID");
                            }
                        },
                        [runtime](const YAML::Node &cfg, const Context &ctx) {
                            return std::make_shared<UwrtRecording>(runtime(ctx), cfg, ctx);
                        }});
}
} // namespace nereus::ros_viewer::panels
