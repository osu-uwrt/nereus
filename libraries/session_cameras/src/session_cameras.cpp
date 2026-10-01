#include <nereus/session_cameras/session_cameras.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace nereus::session_cameras {
namespace {
using Json = session::Json;
constexpr double kNearPlaneM = 0.05; // camera clipping planes
constexpr double kFarPlaneM = 100.0;
constexpr int kDefaultJpegQuality = 93;
const char *const kEyes[2] = {"left", "right"};

std::int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
double ms(std::int64_t ns) {
    return static_cast<double>(ns) / 1e6;
}

// SHA-256 (FIPS 180-4); only used for seed derivation.
struct Sha256 {
    std::uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                          0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    static std::uint32_t rotr(std::uint32_t x, int n) {
        return (x >> n) | (x << (32 - n));
    }
    void block(const std::uint8_t *p) {
        static const std::uint32_t k[64] = {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
        std::uint32_t w[64];
        for (int i = 0; i < 16; ++i)
            w[i] = std::uint32_t(p[4 * i]) << 24 | std::uint32_t(p[4 * i + 1]) << 16 |
                   std::uint32_t(p[4 * i + 2]) << 8 | p[4 * i + 3];
        for (int i = 16; i < 64; ++i) {
            const auto s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const auto s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        auto a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const auto t1 = hh + (rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25)) + ((e & f) ^ (~e & g)) + k[i] + w[i];
            const auto t2 = (rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            hh = g, g = f, f = e, e = d + t1, d = c, c = b, b = a, a = t1 + t2;
        }
        h[0] += a, h[1] += b, h[2] += c, h[3] += d, h[4] += e, h[5] += f, h[6] += g, h[7] += hh;
    }
};

std::filesystem::path defaultShaders(const std::filesystem::path &given) {
    if (!given.empty())
        return given;
    if (const char *env = std::getenv("NEREUS_SHADER_DIR"))
        return env;
#ifdef NEREUS_RENDERING_SHADERS
    return NEREUS_RENDERING_SHADERS;
#else
    return {};
#endif
}

CameraInfo makeInfo(const cameras::Intrinsics &k, bool right, double baseline) {
    CameraInfo info;
    info.width = k.width;
    info.height = k.height;
    info.k = {k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1};
    const double tx = right ? -k.fx * baseline : 0.0;
    info.p = {k.fx, 0, k.cx, tx, 0, k.fy, k.cy, 0, 0, 0, 1, 0};
    return info;
}
} // namespace

const char *outputName(Output output) {
    switch (output) {
    case Output::RgbLeft:
        return "rgb_left";
    case Output::DepthLeft:
        return "depth_left";
    case Output::RgbRight:
        return "rgb_right";
    }
    return "?";
}

std::optional<Output> outputFromName(const std::string &name) {
    for (const auto output : {Output::RgbLeft, Output::DepthLeft, Output::RgbRight})
        if (name == outputName(output))
            return output;
    return std::nullopt;
}

std::array<std::uint8_t, 32> sha256(const std::string &text) {
    Sha256 state;
    std::string data = text;
    const std::uint64_t bits = std::uint64_t(text.size()) * 8;
    data.push_back(char(0x80));
    while (data.size() % 64 != 56)
        data.push_back(0);
    for (int i = 7; i >= 0; --i)
        data.push_back(char((bits >> (8 * i)) & 0xff));
    for (std::size_t i = 0; i < data.size(); i += 64)
        state.block(reinterpret_cast<const std::uint8_t *>(data.data()) + i);
    std::array<std::uint8_t, 32> digest{};
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 4; ++j)
            digest[std::size_t(4 * i + j)] = std::uint8_t(state.h[i] >> (24 - 8 * j));
    return digest;
}

std::uint32_t deriveSeed(std::uint64_t seed, const std::string &sensor_id, const std::string &eye) {
    std::string text = "nereus.camera.v1";
    text.push_back('\0');
    text += std::to_string(seed);
    text.push_back('\0');
    text += sensor_id;
    text.push_back('\0');
    text += eye;
    const auto digest = sha256(text);
    return std::uint32_t(digest[0]) << 24 | std::uint32_t(digest[1]) << 16 | std::uint32_t(digest[2]) << 8 | digest[3];
}

SessionCameras::SessionCameras(const session::ResolvedScenario &resolved, Options options,
                               std::shared_ptr<const pack_scene::PackScene> scene)
    : scene_(std::move(scene)), options_(std::move(options)), always_(options_.always) {
    const auto &scenarioJson = resolved.scenario;
    sensor_noise_ = scenarioJson.value("sensor_noise", options_.sensor_noise);
    if (options_.supersample < 1 || options_.supersample > 4)
        throw std::runtime_error("camera supersample must be 1..4");
    seed_ = scenarioJson.value("seed", std::uint64_t(0));
    if (!scene_)
        scene_ = std::make_shared<pack_scene::PackScene>(resolved);

    std::map<std::string, const Json *> sensors;
    std::vector<std::string> order;
    for (const auto &item : resolved.robot.at("sensors")) {
        sensors[item.at("id").get<std::string>()] = &item;
        order.push_back(item.at("id").get<std::string>());
    }
    std::vector<std::string> chosen;
    if (options_.sensor_ids.empty()) {
        for (const auto &id : order)
            if (sensors.at(id)->at("type") == "stereo_camera" && sensors.at(id)->value("enabled", true))
                chosen.push_back(id);
    } else {
        for (const auto &id : options_.sensor_ids)
            if (std::find(chosen.begin(), chosen.end(), id) == chosen.end())
                chosen.push_back(id);
    }
    if (chosen.empty())
        throw std::runtime_error("no enabled stereo_camera sensors selected");
    for (const auto &id : chosen) {
        const auto found = sensors.find(id);
        if (found == sensors.end())
            throw std::runtime_error("unknown robot sensor '" + id + "'");
        const Json &item = *found->second;
        if (item.at("type") != "stereo_camera")
            throw std::runtime_error("sensor '" + id + "' is a " + item.at("type").get<std::string>() +
                                     ", not a stereo_camera");
        if (!item.value("enabled", true))
            throw std::runtime_error("sensor '" + id + "' is disabled");
        if (item.value("latency_ns", 0) != 0)
            throw std::runtime_error("camera '" + id + "': nonzero delivery latency is not implemented");
        const auto overflow = item.value("overflow", std::string("drop_oldest"));
        if (item.value("capacity", 1) < 1 || (overflow != "fail" && overflow != "drop_oldest"))
            throw std::runtime_error("camera '" + id + "': invalid pending queue policy");

        auto camera = std::make_unique<Camera>();
        camera->id = id;
        camera->frame = item.at("frame").get<std::string>();
        camera->period_ns = item.at("period_ns").get<std::int64_t>();
        if (camera->period_ns <= 0)
            throw std::runtime_error("camera '" + id + "': period_ns must be positive");
        camera->capacity = item.value("capacity", 1);
        camera->fail_on_overflow = overflow == "fail";
        const auto &parameters = item.at("parameters");
        const int width = parameters.at("resolution_px").at(0).get<int>();
        const int height = parameters.at("resolution_px").at(1).get<int>();
        for (int eye = 0; eye < 2; ++eye) {
            auto &k = camera->intrinsics[std::size_t(eye)];
            const auto &values = parameters.at(std::string("intrinsics_") + kEyes[eye]);
            k.width = width;
            k.height = height;
            k.fx = values.at("fx").get<double>();
            k.fy = values.at("fy").get<double>();
            k.cx = values.at("cx").get<double>();
            k.cy = values.at("cy").get<double>();
            k.near_plane = kNearPlaneM;
            k.far_plane = kFarPlaneM;
            k.projection(); // validates against the calibration invariants
        }
        const auto &depth = parameters.at("depth");
        auto &noise = camera->noise;
        const auto &n = depth.at("noise");
        noise.enabled = sensor_noise_;
        noise.min_range = depth.at("min_range_m").get<double>();
        noise.max_range = depth.at("max_range_m").get<double>();
        noise.base_sigma = n.at("base_sigma_m").get<double>();
        noise.range_sigma = n.at("range_coefficient").get<double>();
        noise.exponent = n.at("range_exponent").get<double>();
        noise.bias = n.at("bias_m").get<double>();
        noise.dropout = n.at("dropout").get<double>();
        noise.range_dropout = n.at("range_dropout").get<double>();
        noise.edge_dropout = n.at("edge_dropout").get<double>();
        noise.outliers = n.at("outliers").get<double>();
        noise.correlation = n.at("correlation").get<double>();
        noise.patch_size = n.at("patch_size_px").get<int>();
        noise.validate();
        camera->baseline_m = parameters.at("baseline_m").get<double>();
        for (const auto &output : parameters.at("outputs"))
            if (const auto known = outputFromName(output.get<std::string>()))
                camera->outputs.insert(*known);
        if (parameters.contains("right_frame") && !parameters.at("right_frame").is_null())
            camera->right_frame = parameters.at("right_frame").get<std::string>();
        else if (camera->outputs.count(Output::RgbRight))
            throw std::runtime_error("sensor '" + id + "': rgb_right requires right_frame");
        camera->left_eye = scene_->frames().fromRoot(camera->frame);
        camera->right_eye =
            camera->right_frame.empty() ? camera->left_eye : scene_->frames().fromRoot(camera->right_frame);
        const auto quality = options_.jpeg_quality.find(id);
        if (quality != options_.jpeg_quality.end())
            camera->jpeg_quality = quality->second;
        cameras_[id] = std::move(camera);
    }
    host_ = std::make_unique<rendering::OffscreenRenderer>(defaultShaders(options_.shader_directory));
    reseedAll(seed_);
}

SessionCameras::~SessionCameras() {
    try {
        close();
    } catch (...) {
    }
}

SessionCameras::Camera &SessionCameras::camera(const std::string &id) const {
    const auto found = cameras_.find(id);
    if (found == cameras_.end())
        throw std::invalid_argument("sensor '" + id + "' is not a selected camera");
    return *found->second;
}

void SessionCameras::reseedAll(std::uint64_t seed) {
    for (auto &[id, camera] : cameras_)
        for (std::size_t eye = 0; eye < 2; ++eye) {
            camera->seeds[eye] = deriveSeed(seed, id, kEyes[eye]);
            camera->processors[eye].reset(camera->seeds[eye]);
        }
    seed_ = seed;
}

std::vector<std::string> SessionCameras::cameraIds() const {
    std::vector<std::string> ids;
    for (const auto &[id, camera] : cameras_)
        ids.push_back(id);
    return ids;
}

bool SessionCameras::hasOutput(const std::string &id, Output output) const {
    return camera(id).outputs.count(output) != 0;
}

double SessionCameras::periodSeconds(const std::string &id) const {
    return static_cast<double>(camera(id).period_ns) * 1e-9;
}

CameraInfo SessionCameras::info(const std::string &id, const std::string &eye) const {
    const auto &c = camera(id);
    if (eye != "left" && eye != "right")
        throw std::invalid_argument("eye must be 'left' or 'right'");
    return makeInfo(c.intrinsics[eye == "right"], eye == "right", c.baseline_m);
}

void SessionCameras::setDemand(const std::string &id, Output output, bool wanted, const std::string &consumer) {
    auto &c = camera(id);
    if (!c.outputs.count(output))
        throw std::invalid_argument("camera '" + id + "' has no configured output '" + outputName(output) + "'");
    std::lock_guard<std::mutex> lock(mutex_);
    auto &set = c.demand[output];
    if (wanted)
        set.insert(consumer);
    else
        set.erase(consumer);
}

void SessionCameras::setAlways(bool always) {
    std::lock_guard<std::mutex> lock(mutex_);
    always_ = always;
}

void SessionCameras::setJpegQuality(const std::string &id, std::optional<int> quality) {
    if (quality && (*quality < 0 || *quality > 100))
        throw std::invalid_argument("jpeg quality must be in [0, 100]");
    auto &c = camera(id);
    std::lock_guard<std::mutex> lock(mutex_);
    c.jpeg_quality = quality;
}

void SessionCameras::start(Callback deliver) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!threads_.empty() || stopping_)
        throw std::runtime_error("camera workers cannot be started twice or after close");
    deliver_ = std::move(deliver);
    for (auto &[id, camera] : cameras_)
        threads_.emplace_back([this, c = camera.get()] { run(*c); });
}

void SessionCameras::raiseFailure() const {
    if (failure_) {
        try {
            std::rethrow_exception(failure_);
        } catch (const std::exception &error) {
            throw std::runtime_error(std::string("camera worker failed: ") + error.what());
        }
    }
}

void SessionCameras::request(std::int64_t snapshot_time_ns, std::int64_t ros_stamp_ns,
                             const spatial::Pose &world_from_root, const std::vector<rendering::Instance> &dynamic,
                             const std::vector<pack_scene::RobotOverride> &overrides,
                             const std::map<std::string, bool> &latched) {
    spatial::validate(world_from_root);
    std::shared_ptr<const std::vector<rendering::Instance>> sharedDynamic;
    std::shared_ptr<const std::vector<pack_scene::RobotOverride>> sharedOverrides;
    std::lock_guard<std::mutex> lock(mutex_);
    raiseFailure();
    if (stopping_)
        throw std::runtime_error("camera worker is closed");
    for (auto &[id, holder] : cameras_) {
        auto &c = *holder;
        if (snapshot_time_ns < c.next_ns)
            continue;
        c.next_ns = (snapshot_time_ns / c.period_ns + 1) * c.period_ns;
        ++c.stats.requested;
        const auto wanted = [&](Output output) {
            if (!c.outputs.count(output))
                return false;
            if (always_)
                return true;
            const auto found = c.demand.find(output);
            return found != c.demand.end() && !found->second.empty();
        };
        Job job;
        job.rgb_left = wanted(Output::RgbLeft);
        job.depth_left = wanted(Output::DepthLeft);
        job.rgb_right = wanted(Output::RgbRight);
        if (!job.rgb_left && !job.depth_left && !job.rgb_right) {
            ++c.stats.skipped_no_demand;
            continue;
        }
        if (!sharedDynamic) {
            sharedDynamic = std::make_shared<const std::vector<rendering::Instance>>(dynamic);
            sharedOverrides = std::make_shared<const std::vector<pack_scene::RobotOverride>>(overrides);
        }
        job.native_ns = snapshot_time_ns;
        job.ros_ns = ros_stamp_ns;
        job.revision = revision_;
        job.root = world_from_root;
        job.dynamic = sharedDynamic;
        job.overrides = sharedOverrides;
        job.latched = latched;
        job.jpeg_quality = c.jpeg_quality;
        if (c.pending.size() >= c.capacity) {
            if (c.fail_on_overflow)
                throw std::runtime_error("camera '" + id + "' pending queue is full");
            c.pending.pop_front();
            ++c.stats.dropped_pending;
        }
        c.pending.push_back(std::move(job));
    }
    condition_.notify_all();
}

void SessionCameras::discardPending() {
    for (auto &[id, camera] : cameras_) {
        camera->stats.discarded_stale += camera->pending.size();
        camera->pending.clear();
    }
}

void SessionCameras::invalidate(std::optional<std::uint64_t> seed) {
    std::lock_guard<std::mutex> publication(publication_mutex_);
    std::lock_guard<std::mutex> lock(mutex_);
    ++revision_;
    discardPending();
    if (seed) {
        reset_seed_ = seed;
        for (auto &[id, camera] : cameras_)
            camera->next_ns = 0;
    }
    condition_.notify_all();
}

void SessionCameras::close() {
    std::vector<std::thread> threads;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        ++revision_;
        discardPending();
        threads.swap(threads_);
        condition_.notify_all();
    }
    for (auto &thread : threads)
        if (thread.joinable())
            thread.join();
    std::lock_guard<std::mutex> lock(mutex_);
    raiseFailure();
}

std::map<std::string, CameraStats> SessionCameras::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, CameraStats> result;
    for (const auto &[id, camera] : cameras_)
        result[id] = camera->stats;
    return result;
}

Json SessionCameras::describe() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Json cameraRecords = Json::object();
    for (const auto &[id, holder] : cameras_) {
        const auto &c = *holder;
        Json outputs = Json::array();
        for (const auto output : c.outputs)
            outputs.push_back(outputName(output));
        const auto &n = c.noise;
        cameraRecords[id] = {{"frame", c.frame},
                             {"right_frame", c.right_frame},
                             {"outputs", outputs},
                             {"baseline_m", c.baseline_m},
                             {"period_ns", c.period_ns},
                             {"capacity", c.capacity},
                             {"seeds", {c.seeds[0], c.seeds[1]}},
                             {"depth_noise",
                              {{"enabled", n.enabled},
                               {"min_range", n.min_range},
                               {"max_range", n.max_range},
                               {"base_sigma", n.base_sigma},
                               {"range_sigma", n.range_sigma},
                               {"exponent", n.exponent},
                               {"bias", n.bias},
                               {"dropout", n.dropout},
                               {"range_dropout", n.range_dropout},
                               {"edge_dropout", n.edge_dropout},
                               {"outliers", n.outliers},
                               {"correlation", n.correlation},
                               {"patch_size", n.patch_size}}}};
    }
    return {{"device", host_->device()},
            {"clipping_m", {{"near", kNearPlaneM}, {"far", kFarPlaneM}}},
            {"sensor_noise", sensor_noise_},
            {"seed", seed_},
            {"always", always_},
            {"supersample", options_.supersample},
            {"cameras", cameraRecords},
            {"scene", scene_->describe()}};
}

Products SessionCameras::capture(Camera &c, const Job &job) {
    Products products;
    products.camera = c.id;
    products.snapshot_time_ns = job.native_ns;
    products.ros_stamp_ns = job.ros_ns;
    products.revision = job.revision;
    products.left_info = makeInfo(c.intrinsics[0], false, c.baseline_m);
    products.right_info = makeInfo(c.intrinsics[1], true, c.baseline_m);

    const auto root = pack_scene::toMatrix(job.root);
    const auto scene = scene_->compose(root, *job.dynamic, *job.overrides, job.latched);
    const auto viewFor = [&](int eye) {
        const auto worldEye = spatial::compose(job.root, eye ? c.right_eye : c.left_eye);
        rendering::View view;
        view.view = cameras::opticalView(worldEye);
        view.projection = c.intrinsics[std::size_t(eye)].projection();
        view.eye = worldEye.translation.cast<float>();
        return view;
    };
    const float time = static_cast<float>(static_cast<double>(job.native_ns) / 1e9);
    std::array<rendering::ImageCapture, 2> raw;
    const bool wantEye[2] = {job.rgb_left || job.depth_left, job.rgb_right};
    const bool wantColor[2] = {job.rgb_left, job.rgb_right};
    const bool wantDepth[2] = {job.depth_left, false};
    std::optional<rendering::View> views[2];
    auto appearance = scene_->appearance();
    appearance.supersample = options_.supersample;
    for (int eye = 0; eye < 2; ++eye)
        if (wantEye[eye])
            views[eye] = viewFor(eye); // validates before rendering
    const auto renderStart = nowNs();
    {
        std::lock_guard<std::mutex> gl(gl_mutex_);
        for (int eye = 0; eye < 2; ++eye)
            if (wantEye[eye]) {
                const auto &k = c.intrinsics[std::size_t(eye)];
                raw[std::size_t(eye)] = host_->capture(scene, *views[eye], appearance, time, k.width, k.height,
                                                       wantColor[eye], wantDepth[eye]);
            }
    }
    const auto renderNs = nowNs() - renderStart;
    // The right eye has no depth and so no random state; processing the left eye last keeps a failed
    // call from consuming the left eye's noise stream.
    const auto processStart = nowNs();
    for (int eye = 1; eye >= 0; --eye) {
        if (!wantEye[eye])
            continue;
        const bool jpeg = wantColor[eye] && job.jpeg_quality.has_value();
        auto frame = c.processors[std::size_t(eye)].process(c.intrinsics[std::size_t(eye)], c.noise,
                                                            raw[std::size_t(eye)].rgb, raw[std::size_t(eye)].depth,
                                                            jpeg, job.jpeg_quality.value_or(kDefaultJpegQuality));
        (eye ? products.right : products.left) = std::move(frame);
    }
    products.render_ms = ms(renderNs);
    products.process_ms = ms(nowNs() - processStart);
    return products;
}

void SessionCameras::run(Camera &c) {
    try {
        while (true) {
            Job job;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [&] {
                    return stopping_ || (!resetting_ && (reset_seed_ ? active_ == 0 : !c.pending.empty()));
                });
                if (stopping_)
                    return;
                if (reset_seed_) {
                    const auto seed = *reset_seed_;
                    reset_seed_.reset();
                    resetting_ = true;
                    lock.unlock();
                    reseedAll(seed);
                    lock.lock();
                    resetting_ = false;
                    condition_.notify_all();
                    continue;
                }
                job = std::move(c.pending.front());
                c.pending.pop_front();
                ++active_;
            }
            const auto started = nowNs();
            Products products;
            try {
                products = capture(c, job);
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                --active_;
                throw;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                --active_;
                ++c.stats.captured;
                c.stats.render_ns += static_cast<std::uint64_t>(products.render_ms * 1e6);
                c.stats.process_ns += static_cast<std::uint64_t>(products.process_ms * 1e6);
                c.stats.capture_wall_ns += static_cast<std::uint64_t>(nowNs() - started);
                condition_.notify_all();
            }
            std::lock_guard<std::mutex> publication(publication_mutex_);
            bool stale;
            Callback deliver;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                stale = job.revision != revision_ || stopping_;
                if (stale)
                    ++c.stats.discarded_stale;
                deliver = deliver_;
            }
            if (!stale && deliver) {
                deliver(std::move(products));
                std::lock_guard<std::mutex> lock(mutex_);
                ++c.stats.delivered;
            }
        }
    } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!failure_)
            failure_ = std::current_exception();
        stopping_ = true;
        condition_.notify_all();
    }
}
} // namespace nereus::session_cameras
