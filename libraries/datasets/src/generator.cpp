#include <nereus/datasets/environment.hpp>
#include <nereus/datasets/generator.hpp>
#include <nereus/datasets/output.hpp>

#include <nereus/cameras/camera.hpp>
#include <nereus/session/scenario.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace nereus::datasets {
namespace r = nereus::rendering;
namespace ps = nereus::pack_scene;
namespace {
constexpr double kNearPlaneM = 0.05, kFarPlaneM = 100.0; // as SessionCameras
constexpr double kDeg = M_PI / 180;

double msSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

std::filesystem::path shaderDirectory(const std::filesystem::path &given) {
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

r::View viewFor(const Pose &world_from_optical, const cameras::Intrinsics &k) {
    r::View view;
    view.view = cameras::opticalView(world_from_optical);
    view.projection = k.projection();
    view.eye = world_from_optical.translation.cast<float>();
    return view;
}

// Part-map PNG value at uv (0, 0): the bottom-left pixel (diffuse row convention).
std::optional<int> maskValueAtOrigin(const std::filesystem::path &mask) {
    const cv::Mat image = cv::imread(mask.string(), cv::IMREAD_UNCHANGED);
    if (image.empty() || image.depth() != CV_8U)
        return std::nullopt;
    return int(image.ptr<std::uint8_t>(image.rows - 1)[0]);
}
} // namespace

struct TaskInfo {
    Pose world_from_task;
    std::map<std::string, Pose> frames; // task-relative, including "task"
};

// The scenario's view of a sample's drawn appearance and placement.
struct Draw {
    r::Appearance appearance;
    float time = 0;
    std::map<std::string, Pose> world_from_task; // jittered
    std::map<std::string, bool> latched;
    double noise_sigma = 0, blur_px = 0;
    std::uint64_t noise_seed = 0;
    Json record;
};

struct Generator::Scenario {
    std::string id;
    std::size_t index = 0;
    session::ResolvedScenario resolved;
    std::unique_ptr<ps::PackScene> pack;
    PoolFrame pool;
    Pose root_from_optical;
    cameras::Intrinsics output, accept;
    std::vector<std::string> task_order;
    std::map<std::string, TaskInfo> tasks;
    std::map<std::string, std::pair<std::string, std::string>> indicator_names; // region -> initial, latched
    std::vector<std::string> regions;
    std::vector<SceneEntry> entries; // composed order: static, robot (non-null meshes, if drawn), props
    std::size_t static_count = 0, robot_count = 0;
    std::vector<Eigen::Matrix4d> robot_root_from_asset;
    std::vector<Eigen::Matrix4d> prop_reset;
    std::vector<r::InstanceLabel> labels;
    std::map<std::uint32_t, std::pair<std::size_t, std::size_t>> keys; // label key -> entry, part index
    std::vector<std::uint8_t> self_output;                             // robot pixels (bottom-up), 1 = robot
};

Generator::Generator(Job job, GeneratorOptions options) : job_(std::move(job)), options_(std::move(options)) {
    if (!(options_.acceptance_scale > 0 && options_.acceptance_scale <= 1))
        throw std::invalid_argument("acceptance scale must be in (0, 1]");
    validatePartMasks(job_);
    host_ = std::make_unique<r::OffscreenRenderer>(shaderDirectory(options_.shader_directory));
    scenarios_.resize(job_.scenarios.size());
}

Generator::~Generator() = default;

const std::string &Generator::device() const {
    return host_->device();
}

Generator::Scenario &Generator::scenario(std::size_t index) {
    auto &slot = scenarios_.at(index);
    if (slot)
        return *slot;
    auto s = std::make_unique<Scenario>();
    s->id = job_.scenarios[index].first;
    s->index = index;
    s->resolved = session::loadResolvedScenario(job_.scenarios[index].second);
    s->pack = std::make_unique<ps::PackScene>(s->resolved, ps::Options{true});
    s->pool = PoolFrame::fromScenario(s->resolved);

    // Camera: the sensor's left eye, as SessionCameras.
    const session::Json *sensor = nullptr;
    for (const auto &item : s->resolved.robot.at("sensors"))
        if (item.at("id").get<std::string>() == job_.camera.sensor)
            sensor = &item;
    if (!sensor || sensor->at("type") != "stereo_camera")
        throw std::runtime_error("scenario '" + s->id + "': no stereo_camera sensor '" + job_.camera.sensor + "'");
    const auto &parameters = sensor->at("parameters");
    cameras::Intrinsics native;
    native.width = parameters.at("resolution_px").at(0).get<int>();
    native.height = parameters.at("resolution_px").at(1).get<int>();
    const auto &left = parameters.at("intrinsics_left");
    native.fx = left.at("fx").get<double>();
    native.fy = left.at("fy").get<double>();
    native.cx = left.at("cx").get<double>();
    native.cy = left.at("cy").get<double>();
    native.near_plane = kNearPlaneM;
    native.far_plane = kFarPlaneM;
    native.validate();
    const int width = job_.camera.width ? job_.camera.width : native.width;
    const int height = job_.camera.height ? job_.camera.height : native.height;
    s->output = scaleIntrinsics(native, width, height);
    s->accept = scaleIntrinsics(s->output, std::max(1, int(std::lround(width * options_.acceptance_scale))),
                                std::max(1, int(std::lround(height * options_.acceptance_scale))));
    s->root_from_optical = s->pack->frames().fromRoot(sensor->at("frame").get<std::string>());

    // Tasks: placements, frames, indicator colour names.
    std::map<std::string, session::Json> placements;
    for (const auto &item : s->resolved.scenario.at("task_placements"))
        placements[item.at("task").get<std::string>()] = item;
    for (const auto &task : s->resolved.task_definitions) {
        const auto id = task.at("id").get<std::string>();
        TaskInfo info;
        info.world_from_task =
            ps::upright(placements.at(id).at("position_m"), placements.at(id).at("yaw_deg").get<double>());
        info.frames["task"] = Pose{};
        for (const auto &item : task.at("frames"))
            info.frames[item.at("id").get<std::string>()] = ps::placement(item);
        for (const auto &region : task.at("regions")) {
            const auto &p = region.value("parameters", session::Json::object());
            if (p.contains("indicator"))
                s->indicator_names[region.at("id").get<std::string>()] = {
                    p.at("indicator").at("initial").get<std::string>(),
                    p.at("indicator").at("latched").get<std::string>()};
        }
        s->task_order.push_back(id);
        s->tasks[id] = std::move(info);
    }
    for (const auto &item : s->pack->indicatorVisuals())
        if (std::find(s->regions.begin(), s->regions.end(), item.region) == s->regions.end())
            s->regions.push_back(item.region);

    // Scene entries in composed order, parts resolved once (split rewrites cached).
    const auto &statics = s->pack->staticScene().instances;
    const auto &sources = s->pack->staticSources();
    if (sources.size() != statics.size())
        throw std::logic_error("pack scene static sources are not parallel to its static scene");
    for (std::size_t i = 0; i < statics.size(); ++i) {
        const auto &source = sources[i];
        SceneEntry entry;
        entry.role = source.role;
        if (source.role == "task") {
            entry.origin = {source.task, source.prop, source.asset, source.frame};
            entry.source = source.task + "/" + source.prop + "/" + source.asset + "#" + std::to_string(source.visual);
        } else {
            entry.origin.asset = source.asset;
            entry.source =
                source.role + "/" + (source.asset.empty() ? "generated" : source.asset) + "#" + std::to_string(i);
        }
        entry.parts = resolveParts(statics[i].mesh, entry.origin, job_, &split_cache_);
        s->entries.push_back(std::move(entry));
    }
    s->static_count = s->entries.size();
    for (const auto &visual : s->pack->robotVisuals())
        if (visual.mesh) {
            s->robot_root_from_asset.push_back(visual.rootFromAsset());
            if (job_.camera.robot_visuals) {
                SceneEntry entry;
                entry.role = "robot";
                entry.origin.asset = visual.asset;
                entry.source = "robot/" + visual.asset;
                entry.parts.mesh = visual.mesh;
                s->entries.push_back(std::move(entry));
            }
        }
    s->robot_count = job_.camera.robot_visuals ? s->robot_root_from_asset.size() : 0;
    for (const auto &visual : s->pack->propVisuals()) {
        if (!visual.mesh)
            continue;
        SceneEntry entry;
        entry.role = "prop";
        entry.origin = {visual.task, visual.prop, visual.asset, visual.frame};
        entry.source = visual.task + "/" + visual.prop + "/" + visual.asset + "#prop";
        entry.parts = resolveParts(visual.mesh, entry.origin, job_, &split_cache_);
        s->prop_reset.push_back(visual.world_from_asset_at_reset);
        s->entries.push_back(std::move(entry));
    }
    for (std::size_t i = 0; i < s->entries.size(); ++i) {
        auto &entry = s->entries[i];
        r::InstanceLabel label;
        if (!entry.parts.empty()) {
            entry.label_id = static_cast<std::uint32_t>(i + 1);
            label.id = entry.label_id;
            label.submeshes = entry.parts.submeshes;
            for (std::size_t p = 0; p < entry.parts.parts.size(); ++p)
                s->keys[entry.label_id << 8 | entry.parts.parts[p].value] = {i, p};
        }
        s->labels.push_back(std::move(label));
    }

    // The robot's own pixels (fixed in the camera image): excluded from the near-geometry check.
    if (s->robot_count) {
        r::Scene robot;
        std::vector<r::InstanceLabel> none;
        for (std::size_t j = 0; j < s->robot_count; ++j) {
            r::Instance instance;
            instance.mesh = s->entries[s->static_count + j].parts.mesh;
            instance.transform = s->robot_root_from_asset[j].cast<float>();
            robot.instances.push_back(std::move(instance));
            none.emplace_back();
        }
        const auto &k = s->output;
        const auto capture = host_->captureLabels(robot, none, viewFor(s->root_from_optical, k), k.width, k.height);
        s->self_output.resize(capture.depth.size());
        for (std::size_t p = 0; p < capture.depth.size(); ++p)
            s->self_output[p] = capture.depth[p] < 1.f ? 1 : 0;
    }
    slot = std::move(s);
    return *slot;
}

Json Generator::describe() {
    auto &s = scenario(0);
    Json instances = Json::array();
    std::map<std::filesystem::path, std::optional<int>> origins;
    for (std::size_t i = 0; i < s.entries.size(); ++i) {
        const auto &entry = s.entries[i];
        const auto &mesh = entry.parts.mesh;
        Json submeshes = Json::array();
        for (std::size_t j = 0; mesh && j < mesh->submeshes.size(); ++j) {
            const auto &submesh = mesh->submeshes[j];
            const auto texture = canonicalTexture(submesh);
            Json item = {{"material", submesh.material.name},
                         {"diffuse", texture.empty() ? Json() : Json(texture.string())},
                         {"triangles", submesh.indices.size() / 3}};
            Json part = nullptr, partMap = nullptr;
            if (!entry.parts.submeshes.empty()) {
                const auto &label = entry.parts.submeshes[j];
                std::optional<int> value = label.part;
                if (label.part_map) {
                    partMap = label.part_map->string();
                    if (!origins.count(*label.part_map))
                        origins[*label.part_map] = maskValueAtOrigin(*label.part_map);
                    value = origins[*label.part_map];
                }
                if (value) {
                    std::string name;
                    for (const auto &p : entry.parts.parts)
                        if (p.value == *value)
                            name = p.part;
                    part = {{"value", *value}, {"part", name.empty() ? Json() : Json(name)}};
                }
            }
            item["part_map"] = partMap;
            item["part_at_uv00"] = part;
            submeshes.push_back(std::move(item));
        }
        Json parts = Json::array();
        for (const auto &p : entry.parts.parts)
            parts.push_back({{"value", p.value},
                             {"part", p.part},
                             {"piece", p.piece},
                             {"indicator", p.indicator.empty() ? Json() : Json(p.indicator)}});
        const auto &o = entry.origin;
        instances.push_back({{"index", i},
                             {"role", entry.role},
                             {"source", entry.source},
                             {"task", o.task.empty() ? Json() : Json(o.task)},
                             {"prop", o.prop.empty() ? Json() : Json(o.prop)},
                             {"asset", o.asset.empty() ? Json() : Json(o.asset)},
                             {"frame", o.frame.empty() ? Json() : Json(o.frame)},
                             {"texture_override", i < s.static_count && !s.pack->staticSources()[i].texture.empty()
                                                      ? Json(s.pack->staticSources()[i].texture)
                                                      : Json()},
                             {"rule", entry.parts.rule ? Json(*entry.parts.rule) : Json()},
                             {"label_id", entry.label_id},
                             {"submeshes", submeshes},
                             {"parts", parts}});
    }
    const auto &k = s.output;
    // Share of the image the robot's own visuals cover (excluded from the near-geometry check).
    double robotFraction = 0;
    for (const auto pixel : s.self_output)
        robotFraction += pixel;
    robotFraction /= std::max<double>(1, double(s.self_output.size()));
    return {{"scenario", s.id},
            {"device", host_->device()},
            {"camera",
             {{"sensor", job_.camera.sensor},
              {"width", k.width},
              {"height", k.height},
              {"fx", k.fx},
              {"fy", k.fy},
              {"cx", k.cx},
              {"cy", k.cy},
              {"root_from_camera", poseJson(s.root_from_optical)},
              {"robot_pixel_fraction", robotFraction}}},
            {"instances", instances},
            {"pack_scene", s.pack->describe()}};
}

Json Generator::render(std::int64_t k) {
    const auto started = std::chrono::steady_clock::now();
    const auto &block = job_.block(k);
    const auto name = job_.name(k);
    const auto ext = job_.camera.format == "png" ? ".png" : ".jpg";
    const auto out = job_.output;
    const auto recordPath = out / "records" / (name + ".json");
    Json log = {{"sample", k}, {"name", name}, {"task", block.task ? Json(*block.task) : Json()}};
    const std::string imageRel = std::string("images/") + name + ext, idsRel = "ids/" + name + ".png";
    // Complete = record (written last) plus non-empty image and id map; anything less is rendered again.
    if (options_.resume && nonEmptyFile(recordPath) && nonEmptyFile(out / imageRel) && nonEmptyFile(out / idsRel)) {
        log["status"] = "existing";
        return log;
    }
    const std::size_t scenarioIndex = static_cast<std::size_t>(k % std::int64_t(job_.scenarios.size()));
    auto &s = scenario(scenarioIndex);
    if (block.task && !s.tasks.count(*block.task))
        throw std::runtime_error("scenario '" + s.id + "' has no task '" + *block.task + "'");
    Stream rng(job_.seed, k);

    // Environment first (weighted mode: the stream's first draw), then its values relative to the pool pack.
    const auto &z = job_.randomize;
    const std::size_t environmentIndex = selectEnvironment(job_, k, rng);
    const auto &environment = z.environments[environmentIndex];
    auto drawn = drawEnvironment(environment, s.pack->appearance(), rng);
    Draw d;
    d.appearance = drawn.appearance;
    d.time = drawn.time;
    d.noise_sigma = drawn.noise_sigma;
    d.blur_px = drawn.blur_px;
    Json placementRecord = Json::object();
    // One draw per group (when its first member comes up in task order) or per ungrouped task.
    std::map<std::string, const std::vector<std::string> *> groupOf;
    for (const auto &group : z.placement_groups)
        for (const auto &task : group) {
            if (!s.tasks.count(task))
                throw std::runtime_error("randomize.placement.groups: scenario '" + s.id + "' has no task '" + task +
                                         "'");
            groupOf[task] = &group;
        }
    std::map<const std::vector<std::string> *, std::pair<double, Eigen::Vector3d>> groupDraws;
    for (const auto &task : s.task_order) {
        const auto group = groupOf.find(task);
        std::pair<double, Eigen::Vector3d> jitter;
        if (group != groupOf.end() && groupDraws.count(group->second))
            jitter = groupDraws.at(group->second);
        else {
            const double yaw = rng.symmetric(z.task_yaw_deg) * kDeg;
            const double radius = z.task_offset_m * std::sqrt(rng.uniform()), angle = rng.uniform(0, 2 * M_PI);
            jitter = {yaw, Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0)};
            if (group != groupOf.end())
                groupDraws[group->second] = jitter;
        }
        const auto &[yaw, offset] = jitter;
        const Eigen::Quaterniond turn(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        const Pose &original = s.tasks.at(task).world_from_task;
        Pose placed;
        if (group == groupOf.end()) {
            // Rotate about the task origin, offset in world XY.
            placed = spatial::compose(original, Pose{Eigen::Vector3d::Zero(), turn});
            placed.translation += offset;
        } else {
            // Rotate about the group's first task's origin, offset in world XY.
            const Eigen::Vector3d pivot = s.tasks.at(group->second->front()).world_from_task.translation;
            placed.rotation = (turn * original.rotation).normalized();
            placed.translation = pivot + turn * (original.translation - pivot) + offset;
        }
        d.world_from_task[task] = placed;
        placementRecord[task] = {{"yaw_deg", yaw / kDeg}, {"offset_m", {offset.x(), offset.y()}}};
        if (group != groupOf.end())
            placementRecord[task]["pivot_task"] = group->second->front();
    }
    Json indicatorRecord = Json::object();
    for (const auto &region : s.regions) {
        d.latched[region] = rng.chance(z.latched_probability);
        indicatorRecord[region] = d.latched[region];
    }
    d.noise_seed = rng.bits();
    d.record = std::move(drawn.record);
    d.record["environment"] = environment.id;
    d.record["placement"] = placementRecord;
    d.record["latched"] = indicatorRecord;

    // Task deltas (world), applied to static task visuals and props.
    std::map<std::string, Eigen::Matrix4d> delta;
    for (const auto &task : s.task_order)
        delta[task] =
            ps::toMatrix(d.world_from_task.at(task)) * ps::toMatrix(spatial::inverse(s.tasks.at(task).world_from_task));
    std::vector<r::Instance> props;
    for (std::size_t p = 0; p < s.prop_reset.size(); ++p) {
        const auto &entry = s.entries[s.static_count + s.robot_count + p];
        r::Instance instance;
        instance.mesh = entry.parts.mesh;
        instance.transform = (delta.at(entry.origin.task) * s.prop_reset[p]).cast<float>();
        props.push_back(std::move(instance));
    }
    r::Scene scene = s.pack->compose(Eigen::Matrix4d::Identity(), props, {}, d.latched);
    // compose() draws every robot visual; keep them only when the job wants them (they follow the static part).
    const std::size_t composedRobots = s.robot_root_from_asset.size();
    if (!s.robot_count)
        scene.instances.erase(scene.instances.begin() + static_cast<std::ptrdiff_t>(s.static_count),
                              scene.instances.begin() + static_cast<std::ptrdiff_t>(s.static_count + composedRobots));
    if (scene.instances.size() != s.entries.size())
        throw std::logic_error("composed scene does not match the generator's instance table");
    for (std::size_t i = 0; i < s.static_count; ++i) {
        auto &instance = scene.instances[i];
        instance.mesh = s.entries[i].parts.mesh;
        const auto &task = s.entries[i].origin.task;
        if (s.entries[i].role == "task" && delta.count(task))
            instance.transform = (delta.at(task) * instance.transform.cast<double>()).cast<float>();
    }

    // Target frames of the sample's task, after placement jitter.
    std::map<std::string, Pose> frames;
    if (block.task)
        for (const auto &[id, pose] : s.tasks.at(*block.task).frames)
            frames[id] = spatial::compose(d.world_from_task.at(*block.task), pose);

    const auto &acc = job_.acceptance;
    const double areaScale = double(s.accept.width) * s.accept.height / (double(s.output.width) * s.output.height);
    const bool reduced = s.accept.width != s.output.width || s.accept.height != s.output.height;
    std::map<std::string, int> reasons;
    // §3.2 checks 2-4 on a full-resolution label capture; empty string = accepted.
    const auto judge = [&](const r::LabelCapture &capture, const std::vector<std::uint8_t> &self,
                           LabelStats &stats) -> std::string {
        stats = analyzeLabels(capture, float(kNearPlaneM), float(kFarPlaneM), float(acc.near_m),
                              self.empty() ? nullptr : &self);
        if (stats.counted_pixels > 0 &&
            double(stats.near_pixels) > acc.max_near_fraction * double(stats.counted_pixels))
            return "near_geometry";
        std::int64_t labelledPixels = 0, target = 0;
        for (const auto &[key, item] : stats.keys) {
            const auto found = s.keys.find(key);
            if (found == s.keys.end())
                continue;
            const auto &entry = s.entries[found->second.first];
            const auto &part = entry.parts.parts[found->second.second];
            if (!job_.labelled.count({entry.origin.task, part.part}))
                continue;
            if (item.medianDepth() > acc.max_range_m)
                return "label_too_far";
            labelledPixels += item.pixels;
            if (block.task && entry.origin.task == *block.task)
                target = std::max(target, item.pixels);
        }
        if (block.task && target < acc.min_target_px)
            return "no_target";
        if (!block.task && labelledPixels > acc.background_max_labelled_px)
            return "labelled_in_background";
        // Components for the record; the fragment rule (§10.1) last, on labelled instances only.
        measureComponents(capture, stats);
        if (acc.reject_fragments)
            for (const auto &[key, item] : stats.keys) {
                const auto found = s.keys.find(key);
                if (found == s.keys.end())
                    continue;
                const auto &entry = s.entries[found->second.first];
                if (job_.labelled.count({entry.origin.task, entry.parts.parts[found->second.second].part}) &&
                    item.fragmented(acc.min_visible_px, acc.min_fragment_px))
                    return "fragmented";
            }
        return {};
    };
    // A9 prefilter at reduced resolution: rejects only views whose target is clearly too small (below half the
    // area-scaled threshold), so the full-resolution checks decide every other case and the acceptance scale
    // changes cost, not output.
    const auto prefilter = [&](const r::LabelCapture &capture) {
        if (!block.task)
            return true;
        std::map<std::uint32_t, std::int64_t> pixels;
        for (const auto key : capture.ids)
            if (key & 0xffu)
                ++pixels[key];
        for (const auto &[key, count] : pixels) {
            const auto found = s.keys.find(key);
            if (found == s.keys.end())
                continue;
            const auto &entry = s.entries[found->second.first];
            if (entry.origin.task == *block.task &&
                job_.labelled.count({entry.origin.task, entry.parts.parts[found->second.second].part}) &&
                double(count) >= .5 * double(acc.min_target_px) * areaScale)
                return true;
        }
        return false;
    };

    double labelMs = 0;
    std::int64_t attempts = 0;
    std::optional<Pose> root, camera;
    std::string targetFrame;
    r::LabelCapture full;
    LabelStats fullStats;
    while (attempts < acc.max_attempts) {
        ++attempts;
        const auto draw = samplePose(block.sampler, rng, frames, s.pool, s.root_from_optical);
        if (!draw.world_from_root) {
            ++reasons[draw.reason];
            continue;
        }
        const Pose worldCamera = spatial::compose(*draw.world_from_root, s.root_from_optical);
        if (const auto reason = s.pool.checkCamera(worldCamera.translation)) {
            ++reasons[*reason];
            continue;
        }
        const auto worldRoot = ps::toMatrix(*draw.world_from_root);
        for (std::size_t j = 0; j < s.robot_count; ++j)
            scene.instances[s.static_count + j].transform = (worldRoot * s.robot_root_from_asset[j]).cast<float>();
        const auto labelStart = std::chrono::steady_clock::now();
        std::string reason;
        if (reduced && !prefilter(host_->captureLabels(scene, s.labels, viewFor(worldCamera, s.accept), s.accept.width,
                                                       s.accept.height)))
            reason = "no_target";
        if (reason.empty()) {
            full =
                host_->captureLabels(scene, s.labels, viewFor(worldCamera, s.output), s.output.width, s.output.height);
            reason = judge(full, s.self_output, fullStats);
        }
        labelMs += msSince(labelStart);
        if (!reason.empty()) {
            ++reasons[reason];
            continue;
        }
        root = draw.world_from_root;
        camera = worldCamera;
        targetFrame = draw.frame;
        break;
    }
    log["attempts"] = attempts;
    log["reasons"] = reasons;
    log["scenario_index"] = scenarioIndex;
    log["environment"] = environment.id;
    log["environment_index"] = environmentIndex;
    if (!root) {
        log["status"] = "skipped";
        log["timings_ms"] = {{"label", labelMs}, {"total", msSince(started)}};
        return log;
    }

    // RGB through the camera model.
    const auto rgbStart = std::chrono::steady_clock::now();
    const auto &K = s.output;
    auto image = host_->capture(scene, viewFor(*camera, K), d.appearance, d.time, K.width, K.height, true, false);
    const double rgbMs = msSince(rgbStart);
    const auto encodeStart = std::chrono::steady_clock::now();
    postProcess(image.rgb, K.width, K.height, d.blur_px, d.noise_sigma, d.noise_seed);
    cameras::Processor processor;
    const bool jpeg = job_.camera.format == "jpg";
    auto frame = processor.process(K, cameras::DepthNoise{}, image.rgb, {}, jpeg, job_.camera.jpeg_quality);
    const auto encoded = jpeg ? frame.jpeg : encodePngRgb(frame.rgb, K.width, K.height);

    // Instance table: every visible part instance, in label-key order.
    std::map<std::uint32_t, std::uint16_t> table;
    Json instances = Json::array();
    for (const auto &[key, item] : fullStats.keys) {
        const auto found = s.keys.find(key);
        if (found == s.keys.end())
            continue; // a part-map value no rule names
        if (table.size() >= 65535)
            throw std::runtime_error(name + ": more than 65535 visible instances");
        const auto id = static_cast<std::uint16_t>(table.size() + 1);
        table[key] = id;
        const auto &entry = s.entries[found->second.first];
        const auto &part = entry.parts.parts[found->second.second];
        Json indicator = nullptr;
        if (!part.indicator.empty()) {
            const auto names = s.indicator_names.find(part.indicator);
            if (names != s.indicator_names.end())
                indicator = d.latched[part.indicator] ? names->second.second : names->second.first;
        }
        const auto [lo, hi] = std::minmax_element(item.depth_m.begin(), item.depth_m.end());
        instances.push_back(
            {{"id", id},
             {"task", entry.origin.task.empty() ? Json() : Json(entry.origin.task)},
             {"part", part.part},
             {"source", entry.source},
             {"piece", part.piece},
             {"indicator", indicator},
             {"pixels", item.pixels},
             {"bbox_xywh", {item.min_x, item.min_y, item.max_x - item.min_x + 1, item.max_y - item.min_y + 1}},
             {"depth_m", {{"min", *lo}, {"median", item.medianDepth()}, {"max", *hi}}},
             {"truncated", item.truncated},
             {"components", item.components.size()},
             {"largest_component_px", item.components.empty() ? 0 : item.components.front()}});
    }
    const auto ids = encodePng16(idMap(full, table), K.width, K.height);
    const double encodeMs = msSince(encodeStart);

    Json tasks = Json::object();
    const auto cameraFromWorld = spatial::inverse(*camera);
    for (const auto &task : s.task_order)
        tasks[task] = {{"camera_from_task", poseJson(spatial::compose(cameraFromWorld, d.world_from_task.at(task)))}};
    d.record["target_frame"] = block.task ? Json(targetFrame) : Json();
    Json record = {{"format", "nereus.dataset_record.v1"},
                   {"dataset", job_.dataset},
                   {"sample", k},
                   {"name", name},
                   {"task", block.task ? Json(*block.task) : Json()},
                   {"scenario", s.id},
                   {"scenario_index", scenarioIndex},
                   {"environment", environment.id},
                   {"environment_index", environmentIndex},
                   {"attempts", attempts},
                   {"image", imageRel},
                   {"ids", idsRel},
                   {"camera",
                    {{"sensor", job_.camera.sensor},
                     {"width", K.width},
                     {"height", K.height},
                     {"fx", K.fx},
                     {"fy", K.fy},
                     {"cx", K.cx},
                     {"cy", K.cy},
                     {"world_from_camera", poseJson(*camera)}}},
                   {"robot", {{"world_from_root", poseJson(*root)}}},
                   {"tasks", tasks},
                   {"randomization", d.record},
                   {"instances", instances}};

    const auto writeStart = std::chrono::steady_clock::now();
    writeAtomic(out / imageRel, encoded);
    writeAtomic(out / idsRel, ids);
    writeAtomic(recordPath, record.dump(1) + "\n", true); // last: a record means the sample is complete
    log["status"] = "accepted";
    log["instances"] = instances.size();
    log["timings_ms"] = {{"label", labelMs},
                         {"rgb", rgbMs},
                         {"encode", encodeMs},
                         {"write", msSince(writeStart)},
                         {"total", msSince(started)}};
    return log;
}
} // namespace nereus::datasets
