#include <nereus/datasets/job.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace nereus::datasets {
namespace {
[[noreturn]] void fail(const std::string &where, const std::string &what) {
    throw std::runtime_error("job " + where + ": " + what);
}

double number(const Json &value, const std::string &where) {
    if (!value.is_number())
        fail(where, "expected a number");
    const double result = value.get<double>();
    if (!std::isfinite(result))
        fail(where, "not finite");
    return result;
}

// [lo, hi] or a number n (= [n, n]).
Range range(const Json &value, const std::string &where) {
    if (value.is_number()) {
        const double n = number(value, where);
        return {n, n};
    }
    if (!value.is_array() || value.size() != 2)
        fail(where, "expected [lo, hi]");
    Range result{number(value.at(0), where), number(value.at(1), where)};
    if (result.hi < result.lo)
        fail(where, "hi < lo");
    return result;
}

std::optional<Range> optionalRange(const Json &parent, const char *key, const std::string &where) {
    if (!parent.is_object() || !parent.contains(key) || parent.at(key).is_null())
        return std::nullopt;
    return range(parent.at(key), where + "." + key);
}

Eigen::Vector3d vector3(const Json &value, const std::string &where) {
    if (!value.is_array() || value.size() != 3)
        fail(where, "expected [x, y, z]");
    return {number(value.at(0), where), number(value.at(1), where), number(value.at(2), where)};
}

std::optional<std::string> optionalString(const Json &parent, const char *key) {
    if (!parent.contains(key) || parent.at(key).is_null())
        return std::nullopt;
    return parent.at(key).get<std::string>();
}

std::filesystem::path canonical(const std::filesystem::path &path) {
    std::error_code error;
    auto result = std::filesystem::weakly_canonical(path, error);
    return error ? path.lexically_normal() : result;
}

Sampler parseSampler(const Json &item, const std::string &where) {
    Sampler s;
    s.type = item.at("type").get<std::string>();
    if (s.type != "approach" && s.type != "overhead" && s.type != "free")
        fail(where, "unknown sampler type '" + s.type + "'");
    if (item.contains("frame")) {
        const auto &frame = item.at("frame");
        s.frames.clear();
        if (frame.is_string())
            s.frames.push_back(frame.get<std::string>());
        else
            for (const auto &f : frame)
                s.frames.push_back(f.get<std::string>());
        if (s.frames.empty())
            fail(where, "empty frame list");
    }
    if (item.contains("offset_m"))
        s.offset = vector3(item.at("offset_m"), where + ".offset_m");
    if (item.contains("facing")) {
        s.facing = vector3(item.at("facing"), where + ".facing");
        if (Eigen::Vector2d(s.facing.x(), s.facing.y()).norm() < 1e-9)
            fail(where, "facing needs a horizontal component");
    }
    const auto get = [&](const char *key, Range &target) {
        if (item.contains(key))
            target = range(item.at(key), where + "." + key);
    };
    get("range_m", s.range_m);
    get("elevation_deg", s.elevation_deg);
    get("altitude_m", s.altitude_m);
    get("depth_m", s.depth_m);
    const auto scalar = [&](const char *key, double &target) {
        if (item.contains(key))
            target = std::abs(number(item.at(key), where + "." + key));
    };
    scalar("bearing_deg", s.bearing_deg);
    scalar("aim_jitter_deg", s.aim_jitter_deg);
    scalar("roll_deg", s.roll_deg);
    scalar("pitch_deg", s.pitch_deg);
    scalar("max_aim_pitch_deg", s.max_aim_pitch_deg);
    scalar("radius_m", s.radius_m);
    s.both_sides = item.value("both_sides", false);
    if (item.contains("yaw_deg") && !item.at("yaw_deg").is_null())
        s.yaw_deg = number(item.at("yaw_deg"), where + ".yaw_deg");
    if (s.type == "approach" && s.range_m.lo <= 0)
        fail(where, "range_m must be positive");
    return s;
}
std::array<Range, 3> rgbRange(const Json &value, const std::string &where) {
    if (!value.is_array() || value.size() != 3)
        fail(where, "expected [r, g, b] (numbers or [lo, hi])");
    return {range(value.at(0), where), range(value.at(1), where), range(value.at(2), where)};
}

Environment parseEnvironment(const Json &item, const std::string &index) {
    Environment e;
    if (item.contains("id"))
        e.id = item.at("id").get<std::string>();
    const std::string where = index + " ('" + e.id + "')";
    e.weight = item.value("weight", 1.0);
    if (!(e.weight >= 0))
        fail(where + ".weight", "must be >= 0");
    const auto water = item.value("water", Json::object()), lighting = item.value("lighting", Json::object()),
               image = item.value("image", Json::object());
    const auto w = where + ".water", l = where + ".lighting", im = where + ".image";
    e.tint_scale = optionalRange(water, "tint_scale", w);
    e.absorption_scale = optionalRange(water, "absorption_scale", w);
    e.scattering_scale = optionalRange(water, "scattering_scale", w);
    e.distance_scale_scale = optionalRange(water, "distance_scale_scale", w);
    e.scattering = optionalRange(water, "scattering", w);
    if (water.contains("tint_rgb") && !water.at("tint_rgb").is_null())
        e.tint_rgb = rgbRange(water.at("tint_rgb"), w + ".tint_rgb");
    if (water.contains("absorption_per_m_rgb") && !water.at("absorption_per_m_rgb").is_null())
        e.absorption_per_m_rgb = rgbRange(water.at("absorption_per_m_rgb"), w + ".absorption_per_m_rgb");
    e.caustics = optionalRange(lighting, "caustics", l);
    e.exposure = optionalRange(lighting, "exposure", l);
    e.sun_azimuth_deg = optionalRange(lighting, "sun_azimuth_deg", l);
    e.sun_elevation_deg = optionalRange(lighting, "sun_elevation_deg", l);
    e.glare = optionalRange(lighting, "glare", l);
    e.direct_light_scale = optionalRange(lighting, "direct_light_scale", l);
    e.ambient_light_scale = optionalRange(lighting, "ambient_light_scale", l);
    e.profile = optionalString(lighting, "profile");
    if (e.profile && *e.profile != "indoor" && *e.profile != "outdoor")
        fail(l + ".profile", "indoor or outdoor");
    e.time_s = optionalRange(item, "time_s", where);
    e.noise_sigma = optionalRange(image, "noise_sigma", im);
    e.blur_px = optionalRange(image, "blur_px", im);
    // No silent clamps: every scale and absolute physical value is non-negative, absolute tint at most 1.
    const auto nonNegative = [](const std::optional<Range> &value, const std::string &key) {
        if (value && value->lo < 0)
            fail(key, "must be >= 0");
    };
    nonNegative(e.tint_scale, w + ".tint_scale");
    nonNegative(e.absorption_scale, w + ".absorption_scale");
    nonNegative(e.scattering_scale, w + ".scattering_scale");
    nonNegative(e.distance_scale_scale, w + ".distance_scale_scale");
    nonNegative(e.scattering, w + ".scattering");
    nonNegative(e.caustics, l + ".caustics");
    nonNegative(e.exposure, l + ".exposure");
    nonNegative(e.glare, l + ".glare");
    nonNegative(e.direct_light_scale, l + ".direct_light_scale");
    nonNegative(e.ambient_light_scale, l + ".ambient_light_scale");
    nonNegative(e.noise_sigma, im + ".noise_sigma");
    nonNegative(e.blur_px, im + ".blur_px");
    for (int c = 0; c < 3; ++c) {
        if (e.tint_rgb) {
            nonNegative((*e.tint_rgb)[std::size_t(c)], w + ".tint_rgb");
            if ((*e.tint_rgb)[std::size_t(c)].hi > 1)
                fail(w + ".tint_rgb", "must be <= 1");
        }
        if (e.absorption_per_m_rgb)
            nonNegative((*e.absorption_per_m_rgb)[std::size_t(c)], w + ".absorption_per_m_rgb");
    }
    return e;
}
} // namespace

std::int64_t Job::sampleCount() const {
    std::int64_t total = 0;
    for (const auto &item : samples)
        total += item.count;
    return total;
}

const SampleBlock &Job::block(std::int64_t k) const {
    for (const auto &item : samples) {
        if (k < item.count)
            return item;
        k -= item.count;
    }
    throw std::out_of_range("sample index beyond the job");
}

std::int64_t Job::blockStart(std::int64_t k) const {
    std::int64_t start = 0;
    for (const auto &item : samples) {
        if (k < start + item.count)
            return start;
        start += item.count;
    }
    throw std::out_of_range("sample index beyond the job");
}

std::string Job::name(std::int64_t k) const {
    char digits[32];
    std::snprintf(digits, sizeof digits, "%06lld", static_cast<long long>(k));
    return block(k).task.value_or("background") + "_" + digits;
}

Job parseJob(const Json &d) {
    Job job;
    if (d.value("format", std::string()) != "nereus.dataset_job.v1")
        fail("format", "expected nereus.dataset_job.v1");
    job.dataset = d.at("dataset").get<std::string>();
    job.seed = d.at("seed").get<std::uint64_t>();
    job.output = d.at("output").get<std::string>();
    for (const auto &item : d.at("scenarios"))
        job.scenarios.emplace_back(item.at("id").get<std::string>(), item.at("resolved").get<std::string>());
    if (job.scenarios.empty())
        fail("scenarios", "empty");

    const auto &camera = d.at("camera");
    job.camera.sensor = camera.at("sensor").get<std::string>();
    if (camera.contains("resolution_px") && camera.at("resolution_px").is_array()) {
        job.camera.width = camera.at("resolution_px").at(0).get<int>();
        job.camera.height = camera.at("resolution_px").at(1).get<int>();
        if (job.camera.width < 1 || job.camera.height < 1 || job.camera.width > 4096 || job.camera.height > 4096)
            fail("camera.resolution_px", "out of range");
    }
    job.camera.crop = camera.value("crop", std::string("center"));
    if (job.camera.crop != "center")
        fail("camera.crop", "only 'center' is supported");
    job.camera.format = camera.value("format", std::string("jpg"));
    if (job.camera.format != "jpg" && job.camera.format != "png")
        fail("camera.format", "jpg or png");
    job.camera.jpeg_quality = camera.value("jpeg_quality", 92);
    if (job.camera.jpeg_quality < 0 || job.camera.jpeg_quality > 100)
        fail("camera.jpeg_quality", "0..100");
    job.camera.robot_visuals = camera.value("robot_visuals", true);

    const auto parts = d.value("parts", Json::object());
    for (const auto &item : parts.value("textures", Json::array())) {
        TexturePart part;
        part.texture = canonical(item.at("texture").get<std::string>());
        part.mask = canonical(item.at("mask").get<std::string>());
        part.task = optionalString(item, "task");
        for (const auto &[key, value] : item.at("values").items()) {
            const int v = std::stoi(key);
            if (v < 1 || v > 255)
                fail("parts.textures.values", "value " + key + " outside 1..255");
            part.values[v] = value.get<std::string>();
        }
        job.textures.push_back(std::move(part));
    }
    for (const auto &item : parts.value("visuals", Json::array())) {
        VisualPart part;
        part.task = item.at("task").get<std::string>();
        part.asset = item.at("asset").get<std::string>();
        part.prop = optionalString(item, "prop");
        part.frame = optionalString(item, "frame");
        part.part = optionalString(item, "part");
        part.indicator = optionalString(item, "indicator");
        if (item.contains("materials") && !item.at("materials").is_null())
            for (const auto &[key, value] : item.at("materials").items())
                part.materials[key] = value.get<std::string>();
        if (part.part.has_value() == !part.materials.empty())
            fail("parts.visuals", part.task + "/" + part.asset + ": exactly one of part or materials");
        const auto split = item.value("split", Json()).is_string() ? item.at("split").get<std::string>() : "none";
        if (split != "none" && split != "connected")
            fail("parts.visuals", "split must be none or connected");
        part.split = split == "connected";
        job.visuals.push_back(std::move(part));
    }
    for (const auto &item : d.value("labelled", Json::array()))
        job.labelled.emplace(item.at("task").get<std::string>(), item.at("part").get<std::string>());

    const auto acceptance = d.value("acceptance", Json::object());
    auto &a = job.acceptance;
    a.max_range_m = acceptance.value("max_range_m", a.max_range_m);
    a.min_target_px = acceptance.value("min_target_px", a.min_target_px);
    a.near_m = acceptance.value("near_m", a.near_m);
    a.max_near_fraction = acceptance.value("max_near_fraction", a.max_near_fraction);
    a.max_attempts = acceptance.value("max_attempts", a.max_attempts);
    a.background_max_labelled_px = acceptance.value("background_max_labelled_px", a.background_max_labelled_px);
    const auto fragments = acceptance.value("fragments", std::string("reject"));
    if (fragments != "reject" && fragments != "allow")
        fail("acceptance.fragments", "reject or allow");
    a.reject_fragments = fragments == "reject";
    a.min_fragment_px = acceptance.value("min_fragment_px", a.min_fragment_px);
    a.min_visible_px = acceptance.value("min_visible_px", a.min_visible_px);
    if (a.max_attempts < 1)
        fail("acceptance.max_attempts", "must be >= 1");

    std::size_t index = 0;
    for (const auto &item : d.at("samples")) {
        SampleBlock block;
        block.task = optionalString(item, "task");
        block.count = item.at("count").get<std::int64_t>();
        if (block.count < 0)
            fail("samples", "negative count");
        block.sampler = parseSampler(item.at("sampler"), "samples[" + std::to_string(index++) + "].sampler");
        job.samples.push_back(std::move(block));
    }

    const auto r = d.value("randomize", Json::object());
    auto &z = job.randomize;
    if (r.contains("environments")) {
        z.environments.clear();
        std::size_t i = 0;
        for (const auto &item : r.at("environments"))
            z.environments.push_back(parseEnvironment(item, "randomize.environments[" + std::to_string(i++) + "]"));
        if (z.environments.empty())
            fail("randomize.environments", "needs at least one environment");
    } else // pre-§10 jobs: the top-level water/lighting/image/time_s are the one environment
        z.environments = {parseEnvironment(r, "randomize")};
    const auto mode = r.value("environment_mode", std::string("weighted"));
    if (mode != "weighted" && mode != "sweep")
        fail("randomize.environment_mode", "weighted or sweep");
    z.sweep = mode == "sweep";
    double total = 0;
    for (const auto &item : z.environments)
        total += item.weight;
    if (!z.sweep && !(total > 0))
        fail("randomize.environments", "weights must sum to a positive number");
    const auto placement = r.value("placement", Json::object());
    z.task_yaw_deg = std::abs(placement.value("task_yaw_deg", 0.0));
    z.task_offset_m = std::abs(placement.value("task_offset_m", 0.0));
    std::set<std::string> grouped;
    const auto groups = placement.value("groups", Json::array());
    if (!groups.is_array())
        fail("randomize.placement.groups", "expected a list of task lists");
    for (const auto &group : groups) {
        if (!group.is_array())
            fail("randomize.placement.groups", "expected a list of task lists");
        std::vector<std::string> tasks;
        for (const auto &task : group) {
            tasks.push_back(task.get<std::string>());
            if (!grouped.insert(tasks.back()).second)
                fail("randomize.placement.groups", "task '" + tasks.back() + "' is in more than one group");
        }
        if (tasks.empty())
            fail("randomize.placement.groups", "empty group");
        z.placement_groups.push_back(std::move(tasks));
    }
    z.latched_probability = r.value("indicators", Json::object()).value("latched_probability", 0.0);
    if (z.latched_probability < 0 || z.latched_probability > 1)
        fail("randomize.indicators.latched_probability", "0..1");
    return job;
}

Job loadJob(const std::filesystem::path &path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("cannot read job " + path.string());
    Json document;
    try {
        in >> document;
    } catch (const std::exception &error) {
        throw std::runtime_error("job " + path.string() + ": " + error.what());
    }
    return parseJob(document);
}
} // namespace nereus::datasets
