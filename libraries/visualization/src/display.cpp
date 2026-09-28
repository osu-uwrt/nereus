#include <robotics/visualization/display.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robotics::visualization {
void validate(const DisplaySettings &settings) {
    if (settings.id.empty() || settings.type.empty() || settings.history_limit < 1 ||
        settings.history_limit > 10000 || settings.max_age_ns < 0 ||
        !std::isfinite(settings.scale) || settings.scale <= 0 || settings.scale > 10000 ||
        std::any_of(settings.color.begin(), settings.color.end(),
                    [](int value) { return value < 0 || value > 255; }))
        throw std::invalid_argument("invalid display identity, history, age, scale, or color");
}
void Displays::add(std::string name, Display display) {
    if (name.empty() || !display || !displays_.emplace(std::move(name), std::move(display)).second)
        throw std::invalid_argument("invalid or duplicate display registration");
}
DisplayResult Displays::draw(const DisplayContext &context) const {
    validate(context.settings);
    if (!context.settings.enabled)
        return {{}, Level::ready, "Hidden"};
    const auto found = displays_.find(context.settings.type);
    if (found == displays_.end())
        return {{}, Level::error, "Unknown display: " + context.settings.type};
    if (!context.settings.source.empty() &&
        (!context.source || context.source->id != context.settings.source || !context.source->data))
        return {{}, Level::warning, "Source unavailable"};
    return found->second(context);
}
namespace {
void axes(DisplayResult &result, const Pose &pose, double scale) {
    const std::array<Color, 3> colors{{{245, 96, 100}, {102, 217, 144}, {101, 162, 255}}};
    for (int i = 0; i < 3; ++i)
        result.lines.push_back({pose.translation, apply(pose, Eigen::Vector3d::Unit(i) * scale),
                                colors[static_cast<std::size_t>(i)]});
}
const PoseHistory *stream(const DisplayContext &context) {
    if (context.settings.source.empty() || !context.source || !context.source->data)
        return nullptr;
    const auto found = context.source->data->streams.find(context.settings.stream);
    return found == context.source->data->streams.end() ? nullptr : &found->second;
}
Lookup placement(const DisplayContext &context, const PoseSample &sample) {
    const auto transform =
        context.source->data->frames->lookup(context.fixed_frame, sample.frame, sample.time_ns);
    return transform.pose ? Lookup{compose(*transform.pose, sample.pose), {}} : transform;
}
DisplayResult poseDisplay(const DisplayContext &context) {
    const auto *samples = stream(context);
    if (!samples)
        return {{}, Level::warning, "Pose stream unavailable"};
    auto end = std::upper_bound(samples->begin(), samples->end(), context.source->time_ns,
                                [](Time t, const PoseSample &s) { return t < s.time_ns; });
    if (end == samples->begin())
        return {{}, Level::warning, "No pose at presentation time"};
    const auto &sample = *std::prev(end);
    if (context.source->time_ns - sample.time_ns > context.settings.max_age_ns)
        return {{}, Level::warning, "Stale pose (hidden)"};
    const auto placed = placement(context, sample);
    if (!placed.pose)
        return {{}, Level::warning, placed.issue};
    DisplayResult result;
    axes(result, *placed.pose, context.settings.scale * 2);
    const double size = context.settings.scale;
    for (int corner = 0; corner < 8; ++corner) {
        const Eigen::Vector3d point((corner & 1) ? size : -size,
                                    (corner & 2) ? size / 2 : -size / 2,
                                    (corner & 4) ? size / 4 : -size / 4);
        for (int axis = 0; axis < 3; ++axis) {
            if ((corner & (1 << axis)) != 0)
                continue;
            auto next = point;
            next[axis] = -next[axis];
            result.lines.push_back(
                {apply(*placed.pose, point), apply(*placed.pose, next), context.settings.color});
        }
    }
    return result;
}
DisplayResult trajectoryDisplay(const DisplayContext &context) {
    const auto *samples = stream(context);
    if (!samples)
        return {{}, Level::warning, "Pose stream unavailable"};
    auto end = std::upper_bound(samples->begin(), samples->end(), context.source->time_ns,
                                [](Time t, const PoseSample &s) { return t < s.time_ns; });
    const auto count = static_cast<std::size_t>(end - samples->begin());
    auto begin = end - static_cast<std::ptrdiff_t>(std::min(count, context.settings.history_limit));
    DisplayResult result;
    std::optional<Eigen::Vector3d> previous;
    for (auto it = begin; it != end; ++it) {
        const auto placed = placement(context, *it);
        if (!placed.pose) {
            previous.reset(); // Do not bridge transform gaps with an invented segment.
            result.level = Level::warning;
            result.status = placed.issue;
            continue;
        }
        if (previous)
            result.lines.push_back({*previous, placed.pose->translation, context.settings.color});
        previous = placed.pose->translation;
    }
    if (count == 0) {
        result.level = Level::warning;
        result.status = "No trajectory at presentation time";
    } else if (count > context.settings.history_limit && result.level == Level::ready) {
        result.status =
            "History limited to " + std::to_string(context.settings.history_limit) + " samples";
    }
    return result;
}
} // namespace
Displays standardDisplays() {
    Displays result;
    result.add("grid", [](const DisplayContext &context) {
        DisplayResult result;
        for (int i = -10; i <= 10; ++i) {
            const double x = static_cast<double>(i);
            result.lines.push_back({{x, -10, 0}, {x, 10, 0}, context.settings.color});
            result.lines.push_back({{-10, x, 0}, {10, x, 0}, context.settings.color});
        }
        axes(result, Pose{}, 1.0);
        return result;
    });
    result.add("axes", [](const DisplayContext &context) -> DisplayResult {
        if (context.settings.source.empty() || !context.source || !context.source->data)
            return {{}, Level::warning, "Frame source unavailable"};
        const auto placed = context.source->data->frames->lookup(
            context.fixed_frame, context.settings.frame, context.source->time_ns);
        if (!placed.pose)
            return {{}, Level::warning, placed.issue};
        DisplayResult result;
        axes(result, *placed.pose, context.settings.scale);
        return result;
    });
    result.add("pose", poseDisplay);
    result.add("trajectory", trajectoryDisplay);
    return result;
}
} // namespace robotics::visualization
