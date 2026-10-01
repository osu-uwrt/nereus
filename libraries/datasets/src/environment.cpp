#include <nereus/datasets/environment.hpp>

#include <algorithm>

namespace nereus::datasets {
std::size_t selectEnvironment(const Job &job, std::int64_t k, Stream &rng) {
    const auto &list = job.randomize.environments;
    if (const auto forced = job.block(k).environment) // §11.1: the block's environment, no draw
        return *forced;
    // Sweep: scenario = k mod S, so stepping the environment every S samples gives every scenario every
    // environment.
    if (job.randomize.sweep)
        return static_cast<std::size_t>(((k - job.blockStart(k)) / static_cast<std::int64_t>(job.scenarios.size())) %
                                        static_cast<std::int64_t>(list.size()));
    double total = 0;
    for (const auto &item : list)
        total += item.weight;
    const double u = rng.uniform() * total;
    double cumulative = 0;
    for (std::size_t i = 0; i < list.size(); ++i) {
        cumulative += list[i].weight;
        if (u < cumulative)
            return i;
    }
    // Rounding at the top end: the last environment with a nonzero weight.
    for (std::size_t i = list.size(); i-- > 0;)
        if (list[i].weight > 0)
            return i;
    return list.size() - 1;
}

EnvironmentDraw drawEnvironment(const Environment &e, const rendering::Appearance &pool, Stream &rng) {
    EnvironmentDraw d;
    d.appearance = pool;
    auto &a = d.appearance;
    auto &water = a.water;
    const auto draw = [&](const std::optional<Range> &range) { return float(rng.uniform(*range)); };
    if (e.tint_rgb)
        for (int c = 0; c < 3; ++c)
            water.tint[c] = float(rng.uniform((*e.tint_rgb)[std::size_t(c)])); // validated to [0, 1]
    else if (e.tint_scale)
        for (int c = 0; c < 3; ++c)
            water.tint[c] = std::clamp(water.tint[c] * draw(e.tint_scale), 0.f, 1.f);
    if (e.absorption_per_m_rgb)
        for (int c = 0; c < 3; ++c)
            water.absorption[c] = float(rng.uniform((*e.absorption_per_m_rgb)[std::size_t(c)]));
    else if (e.absorption_scale)
        water.absorption *= draw(e.absorption_scale);
    if (e.scattering)
        water.scattering = draw(e.scattering);
    else if (e.scattering_scale)
        water.scattering *= draw(e.scattering_scale);
    if (e.distance_scale_scale)
        water.distance_scale *= draw(e.distance_scale_scale);
    if (e.caustics)
        a.caustics = draw(e.caustics);
    if (e.exposure)
        a.exposure = draw(e.exposure);
    if (e.direct_light_scale)
        a.direct_light *= draw(e.direct_light_scale);
    if (e.ambient_light_scale)
        a.ambient_light *= draw(e.ambient_light_scale);
    if (e.sun_azimuth_deg)
        a.sun_azimuth = draw(e.sun_azimuth_deg);
    if (e.sun_elevation_deg)
        a.sun_elevation = draw(e.sun_elevation_deg);
    if (e.glare)
        a.glare = draw(e.glare);
    if (e.profile)
        a.outdoor = *e.profile == "outdoor";
    d.time = e.time_s ? draw(e.time_s) : 0.f;
    d.noise_sigma = e.noise_sigma ? rng.uniform(*e.noise_sigma) : 0.0;
    d.blur_px = e.blur_px ? rng.uniform(*e.blur_px) : 0.0;
    const auto vec = [](const Eigen::Vector3f &v) { return Json{v.x(), v.y(), v.z()}; };
    d.record = {{"water",
                 {{"tint_rgb", vec(water.tint)},
                  {"absorption_per_m_rgb", vec(water.absorption)},
                  {"scattering", water.scattering},
                  {"distance_scale", water.distance_scale}}},
                {"lighting",
                 {{"profile", a.outdoor ? "outdoor" : "indoor"},
                  {"caustics", a.caustics},
                  {"exposure", a.exposure},
                  {"direct_light", a.direct_light},
                  {"ambient_light", a.ambient_light},
                  {"sun_azimuth_deg", a.sun_azimuth},
                  {"sun_elevation_deg", a.sun_elevation},
                  {"glare", a.glare}}},
                {"time_s", d.time},
                {"image", {{"noise_sigma", d.noise_sigma}, {"blur_px", d.blur_px}}}};
    return d;
}
} // namespace nereus::datasets
