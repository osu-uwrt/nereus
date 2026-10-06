#pragma once
// Appearance environments (§10.2): per-sample selection and the drawn appearance relative to the pool pack.
#include <nereus/datasets/job.hpp>
#include <nereus/datasets/sampling.hpp>
#include <nereus/rendering/scene.hpp>

namespace nereus::datasets {
// A block's `environment` index wins (no draw). Weighted mode: the sample stream's FIRST draw picks by cumulative
// weight. Sweep mode:
// ((k - block start) / scenarios) mod n, no draw. Call first on a fresh stream.
std::size_t selectEnvironment(const Job &, std::int64_t k, Stream &);

// One sample's drawn environment: the appearance to render with plus the image post-processing it asks for.
struct EnvironmentDraw {
    rendering::Appearance appearance;
    float time = 0;                      // render time passed to capture(), seconds
    double noise_sigma = 0, blur_px = 0; // postProcess() inputs: RGB noise (8-bit units), Gaussian blur sigma (px)
    Json record;                         // water, lighting, time_s, image: the values drawn
};

// Draws an environment's values on top of the pool pack's appearance: water scales (tint per channel, absorption,
// scattering, distance scale) unless absolute overrides are given; lighting absolute values and scales; profile.
EnvironmentDraw drawEnvironment(const Environment &, const rendering::Appearance &pool, Stream &);
} // namespace nereus::datasets
