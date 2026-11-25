// Empirical stereo depth noise model. Parameters and pixels are caller-owned.
#include "depth_noise.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace nereus::cameras {

void DepthNoise::validate() const {
    for (double value : {base_sigma,  range_sigma,      exponent,         min_range,         max_range,
                         bias,        dropout,          range_dropout,    edge_dropout,      outliers,
                         correlation, wave_sigma,       wave_range_sigma, wave_correlation,  tilt_sigma,
                         scale_sigma, tilt_correlation, edge_smooth,      edge_band_dropout, edge_jump})
        if (!std::isfinite(value))
            throw std::invalid_argument("camera depth settings must be finite");
    if (base_sigma < 0 || range_sigma < 0 || exponent < 0 || exponent > 4 || min_range <= 0 || max_range <= min_range ||
        max_range > 100 || patch_size < 1 || patch_size > 64)
        throw std::invalid_argument("invalid camera depth noise or range settings");
    for (double value : {dropout, range_dropout, edge_dropout, outliers, correlation})
        if (value < 0 || value > 1)
            throw std::invalid_argument("camera depth probabilities must be in [0, 1]");
    if (wave_sigma < 0 || wave_range_sigma < 0 || tilt_sigma < 0 || scale_sigma < 0 || wave_correlation < 0 ||
        wave_correlation >= 1 || tilt_correlation < 0 || tilt_correlation >= 1 || wave_patch < 2 || wave_patch > 4096 ||
        edge_fatten < 0 || edge_fatten > 64 || edge_smooth < 0 || edge_smooth > 64 || edge_band_dropout < 0 ||
        edge_band_dropout > 1 || edge_jump <= 0)
        throw std::invalid_argument("invalid camera structured depth noise settings");
}

namespace detail {

// Range-gates, drops out, perturbs and adds outliers to top-down metric depth in place.
// All randomness comes from `random`, so a copied generator replays the same frame.
void applyNoise(const DepthNoise &p, std::vector<float> &values, int width, int height, std::mt19937 &random) {
    cv::Mat depth(height, width, CV_32FC1, values.data());
    const cv::Mat truth = depth.clone(); // Unperturbed copy for edge detection.
    std::normal_distribution<float> normal(0, 1);
    cv::Mat patches;
    std::vector<double> normalize_x, normalize_y;

    // Mix weights so independent + shared noise keeps unit variance.
    const double independent_weight = std::sqrt(1 - p.correlation);
    const double shared_weight = std::sqrt(p.correlation);

    // Spatially correlated noise: a coarse grid of unit normals bilinearly upsampled to full size.
    if (p.enabled && p.correlation > 0) {
        cv::Mat coarse((height + p.patch_size - 1) / p.patch_size + 1, (width + p.patch_size - 1) / p.patch_size + 1,
                       CV_32FC1);
        for (int y = 0; y < coarse.rows; ++y)
            for (int x = 0; x < coarse.cols; ++x)
                coarse.at<float>(y, x) = normal(random);
        cv::resize(coarse, patches, depth.size(), 0, 0, cv::INTER_LINEAR);

        // Bilinear blending shrinks variance between grid nodes; these per-row/column factors
        // rescale each interpolated sample back to unit variance.
        const auto normalization = [](int size, int coarse_size) {
            std::vector<double> factors(size);
            for (int i = 0; i < size; ++i) {
                const double s = (i + .5) * coarse_size / size - .5;
                const double f = s < 0 || s >= coarse_size - 1 ? 0 : s - std::floor(s);
                factors[i] = 1. / std::sqrt(f * f + (1 - f) * (1 - f));
            }
            return factors;
        };
        normalize_x = normalization(width, coarse.cols);
        normalize_y = normalization(height, coarse.rows);
    }

    // Per-pixel draws use a fast OpenCV RNG seeded once from the camera's stream.
    cv::RNG pixel_random(p.enabled ? random() : 0);
    cv::Mat independent, probabilities;
    if (p.enabled) {
        independent.create(1, width, CV_32FC1);
        probabilities.create(1, width, CV_32FC2);
    }

    for (int y = 0; y < height; ++y) {
        // One row of independent normals and two uniforms per pixel (dropout, outlier).
        if (p.enabled) {
            pixel_random.fill(independent, cv::RNG::NORMAL, 0., 1.);
            pixel_random.fill(probabilities, cv::RNG::UNIFORM, 0., 1.);
        }
        for (int x = 0; x < width; ++x) {
            float &z = depth.at<float>(y, x);
            if (!std::isfinite(z) || z < p.min_range || z > p.max_range) {
                z = NAN;
                continue;
            }
            if (!p.enabled)
                continue;

            // A pixel is on an edge if any 4-neighbor is invalid or jumps by more than 5 cm + 3 % (only needed when
            // edges drop out).
            bool edge = false;
            if (p.edge_dropout > 0)
                for (const cv::Point &offset : {cv::Point(-1, 0), cv::Point(1, 0), cv::Point(0, -1), cv::Point(0, 1)}) {
                    const int u = x + offset.x, v = y + offset.y;
                    if (u < 0 || v < 0 || u >= width || v >= height)
                        continue;
                    const float adjacent = truth.at<float>(v, u);
                    if (!std::isfinite(adjacent) || std::abs(adjacent - z) > .05 + .03 * z)
                        edge = true;
                }

            // Dropout.
            const double probability = std::clamp(
                p.dropout + p.range_dropout * std::pow(z / p.max_range, 2) + (edge ? p.edge_dropout : 0), 0., 1.);
            const auto chance = probabilities.at<cv::Vec2f>(0, x);
            if (chance[0] < probability) {
                z = NAN;
                continue;
            }

            // Range-dependent Gaussian error mixing independent and patch-correlated noise.
            float shared = 0;
            if (!patches.empty())
                shared = patches.at<float>(y, x) * normalize_x[x] * normalize_y[y];
            const double perturbation = independent_weight * independent.at<float>(0, x) + shared_weight * shared;
            const float original = z;
            const double distance = z;
            const double sigma =
                p.base_sigma + p.range_sigma * (p.exponent == 2 ? distance * distance : std::pow(distance, p.exponent));
            z += p.bias + sigma * perturbation;

            // Rare gross outliers of up to +/-25 % of the true depth.
            if (chance[1] < p.outliers)
                z += pixel_random.uniform(-.25f, .25f) * original;
            if (!std::isfinite(z) || z < p.min_range || z > p.max_range)
                z = NAN;
        }
    }
}

void applyStructuredNoise(const DepthNoise &p, std::vector<float> &values, int width, int height, double fx, double fy,
                          double cx, double cy, std::mt19937 &random, NoiseState &state) {
    const bool waves = p.wave_sigma > 0 || p.wave_range_sigma > 0;
    const bool plane = p.tilt_sigma > 0 || p.scale_sigma > 0;
    if (!waves && !plane && p.edge_fatten <= 0 && p.edge_smooth <= 0)
        return;
    cv::Mat depth(height, width, CV_32FC1, values.data());
    std::normal_distribution<double> normal(0, 1);

    // Edge smearing (and its dropout band) around depth jumps, from the unperturbed depth. The jump search and the
    // valid-weighted blur run at 1/factor resolution (the smear is smooth); full-resolution pixels sample them back.
    if (p.edge_smooth > 0) {
        const int factor = std::clamp(static_cast<int>(std::ceil(p.edge_smooth / 1.5)), 2, 8);
        const cv::Size small((width + factor - 1) / factor, (height + factor - 1) / factor);
        const cv::Mat valid = depth == depth;
        cv::Mat valid_f, filled, high, low;
        valid.convertTo(valid_f, CV_32F, 1. / 255);
        depth.copyTo(filled);
        filled.setTo(0.f, ~valid);
        depth.copyTo(high);
        high.setTo(-1e3f, ~valid);
        depth.copyTo(low);
        low.setTo(1e3f, ~valid);
        cv::Mat weight_s, sum_s, high_s, low_s;
        cv::resize(valid_f, weight_s, small, 0, 0, cv::INTER_AREA);
        cv::resize(filled, sum_s, small, 0, 0, cv::INTER_AREA);
        if (factor > 1) {
            // max/min pooling to the small grid keeps every jump
            const cv::Mat pool = cv::Mat::ones(factor, factor, CV_8U);
            cv::dilate(high, high, pool, cv::Point(0, 0));
            cv::erode(low, low, pool, cv::Point(0, 0));
        }
        cv::resize(high, high_s, small, 0, 0, cv::INTER_NEAREST);
        cv::resize(low, low_s, small, 0, 0, cv::INTER_NEAREST);
        const int radius = std::max(1, static_cast<int>(std::ceil(2 * p.edge_smooth / factor)));
        const cv::Mat kernel = cv::Mat::ones(2 * radius + 1, 2 * radius + 1, CV_8U);
        cv::dilate(high_s, high_s, kernel);
        cv::erode(low_s, low_s, kernel);
        const double sigma_s = p.edge_smooth / factor;
        cv::GaussianBlur(weight_s, weight_s, cv::Size(0, 0), sigma_s);
        cv::GaussianBlur(sum_s, sum_s, cv::Size(0, 0), sigma_s);
        std::uniform_real_distribution<float> uniform(0.f, 1.f);
        const float scale = 1.f / factor;
        for (int y = 0; y < height; ++y) {
            const int sy = std::min(y / factor, small.height - 1);
            for (int x = 0; x < width; ++x) {
                float &z = depth.at<float>(y, x);
                const int sx = std::min(x / factor, small.width - 1);
                const float hi = high_s.at<float>(sy, sx), lo = low_s.at<float>(sy, sx);
                if (!std::isfinite(z) || hi < 0 || lo > 1e2f || hi - lo <= p.edge_jump)
                    continue;
                if (p.edge_band_dropout > 0 && uniform(random) < p.edge_band_dropout) {
                    z = NAN;
                    continue;
                }
                // bilinear sample of the small blurred grids at this pixel's center
                const float fx = std::clamp((x + .5f) * scale - .5f, 0.f, small.width - 1.f);
                const float fy = std::clamp((y + .5f) * scale - .5f, 0.f, small.height - 1.f);
                const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
                const int x1 = std::min(x0 + 1, small.width - 1), y1 = std::min(y0 + 1, small.height - 1);
                const float ax = fx - x0, ay = fy - y0;
                const auto lerp = [&](const cv::Mat &m) {
                    return (1 - ay) * ((1 - ax) * m.at<float>(y0, x0) + ax * m.at<float>(y0, x1)) +
                           ay * ((1 - ax) * m.at<float>(y1, x0) + ax * m.at<float>(y1, x1));
                };
                z = lerp(sum_s) / std::max(lerp(weight_s), 1e-6f);
            }
        }
    }

    // Foreground fattening: a pixel whose neighborhood holds a surface >5 cm nearer takes that nearer depth.
    if (p.edge_fatten > 0) {
        const cv::Mat valid = depth == depth; // false for NaN
        cv::Mat nearest;
        depth.copyTo(nearest);
        nearest.setTo(1e6f, ~valid);
        const cv::Mat kernel = cv::Mat::ones(2 * p.edge_fatten + 1, 2 * p.edge_fatten + 1, CV_8U);
        cv::erode(nearest, nearest, kernel);
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                float &z = depth.at<float>(y, x);
                const float n = nearest.at<float>(y, x);
                if (std::isfinite(z) && n < 1e5f && z - n > .05f)
                    z = n;
            }
    }

    // Waves: a coarse grid of normals, carried between frames with wave_correlation, upsampled with bicubic
    // interpolation and normalized to unit variance over the frame.
    cv::Mat field;
    if (waves) {
        const double spacing = std::max(2, p.wave_patch);
        const int rows = static_cast<int>(std::ceil(height / spacing)) + 4,
                  cols = static_cast<int>(std::ceil(width / spacing)) + 4;
        const bool carry = state.rows == rows && state.cols == cols && p.wave_correlation > 0;
        if (!carry) {
            state.waves.assign(static_cast<std::size_t>(rows) * cols, 0.f);
            state.rows = rows;
            state.cols = cols;
        }
        const double keep = carry ? p.wave_correlation : 0, fresh = std::sqrt(1 - keep * keep);
        for (float &w : state.waves)
            w = static_cast<float>(keep * w + fresh * normal(random));
        const cv::Mat coarse(rows, cols, CV_32FC1, state.waves.data());
        cv::Mat up;
        cv::resize(
            coarse, up,
            cv::Size(static_cast<int>(std::lround(cols * spacing)), static_cast<int>(std::lround(rows * spacing))), 0,
            0, cv::INTER_CUBIC);
        const int offset = static_cast<int>(std::lround(spacing));
        field = up(cv::Rect(offset, offset, width, height)).clone();
        cv::Scalar mean, deviation;
        cv::meanStdDev(field, mean, deviation);
        field = (field - mean[0]) / std::max(deviation[0], 1e-9);
    }

    // Per-frame plane error, carried between frames with tilt_correlation.
    if (plane) {
        const double keep = p.tilt_correlation, fresh = std::sqrt(1 - keep * keep);
        state.tilt_x = keep * state.tilt_x + fresh * p.tilt_sigma * normal(random);
        state.tilt_y = keep * state.tilt_y + fresh * p.tilt_sigma * normal(random);
        state.scale = keep * state.scale + fresh * p.scale_sigma * normal(random);
    }

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) {
            float &z = depth.at<float>(y, x);
            if (!std::isfinite(z))
                continue;
            double value = z;
            if (waves)
                value += (p.wave_sigma + p.wave_range_sigma * value) * field.at<float>(y, x);
            if (plane)
                value *= 1 + state.scale + state.tilt_x * (x - cx) / fx + state.tilt_y * (y - cy) / fy;
            z = static_cast<float>(value);
        }
}

} // namespace detail
} // namespace nereus::cameras
