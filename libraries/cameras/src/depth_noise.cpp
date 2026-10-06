// Empirical stereo depth noise model. Parameters and pixels are caller-owned.
#include "depth_noise.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace nereus::cameras {

void DepthNoise::validate() const {
    for (double value : {base_sigma, range_sigma, exponent, min_range, max_range, bias, dropout, range_dropout,
                         edge_dropout, outliers, correlation})
        if (!std::isfinite(value))
            throw std::invalid_argument("camera depth settings must be finite");
    if (base_sigma < 0 || range_sigma < 0 || exponent < 0 || exponent > 4 || min_range <= 0 || max_range <= min_range ||
        max_range > 100 || patch_size < 1 || patch_size > 64)
        throw std::invalid_argument("invalid camera depth noise or range settings");
    for (double value : {dropout, range_dropout, edge_dropout, outliers, correlation})
        if (value < 0 || value > 1)
            throw std::invalid_argument("camera depth probabilities must be in [0, 1]");
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

            // A pixel is on an edge if any 4-neighbour is invalid or jumps by more than 5 cm + 3 %.
            bool edge = false;
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

} // namespace detail
} // namespace nereus::cameras
