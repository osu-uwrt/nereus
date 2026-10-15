// Covariance ellipsoids for the mapping estimate overlay: the principal axes of a position covariance and the
// points round the ellipses that draw it. GL-free so it is tested on its own.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <glm/glm.hpp>
#include <utility>
#include <vector>

namespace nereus::ros_viewer::host {
// The principal axes of a position covariance: axis[k] is the k-th eigenvector scaled to its 1-sigma length,
// largest first. Negative eigenvalues (round-off) count as zero.
struct CovarianceAxes {
    std::array<glm::vec3, 3> axis{};
};

// Jacobi eigen decomposition of the symmetric part of `covariance` (glm is column-major: m[column][row]).
inline CovarianceAxes principalAxes(const glm::mat3 &covariance) {
    double a[3][3], v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            a[r][c] = .5 * (double(covariance[c][r]) + double(covariance[r][c]));
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        const double scale = a[0][0] * a[0][0] + a[1][1] * a[1][1] + a[2][2] * a[2][2];
        if (off <= 1e-24 * scale || off == 0)
            break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (a[p][q] == 0)
                    continue;
                // The rotation in the p-q plane that zeroes a[p][q]
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (theta >= 0 ? 1. : -1.) / (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 3; ++k) { // A P
                    const double kp = a[k][p], kq = a[k][q];
                    a[k][p] = c * kp - s * kq;
                    a[k][q] = s * kp + c * kq;
                }
                for (int k = 0; k < 3; ++k) { // P^T (A P)
                    const double pk = a[p][k], qk = a[q][k];
                    a[p][k] = c * pk - s * qk;
                    a[q][k] = s * pk + c * qk;
                }
                for (int k = 0; k < 3; ++k) { // V P
                    const double kp = v[k][p], kq = v[k][q];
                    v[k][p] = c * kp - s * kq;
                    v[k][q] = s * kp + c * kq;
                }
            }
    }
    std::array<std::pair<double, int>, 3> order{{{a[0][0], 0}, {a[1][1], 1}, {a[2][2], 2}}};
    std::sort(order.begin(), order.end(), [](const auto &x, const auto &y) { return x.first > y.first; });
    CovarianceAxes axes;
    for (int k = 0; k < 3; ++k) {
        const int i = order[k].second;
        const double length = std::sqrt(std::max(order[k].first, 0.));
        axes.axis[k] = glm::vec3(float(v[0][i] * length), float(v[1][i] * length), float(v[2][i] * length));
    }
    return axes;
}

// `segments` + 1 points round the ellipse spanned by axes i and j at `sigmas` times their 1-sigma lengths, closed
// (the last point repeats the first).
inline std::vector<glm::vec3> ellipsePoints(const glm::vec3 &center, const CovarianceAxes &axes, int i, int j,
                                            float sigmas, int segments) {
    std::vector<glm::vec3> points;
    points.reserve(std::size_t(segments) + 1);
    for (int k = 0; k <= segments; ++k) {
        const float angle = 2.f * float(M_PI) * float(k % segments) / float(segments);
        points.push_back(center + sigmas * (std::cos(angle) * axes.axis[i] + std::sin(angle) * axes.axis[j]));
    }
    return points;
}

// The largest horizontal (x, y) 1-sigma spread: the overlay label's size.
inline float horizontalSigma(const glm::mat3 &covariance) {
    const double xx = covariance[0][0], yy = covariance[1][1], xy = .5 * (double(covariance[1][0]) + covariance[0][1]);
    const double mean = .5 * (xx + yy), spread = std::sqrt(.25 * (xx - yy) * (xx - yy) + xy * xy);
    return float(std::sqrt(std::max(mean + spread, 0.)));
}
} // namespace nereus::ros_viewer::host
