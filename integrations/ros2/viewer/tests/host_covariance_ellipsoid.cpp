// Covariance ellipsoids for the mapping estimate overlay: principal axes and the ellipses drawn from them.
#include "covariance_ellipsoid.hpp"

#include <gtest/gtest.h>

#include <glm/gtc/matrix_transform.hpp>

using nereus::ros_viewer::host::ellipsePoints;
using nereus::ros_viewer::host::horizontalSigma;
using nereus::ros_viewer::host::principalAxes;

namespace {
// Rebuilds the covariance from its axes: sum of axis axis^T
glm::mat3 rebuild(const nereus::ros_viewer::host::CovarianceAxes &axes) {
    glm::mat3 out(0);
    for (const auto &a : axes.axis)
        out += glm::outerProduct(a, a);
    return out;
}

void expectNear(const glm::mat3 &a, const glm::mat3 &b, float tolerance) {
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            EXPECT_NEAR(a[c][r], b[c][r], tolerance) << "column " << c << " row " << r;
}
} // namespace

TEST(CovarianceEllipsoid, DiagonalAxesComeOutLargestFirst) {
    const glm::mat3 cov(glm::vec3(0.01f, 0, 0), glm::vec3(0, 0.09f, 0), glm::vec3(0, 0, 0.04f));
    const auto axes = principalAxes(cov);
    EXPECT_NEAR(glm::length(axes.axis[0]), 0.3f, 1e-6f);
    EXPECT_NEAR(std::abs(axes.axis[0].y), 0.3f, 1e-6f); // along y
    EXPECT_NEAR(glm::length(axes.axis[1]), 0.2f, 1e-6f);
    EXPECT_NEAR(std::abs(axes.axis[1].z), 0.2f, 1e-6f); // along z
    EXPECT_NEAR(glm::length(axes.axis[2]), 0.1f, 1e-6f);
}

TEST(CovarianceEllipsoid, RotatedCovarianceRebuildsFromItsAxes) {
    // A depth-heavy camera covariance (along the ray) seen from a yawed, pitched camera
    const glm::mat3 local(glm::vec3(0.0004f, 0, 0), glm::vec3(0, 0.0009f, 0), glm::vec3(0, 0, 0.0400f));
    const glm::mat3 rotation(glm::rotate(glm::rotate(glm::mat4(1), 0.7f, glm::vec3(0, 0, 1)), -0.3f, glm::vec3(0, 1, 0)));
    const glm::mat3 cov = rotation * local * glm::transpose(rotation);
    const auto axes = principalAxes(cov);
    expectNear(rebuild(axes), cov, 1e-7f);
    EXPECT_NEAR(glm::length(axes.axis[0]), 0.2f, 1e-5f);
    // The long axis is the rotated z
    EXPECT_NEAR(std::abs(glm::dot(glm::normalize(axes.axis[0]), rotation[2])), 1.f, 1e-5f);
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            EXPECT_NEAR(glm::dot(axes.axis[i], axes.axis[j]), 0.f, 1e-7f); // orthogonal
}

TEST(CovarianceEllipsoid, DegenerateAndNegativeRoundOff) {
    const auto zero = principalAxes(glm::mat3(0));
    for (const auto &a : zero.axis)
        EXPECT_EQ(glm::length(a), 0.f);
    // A tiny negative eigenvalue from round-off counts as zero instead of producing NaN
    const glm::mat3 cov(glm::vec3(0.04f, 0, 0), glm::vec3(0, 0.01f, 0), glm::vec3(0, 0, -1e-9f));
    const auto axes = principalAxes(cov);
    for (const auto &a : axes.axis)
        EXPECT_TRUE(std::isfinite(glm::length(a)));
    EXPECT_EQ(glm::length(axes.axis[2]), 0.f);
}

TEST(CovarianceEllipsoid, EllipseIsClosedAtSigmaTimesTheAxes) {
    const glm::mat3 cov(glm::vec3(0.04f, 0, 0), glm::vec3(0, 0.01f, 0), glm::vec3(0, 0, 0.0025f));
    const auto axes = principalAxes(cov);
    const glm::vec3 center(1, 2, -1);
    const auto points = ellipsePoints(center, axes, 0, 1, 2.f, 32);
    ASSERT_EQ(points.size(), 33u);
    EXPECT_EQ(points.front(), points.back());
    float widest = 0, narrowest = 1e9f;
    for (const auto &p : points) {
        EXPECT_NEAR(p.z, center.z, 1e-6f); // the x-y ellipse
        widest = std::max(widest, glm::length(p - center));
        narrowest = std::min(narrowest, glm::length(p - center));
    }
    EXPECT_NEAR(widest, 2 * 0.2f, 1e-5f);
    EXPECT_NEAR(narrowest, 2 * 0.1f, 1e-5f);
}

TEST(CovarianceEllipsoid, HorizontalSigmaIgnoresDepth) {
    EXPECT_NEAR(horizontalSigma(glm::mat3(glm::vec3(0.04f, 0, 0), glm::vec3(0, 0.01f, 0), glm::vec3(0, 0, 9.f))), 0.2f,
                1e-6f);
    // Correlated x-y: the major axis of the 2x2 block
    const glm::mat3 cov(glm::vec3(0.02f, 0.01f, 0), glm::vec3(0.01f, 0.02f, 0), glm::vec3(0, 0, 0));
    EXPECT_NEAR(horizontalSigma(cov), std::sqrt(0.03f), 1e-6f);
}
