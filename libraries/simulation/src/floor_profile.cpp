#include <nereus/simulation/floor_profile.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace nereus::simulation {
namespace {
void require(bool condition, const std::string &message) {
    if (!condition)
        throw std::invalid_argument("floor profile: " + message);
}

double sign(double value) {
    return (value > 0) - (value < 0);
}

// Fritsch-Carlson tangents: zero at local extrema and next to flat intervals, weighted harmonic
// means elsewhere, shape-preserving three-point estimates at the ends (as scipy's PchipInterpolator).
std::vector<double> tangents(const std::vector<Eigen::Vector2d> &p) {
    const std::size_t n = p.size();
    std::vector<double> h(n - 1), d(n - 1), m(n, 0.0);
    for (std::size_t k = 0; k + 1 < n; ++k) {
        h[k] = p[k + 1].x() - p[k].x();
        d[k] = (p[k + 1].y() - p[k].y()) / h[k];
    }
    if (n == 2) {
        m[0] = m[1] = d[0];
        return m;
    }
    for (std::size_t k = 1; k + 1 < n; ++k) {
        if (d[k - 1] * d[k] <= 0)
            continue;
        const double w1 = 2 * h[k] + h[k - 1], w2 = h[k] + 2 * h[k - 1];
        m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k]);
    }
    const auto edge = [](double h0, double h1, double d0, double d1) {
        double t = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
        if (sign(t) != sign(d0))
            t = 0;
        else if (sign(d0) != sign(d1) && std::abs(t) > 3 * std::abs(d0))
            t = 3 * d0;
        return t;
    };
    m[0] = edge(h[0], h[1], d[0], d[1]);
    m[n - 1] = edge(h[n - 2], h[n - 3], d[n - 2], d[n - 3]);
    return m;
}

// Drop vertices that lie within `tolerance` (in depth) of the chord between their kept neighbours. Vertices
// flagged in `keep` (the control points) only go when the chord passes exactly through them.
std::vector<Eigen::Vector2d> merged(const std::vector<Eigen::Vector2d> &samples, const std::vector<bool> &keep,
                                    double tolerance) {
    std::vector<Eigen::Vector2d> out{samples.front()};
    std::size_t start = 0;
    for (std::size_t end = 2; end < samples.size(); ++end) {
        const Eigen::Vector2d &a = samples[start], &b = samples[end];
        bool fits = true;
        for (std::size_t k = start + 1; k < end && fits; ++k) {
            const double chord = a.y() + (b.y() - a.y()) * (samples[k].x() - a.x()) / (b.x() - a.x());
            fits = std::abs(samples[k].y() - chord) <= (keep[k] ? 1e-12 : tolerance);
        }
        if (!fits) {
            out.push_back(samples[end - 1]);
            start = end - 1;
        }
    }
    out.push_back(samples.back());
    return out;
}
} // namespace

FloorProfile FloorProfile::flat(double depth, double extent, Axis axis) {
    require(std::isfinite(depth) && depth > 0 && std::isfinite(extent) && extent > 0,
            "flat floor needs a positive depth and extent");
    FloorProfile profile;
    profile.axis_ = axis;
    profile.polyline_ = {{0, depth}, {extent, depth}};
    return profile;
}

FloorProfile FloorProfile::smooth(Axis axis, const std::vector<Eigen::Vector2d> &points, double step,
                                  double tolerance) {
    require(points.size() >= 2, "needs at least two points");
    require(std::isfinite(step) && step > 0 && std::isfinite(tolerance) && tolerance >= 0,
            "step must be positive and tolerance non-negative");
    require(points.front().x() == 0, "first point must be at position 0");
    for (std::size_t k = 0; k < points.size(); ++k) {
        require(points[k].allFinite() && points[k].y() > 0, "depths must be positive and finite");
        require(k == 0 || points[k].x() > points[k - 1].x(), "positions must strictly increase");
    }
    const auto m = tangents(points);
    std::vector<Eigen::Vector2d> samples{points.front()};
    std::vector<bool> keep{true};
    for (std::size_t k = 0; k + 1 < points.size(); ++k) {
        const double x0 = points[k].x(), h = points[k + 1].x() - x0, y0 = points[k].y(), y1 = points[k + 1].y();
        const int count = std::max(1, static_cast<int>(std::ceil(h / step)));
        for (int i = 1; i <= count; ++i) {
            if (i == count) {
                samples.push_back(points[k + 1]);
                keep.push_back(true);
                break;
            }
            const double t = static_cast<double>(i) / count, t2 = t * t, t3 = t2 * t;
            const double y = (2 * t3 - 3 * t2 + 1) * y0 + (t3 - 2 * t2 + t) * h * m[k] + (-2 * t3 + 3 * t2) * y1 +
                             (t3 - t2) * h * m[k + 1];
            samples.push_back({x0 + t * h, y});
            keep.push_back(false);
        }
    }
    FloorProfile profile;
    profile.axis_ = axis;
    profile.polyline_ = merged(samples, keep, tolerance);
    return profile;
}

double FloorProfile::minDepth() const {
    double out = std::numeric_limits<double>::infinity();
    for (const auto &v : polyline_)
        out = std::min(out, v.y());
    return out;
}

double FloorProfile::maxDepth() const {
    double out = 0;
    for (const auto &v : polyline_)
        out = std::max(out, v.y());
    return out;
}

double FloorProfile::depthAt(double position) const {
    require(!empty(), "empty profile has no depth");
    if (!(position > polyline_.front().x()))
        return polyline_.front().y();
    if (position >= polyline_.back().x())
        return polyline_.back().y();
    const auto upper = std::upper_bound(polyline_.begin(), polyline_.end(), position,
                                        [](double s, const Eigen::Vector2d &v) { return s < v.x(); });
    const Eigen::Vector2d &a = *(upper - 1), &b = *upper;
    return a.y() + (b.y() - a.y()) * (position - a.x()) / (b.x() - a.x());
}

std::optional<double> FloorProfile::rayDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                                                double surface_z) const {
    require(!empty(), "empty profile has no floor");
    if (direction.z() >= 0)
        return std::nullopt;
    const double os = axis_ == Axis::X ? origin.x() : origin.y();
    const double ds = axis_ == Axis::X ? direction.x() : direction.y();
    if (isFlat() || ds == 0) {
        // Same arithmetic as a horizontal plane, so flat pools sense exactly as before.
        const double floor = surface_z - depthAt(os);
        return (floor - origin.z()) / direction.z();
    }
    std::optional<double> best;
    const auto consider = [&](double s0, double s1, double depth0, double slope) {
        // Floor z over [s0, s1]: surface - (depth0 + slope * (s - s0)).
        const double denominator = direction.z() + slope * ds;
        if (denominator == 0)
            return;
        const double rise = slope == 0 ? 0 : slope * (os - s0); // the flat extensions start at -/+infinity
        const double t = (surface_z - depth0 - rise - origin.z()) / denominator;
        const double s = os + t * ds;
        if (t >= 0 && s >= s0 && s <= s1 && (!best || t < *best))
            best = t;
    };
    const double infinity = std::numeric_limits<double>::infinity();
    consider(-infinity, polyline_.front().x(), polyline_.front().y(), 0);
    for (std::size_t k = 0; k + 1 < polyline_.size(); ++k) {
        const Eigen::Vector2d &a = polyline_[k], &b = polyline_[k + 1];
        consider(a.x(), b.x(), a.y(), (b.y() - a.y()) / (b.x() - a.x()));
    }
    consider(polyline_.back().x(), infinity, polyline_.back().y(), 0);
    return best;
}

PoolFloor::PoolFloor(std::vector<FloorProfile> profiles) : profiles_(std::move(profiles)) {
    for (const auto &profile : profiles_)
        require(!profile.empty(), "pool floor profiles must not be empty");
}

PoolFloor PoolFloor::flat(double depth, double length) {
    return PoolFloor({FloorProfile::flat(depth, length)});
}

double PoolFloor::depthAt(const Eigen::Vector2d &pool_xy) const {
    require(!empty(), "empty floor has no depth");
    double depth = profiles_.front().depthAt(pool_xy);
    for (std::size_t k = 1; k < profiles_.size(); ++k)
        depth = std::min(depth, profiles_[k].depthAt(pool_xy));
    return depth;
}

double PoolFloor::maxDepth() const {
    require(!empty(), "empty floor has no depth");
    double depth = profiles_.front().maxDepth();
    for (std::size_t k = 1; k < profiles_.size(); ++k)
        depth = std::min(depth, profiles_[k].maxDepth());
    return depth;
}

std::optional<double> PoolFloor::rayDistance(const Eigen::Vector3d &origin, const Eigen::Vector3d &direction,
                                             double surface_z) const {
    require(!empty(), "empty floor has no floor");
    std::optional<double> best;
    for (const auto &profile : profiles_)
        if (const auto t = profile.rayDistance(origin, direction, surface_z); t && (!best || *t < *best))
            best = t;
    return best;
}

std::vector<FloorBox> floorBoxes(const PoolFloor &floor, double length, double width, double surface_z,
                                 double thickness, double overlap) {
    std::vector<FloorBox> boxes;
    for (const auto &profile : floor.profiles()) {
        const double span = profile.axis() == FloorProfile::Axis::X ? width : length;
        for (auto &box : floorBoxes(profile, span, surface_z, thickness, overlap))
            boxes.push_back(box);
    }
    return boxes;
}

std::vector<FloorBox> floorBoxes(const FloorProfile &profile, double span, double surface_z, double thickness,
                                 double overlap) {
    require(!profile.empty(), "empty profile has no floor boxes");
    require(std::isfinite(span) && span > 0 && std::isfinite(thickness) && thickness > 0 && std::isfinite(overlap) &&
                overlap >= 0,
            "floor boxes need a positive span and thickness");
    const bool alongX = profile.axis() == FloorProfile::Axis::X;
    const auto lift = [&](double s, double depth) {
        return alongX ? Eigen::Vector3d(s, span / 2, surface_z - depth)
                      : Eigen::Vector3d(span / 2, s, surface_z - depth);
    };
    std::vector<FloorBox> boxes;
    const auto &line = profile.polyline();
    for (std::size_t k = 0; k + 1 < line.size(); ++k) {
        const Eigen::Vector3d a = lift(line[k].x(), line[k].y()), b = lift(line[k + 1].x(), line[k + 1].y());
        const Eigen::Vector3d u = (b - a).normalized();
        // Up normal of the segment, in the plane of the axis and z.
        const Eigen::Vector3d n = alongX ? Eigen::Vector3d(-u.z(), 0, u.x()) : Eigen::Vector3d(0, -u.z(), u.y());
        Eigen::Matrix3d rotation;
        rotation << u, n.cross(u), n;
        FloorBox box;
        box.size = {(b - a).norm() + 2 * overlap, span, thickness};
        box.center = (a + b) / 2 - n * (thickness / 2);
        box.orientation = Eigen::Quaterniond(rotation).normalized();
        boxes.push_back(box);
    }
    return boxes;
}

} // namespace nereus::simulation
