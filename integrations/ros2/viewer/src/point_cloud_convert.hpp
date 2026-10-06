// sensor_msgs/PointCloud2 -> renderer points (xyz in the message frame, rgb in [0, 1]).
#pragma once
#include <cmath>
#include <cstring>
#include <memory>
#include <nereus/rendering/scene.hpp>
#include <optional>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace nereus::ros_viewer::host {
// Reads float32 x/y/z and, when present, a packed float32/uint32 `rgb` or `rgba` field (bytes B, G, R as the
// ZED driver, PCL and the simulator bridge publish). Without a colour field every point gets `fallback`.
// Non-finite points are skipped; at most `maxPoints` are kept (uniform decimation). Returns null for layouts
// it cannot read (big-endian, missing or non-float32 coordinates, inconsistent sizes).
inline std::shared_ptr<rendering::PointData> convertPointCloud(const sensor_msgs::msg::PointCloud2 &msg,
                                                               const Eigen::Vector3f &fallback,
                                                               std::size_t maxPoints = 500000) {
    using Field = sensor_msgs::msg::PointField;

    // Locate the scalar fields by name and check their types and offsets.
    const auto field = [&](const char *name) -> const Field * {
        for (const auto &f : msg.fields)
            if (f.name == name && f.count == 1)
                return &f;
        return nullptr;
    };
    const Field *x = field("x"), *y = field("y"), *z = field("z");
    const Field *rgb = field("rgb") ? field("rgb") : field("rgba");
    if (msg.is_bigendian || !x || !y || !z)
        return nullptr;
    for (const Field *f : {x, y, z})
        if (f->datatype != Field::FLOAT32 || f->offset + 4 > msg.point_step)
            return nullptr;
    if (rgb &&
        ((rgb->datatype != Field::FLOAT32 && rgb->datatype != Field::UINT32) || rgb->offset + 4 > msg.point_step))
        rgb = nullptr;

    // The buffer must hold every row it claims.
    const std::size_t count = std::size_t(msg.width) * msg.height;
    if (msg.point_step == 0 || msg.row_step < std::size_t(msg.width) * msg.point_step ||
        msg.data.size() < std::size_t(msg.row_step) * msg.height)
        return nullptr;

    // Keep every `step`-th point so at most maxPoints remain.
    const std::size_t step = maxPoints && count > maxPoints ? (count + maxPoints - 1) / maxPoints : 1;
    auto out = std::make_shared<rendering::PointData>();
    out->xyzrgb.reserve(6 * (count / step + 1));
    // Unaligned float32 read.
    const auto read = [](const std::uint8_t *p) {
        float v;
        std::memcpy(&v, p, 4);
        return v;
    };

    // Interleave x, y, z, r, g, b per kept point.
    for (std::size_t i = 0; i < count; i += step) {
        const std::uint8_t *point = msg.data.data() + (i / msg.width) * msg.row_step + (i % msg.width) * msg.point_step;
        const float px = read(point + x->offset), py = read(point + y->offset), pz = read(point + z->offset);
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
            continue;
        out->xyzrgb.insert(out->xyzrgb.end(), {px, py, pz});
        if (rgb) {
            const std::uint8_t *c = point + rgb->offset;
            out->xyzrgb.insert(out->xyzrgb.end(), {c[2] / 255.f, c[1] / 255.f, c[0] / 255.f});
        } else {
            out->xyzrgb.insert(out->xyzrgb.end(), {fallback.x(), fallback.y(), fallback.z()});
        }
    }
    return out;
}
} // namespace nereus::ros_viewer::host
