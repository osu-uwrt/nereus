// PointCloud2 -> packed xyzrgb floats for the viewer's point-cloud renderer.
#include "point_cloud_convert.hpp"

#include <gtest/gtest.h>

using nereus::ros_viewer::host::convertPointCloud;
using Field = sensor_msgs::msg::PointField;

namespace {
// One scalar PointField (count 1) at a byte offset.
Field field(const char *name, std::uint32_t offset, std::uint8_t type = Field::FLOAT32) {
    Field f;
    f.name = name;
    f.offset = offset;
    f.datatype = type;
    f.count = 1;
    return f;
}

// Organized 2x2 cloud in the PCL PointXYZRGB layout (point_step 32, packed B,G,R at offset 16).
sensor_msgs::msg::PointCloud2 zedLayout() {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 2;
    cloud.width = 2;
    cloud.point_step = 32;
    cloud.row_step = 64;
    cloud.fields = {field("x", 0), field("y", 4), field("z", 8), field("rgb", 16)};
    cloud.data.assign(128, 0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    // The second point is invalid (NaN) and must be dropped.
    const float xyz[4][3] = {{1, 2, 3}, {nan, nan, nan}, {-1, 0, 2}, {0, 0, 1}};
    for (int i = 0; i < 4; ++i) {
        std::memcpy(&cloud.data[32 * i], xyz[i], 12);
        cloud.data[32 * i + 16] = 10;  // B
        cloud.data[32 * i + 17] = 20;  // G
        cloud.data[32 * i + 18] = 255; // R
    }
    return cloud;
}
} // namespace

// Output is 6 floats per point (x y z r g b, colour in 0..1).
TEST(PointCloud, ReadsPackedColourAndSkipsInvalidPoints) {
    const auto points = convertPointCloud(zedLayout(), {0, 0, 1});
    ASSERT_TRUE(points);
    ASSERT_EQ(points->xyzrgb.size(), 3u * 6); // the NaN point is dropped
    EXPECT_FLOAT_EQ(points->xyzrgb[0], 1);
    EXPECT_FLOAT_EQ(points->xyzrgb[2], 3);
    EXPECT_FLOAT_EQ(points->xyzrgb[3], 1.f);          // R
    EXPECT_NEAR(points->xyzrgb[4], 20 / 255.f, 1e-6); // G
    EXPECT_NEAR(points->xyzrgb[5], 10 / 255.f, 1e-6); // B
    EXPECT_FLOAT_EQ(points->xyzrgb[6], -1);           // next valid point
}

TEST(PointCloud, UsesTheFallbackColourWithoutAnRgbField) {
    auto cloud = zedLayout();
    cloud.fields.pop_back();
    const auto points = convertPointCloud(cloud, {.2f, .4f, .6f});
    ASSERT_TRUE(points);
    EXPECT_FLOAT_EQ(points->xyzrgb[3], .2f);
    EXPECT_FLOAT_EQ(points->xyzrgb[5], .6f);
}

// A point limit strides through the cloud rather than truncating it.
TEST(PointCloud, DecimatesToTheLimit) {
    const auto points = convertPointCloud(zedLayout(), {1, 1, 1}, 2);
    ASSERT_TRUE(points);
    EXPECT_EQ(points->xyzrgb.size(), 2u * 6); // points 0 and 2
    EXPECT_FLOAT_EQ(points->xyzrgb[6], -1);
}

// Big-endian data, non-float32 xyz, short buffers and a missing axis all return nothing.
TEST(PointCloud, RejectsUnreadableLayouts) {
    auto big = zedLayout();
    big.is_bigendian = true;
    EXPECT_FALSE(convertPointCloud(big, {1, 1, 1}));

    auto doubles = zedLayout();
    doubles.fields[0].datatype = Field::FLOAT64;
    EXPECT_FALSE(convertPointCloud(doubles, {1, 1, 1}));

    auto truncated = zedLayout();
    truncated.data.resize(100);
    EXPECT_FALSE(convertPointCloud(truncated, {1, 1, 1}));

    auto missing = zedLayout();
    missing.fields.erase(missing.fields.begin() + 2); // no z
    EXPECT_FALSE(convertPointCloud(missing, {1, 1, 1}));
}
