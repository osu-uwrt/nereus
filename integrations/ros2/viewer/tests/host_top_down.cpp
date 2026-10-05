// The course map's top-down images: a mesh projected straight down, the highest surface on top, its edge darkened.
#include "../src/top_down.hpp"
#include <gtest/gtest.h>

using namespace nereus::ros_viewer::host;

namespace {
// An axis-aligned box mesh (12 triangles) from `low` to `high`, coloured `rgb`.
std::shared_ptr<nereus::rendering::MeshAsset> box(Eigen::Vector3f low, Eigen::Vector3f high, Eigen::Vector3f rgb) {
    auto mesh = std::make_shared<nereus::rendering::MeshAsset>();
    nereus::rendering::Submesh part;
    part.material.base_color = {rgb.x(), rgb.y(), rgb.z(), 1};
    for (int i = 0; i < 8; ++i)
        part.vertices.push_back({Eigen::Vector3f(i & 1 ? high.x() : low.x(), i & 2 ? high.y() : low.y(),
                                                 i & 4 ? high.z() : low.z()),
                                 Eigen::Vector3f::UnitZ(), Eigen::Vector2f::Zero()});
    part.indices = {0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                    2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 3, 7, 1, 7, 5};
    mesh->submeshes.push_back(part);
    mesh->minimum = low;
    mesh->maximum = high;
    return mesh;
}
const std::uint8_t *pixel(const TopDownImage &image, int x, int y) {
    return &image.rgba[(std::size_t(y) * std::size_t(image.width) + std::size_t(x)) * 4];
}
} // namespace

TEST(TopDown, ProjectsTheFootprintWithItsColourAndARim) {
    const auto red = box({1, 1, 0}, {2, 3, 1}, {1, 0, 0});
    const auto image = bakeTopDown({{red, glm::mat4(1)}}, {0, 0}, {4, 4}, 10);
    ASSERT_EQ(image.width, 40);
    ASSERT_EQ(image.height, 40);
    // inside the footprint (x 1..2, y 1..3; row 0 is y = 4): red and opaque; outside: transparent
    const auto *inside = pixel(image, 15, 20);
    EXPECT_EQ(inside[3], 255);
    EXPECT_GT(inside[0], 200);
    EXPECT_LT(inside[1], 20);
    EXPECT_EQ(pixel(image, 5, 20)[3], 0);
    EXPECT_EQ(pixel(image, 15, 5)[3], 0);
    // the footprint's edge is darker than its middle
    EXPECT_LT(pixel(image, 10, 20)[0], inside[0]);
}

TEST(TopDown, TheHighestSurfaceWins) {
    const auto low = box({0, 0, 0}, {2, 2, .5f}, {0, 0, 1}), high = box({.5f, .5f, 0}, {1.5f, 1.5f, 1}, {0, 1, 0});
    const auto image = bakeTopDown({{high, glm::mat4(1)}, {low, glm::mat4(1)}}, {0, 0}, {2, 2}, 10);
    const auto *middle = pixel(image, 10, 10), *edge = pixel(image, 3, 10);
    EXPECT_GT(middle[1], 150); // the taller green box covers the blue one's middle whatever the order
    EXPECT_LT(middle[2], 50);
    EXPECT_GT(edge[2], 150); // the blue box shows around it
}

TEST(TopDown, BoundsFollowThePlacement) {
    const auto unit = box({0, 0, 0}, {1, 1, 1}, {1, 1, 1});
    glm::mat4 moved(1);
    moved[3] = {5, -2, 0, 1};
    const auto [lo, hi] = topDownBounds({{unit, moved}});
    EXPECT_FLOAT_EQ(lo.x, 5);
    EXPECT_FLOAT_EQ(lo.y, -2);
    EXPECT_FLOAT_EQ(hi.x, 6);
    EXPECT_FLOAT_EQ(hi.y, -1);
}

TEST(TopDown, TheHaloGrowsTheSilhouette) {
    const auto dot = box({1.9f, 1.9f, 0}, {2.1f, 2.1f, 1}, {1, 1, 1});
    const auto image = bakeTopDown({{dot, glm::mat4(1)}}, {0, 0}, {4, 4}, 10);
    const auto halo = haloOf(image, 5);
    EXPECT_EQ(pixel(image, 15, 20)[3], 0);  // half a metre from the post: empty in the image
    EXPECT_EQ(pixel(halo, 16, 20)[3], 255); // within the halo's 5 px
    EXPECT_EQ(pixel(halo, 5, 20)[3], 0);    // beyond it
}
