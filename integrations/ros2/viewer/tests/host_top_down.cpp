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
    // Corner i takes high x/y/z where bits 0/1/2 of i are set.
    for (int i = 0; i < 8; ++i)
        part.vertices.push_back(
            {Eigen::Vector3f(i & 1 ? high.x() : low.x(), i & 2 ? high.y() : low.y(), i & 4 ? high.z() : low.z()),
             Eigen::Vector3f::UnitZ(), Eigen::Vector2f::Zero()});
    part.indices = {0, 1, 3, 0, 3, 2, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4,
                    2, 3, 7, 2, 7, 6, 0, 2, 6, 0, 6, 4, 1, 3, 7, 1, 7, 5};
    mesh->submeshes.push_back(part);
    mesh->minimum = low;
    mesh->maximum = high;
    return mesh;
}

// The RGBA bytes of pixel (x, y), row 0 at the top.
const std::uint8_t *pixel(const TopDownImage &image, int x, int y) {
    return &image.rgba[(std::size_t(y) * std::size_t(image.width) + std::size_t(x)) * 4];
}
} // namespace

// A 4 x 4 m area at 10 px/m: the box covers its footprint in its own colour, everything else is transparent.
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

    // its levels: the full size gets a darker edge than its middle
    const auto levels = topDownLevels(image);
    EXPECT_LT(pixel(levels[0], 10, 20)[0], pixel(levels[0], 15, 20)[0]);
}

TEST(TopDown, TheHighestSurfaceWins) {
    const auto low = box({0, 0, 0}, {2, 2, .5f}, {0, 0, 1}), high = box({.5f, .5f, 0}, {1.5f, 1.5f, 1}, {0, 1, 0});
    const auto image = bakeTopDown({{high, glm::mat4(1)}, {low, glm::mat4(1)}}, {0, 0}, {2, 2}, 10);
    const auto *middle = pixel(image, 10, 10), *edge = pixel(image, 3, 10);
    EXPECT_GT(middle[1], 150); // the taller green box covers the blue one's middle whatever the order
    EXPECT_LT(middle[2], 50);
    EXPECT_GT(edge[2], 150); // the blue box shows around it
}

// topDownBounds is the XY extent of the placed meshes (placement transform applied).
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

TEST(TopDown, ThinPropsStayOpaqueInEverySmallerLevel) {
    // a 5 cm post in a 4 m image at 32 px/m: under 2 px wide, gone from an averaged mip chain's small levels
    const auto post = box({1.98f, 0, 0}, {2.03f, 4, 1}, {1, 1, 1});
    const auto levels = topDownLevels(bakeTopDown({{post, glm::mat4(1)}}, {0, 0}, {4, 4}, 32));
    ASSERT_GE(levels.size(), 6u); // 128, 64, 32, 16, 8, 4, ...

    // Count opaque pixels along the middle row of each level.
    for (std::size_t i = 0; i < 6; ++i) {
        const auto &level = levels[i];
        int opaque = 0;
        for (int x = 0; x < level.width; ++x)
            opaque += pixel(level, x, level.height / 2)[3] == 255;
        EXPECT_GE(opaque, 1) << "level " << i; // the post is still drawn, solid
        EXPECT_LE(opaque, 2) << "level " << i; // and not grown into a smear
    }
}
