#include <fstream>
#include <gtest/gtest.h>
#include <robotics/scene_view/scene.hpp>
#include <unistd.h>

namespace s = robotics::scene_view;
namespace v = robotics::visualization;
TEST(SceneView, SourceIdentityDisconnectAndRewindNeverRetainPose) {
    s::Document document;
    robotics::rendering::Scene content;
    content.instances.emplace_back();
    document.groups.push_back({"robot", "recording", "body", content});
    auto data = std::make_shared<v::SourceData>();
    data->frames = std::make_shared<v::FrameGraph>(
        "world", std::vector<v::FrameEdge>{{"world",
                                            "body",
                                            false,
                                            {{0, {{1, 2, 3}, Eigen::Quaterniond::Identity()}},
                                             {10, {{5, 6, 7}, Eigen::Quaterniond::Identity()}}}}});
    v::SourceSnapshot source{"recording", 0, 10, data};
    auto resolved = s::resolve(document, "world", &source);
    ASSERT_EQ(resolved.scene.instances.size(), 1);
    EXPECT_FLOAT_EQ(resolved.scene.instances[0].transform(0, 3), 5);
    source.time_ns = 0;
    ++source.generation;
    resolved = s::resolve(document, "world", &source);
    ASSERT_EQ(resolved.scene.instances.size(), 1);
    EXPECT_FLOAT_EQ(resolved.scene.instances[0].transform(0, 3), 1);
    source.id = "other";
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
    source.id = "recording";
    source.data.reset();
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
    EXPECT_TRUE(s::resolve(document, "world", nullptr).scene.instances.empty());
}
TEST(SceneView, StaticPoolAndTiltedSourceBindingAreExplicit) {
    s::Document document;
    document.groups.push_back({"pool", "", "world", robotics::rendering::makePoolScene()});
    EXPECT_EQ(s::resolve(document, "world", nullptr).scene.instances.size(), 13);
    EXPECT_TRUE(s::resolve(document, "world", nullptr)
                    .scene.lighting_center.isApprox(document.groups[0].content.lighting_center));
    EXPECT_TRUE(s::resolve(document, "different_world", nullptr).scene.instances.empty());
    document.groups[0].source = "live";
    auto data = std::make_shared<v::SourceData>();
    data->frames = std::make_shared<v::FrameGraph>(
        "body", std::vector<v::FrameEdge>{
                    {"body",
                     "world",
                     true,
                     {{0,
                       {{0, 0, 0},
                        Eigen::Quaterniond(Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()))}}}}});
    v::SourceSnapshot source{"live", 0, 0, data};
    const auto resolved = s::resolve(document, "body", &source);
    EXPECT_TRUE(resolved.scene.instances.empty());
    EXPECT_FALSE(resolved.scene.water);
    EXPECT_NE(resolved.issues.at("pool").find("Z-up"), std::string::npos);
}
TEST(SceneView, LoaderRejectsUnknownFieldsAndResolvesRelativeMeshes) {
    const auto temp = std::filesystem::temp_directory_path() /
                      ("robotics-scene-test-" + std::to_string(getpid()) + ".yaml");
    {
        std::ofstream output(temp);
        output << "version: 1\ngroups: []\nunknown: true\n";
    }
    EXPECT_THROW(s::load(temp), std::invalid_argument);
    {
        std::ofstream output(temp);
        output << "version: 1\ngroups:\n  - id: sample\n    frame: world\n    instances:\n      - "
                  "mesh: "
               << (std::filesystem::path(RP_ROOT) / "tests/fixtures/nested_mesh.gltf")
                      .lexically_relative(temp.parent_path())
                      .string()
               << "\n";
    }
    const auto document = s::load(temp);
    ASSERT_EQ(document.groups.size(), 1);
    ASSERT_EQ(document.groups[0].content.instances.size(), 1);
    EXPECT_FALSE(document.groups[0].content.instances[0].mesh->submeshes.empty());
    std::filesystem::remove(temp);
}
