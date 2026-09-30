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
                      ("nereus-scene-test-" + std::to_string(getpid()) + ".yaml");
    {
        std::ofstream output(temp);
        output << "version: 1\ngroups: []\nunknown: true\n";
    }
    EXPECT_THROW(s::load(temp), std::invalid_argument);
    {
        std::ofstream output(temp);
        output << "version: 1\ngroups:\n  - id: sample\n    frame: world\n    instances:\n      - "
                  "mesh: "
               << (std::filesystem::path(NEREUS_ROOT) / "tests/fixtures/nested_mesh.gltf")
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

TEST(SceneView, ColorBindingsUseTheSameFrozenSourceTimeAndOmitIncompleteGroups) {
    s::Document document;
    robotics::rendering::Scene content;
    content.instances.emplace_back();
    content.instances[0].tint = {.5f, .8f, 1, .25f};
    content.instances[0].radiance = 240;
    document.groups.push_back({"lamp", "recording", "world", content, {{0, "status"}}});
    auto data = std::make_shared<v::SourceData>();
    data->frames = std::make_shared<v::FrameGraph>("world", std::vector<v::FrameEdge>{});
    data->colors["status"] = {{0, {1, .5f, 0}}, {10, {0, 0, 1}}};
    v::SourceSnapshot source{"recording", 0, 5, data};
    auto resolved = s::resolve(document, "world", &source);
    ASSERT_EQ(resolved.scene.instances.size(), 1U);
    EXPECT_EQ(resolved.scene.instances[0].tint, Eigen::Vector4f(.5f, .4f, 0, .25f));
    EXPECT_EQ(resolved.scene.instances[0].radiance, 240);
    source.time_ns = 10;
    EXPECT_EQ(s::resolve(document, "world", &source).scene.instances[0].tint,
              Eigen::Vector4f(0, 0, 1, .25f));
    source.time_ns = 11;
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
    source.time_ns = 0;
    EXPECT_EQ(s::resolve(document, "world", &source).scene.instances[0].tint,
              Eigen::Vector4f(.5f, .4f, 0, .25f));
    document.groups[0].colors[0].instance = 1;
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
    document.groups[0].colors[0] = {0, "absent"};
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
    source.data.reset();
    EXPECT_TRUE(s::resolve(document, "world", &source).scene.instances.empty());
}

TEST(SceneView, EmissiveBoxesPreserveDimensionsMountAndSourceBinding) {
    const auto temp = std::filesystem::temp_directory_path() /
                      ("nereus-box-test-" + std::to_string(getpid()) + ".yaml");
    const std::string valid = R"(version: 1
groups:
  - id: lamp
    source: live
    frame: cad
    instances:
      - box: [0.012, 0.055, 0.003]
        pose: {position: [-0.181, 0.1573, 0.094], orientation_wxyz: [1, 0, 0, 0]}
        color_channel: status
        material: emissive
        radiance: 240
        casts_shadow: false
)";
    {
        std::ofstream(temp) << valid;
    }
    const auto document = s::load(temp);
    const auto &group = document.groups.at(0);
    const auto &instance = group.content.instances.at(0);
    EXPECT_EQ(instance.mesh->submeshes.at(0).vertices.size(), 24U);
    EXPECT_EQ(instance.mesh->submeshes.at(0).indices.size(), 36U);
    EXPECT_EQ(instance.transform(0, 0), .012f);
    EXPECT_EQ(instance.transform(1, 1), .055f);
    EXPECT_EQ(instance.transform(2, 2), .003f);
    EXPECT_EQ(instance.transform(0, 3), -.181f);
    EXPECT_EQ(instance.material, robotics::rendering::SurfaceMaterial::Emissive);
    EXPECT_EQ(instance.radiance, 240);
    EXPECT_FALSE(instance.casts_shadow);
    EXPECT_EQ(group.colors.at(0).channel, "status");
    EXPECT_EQ(group.colors.at(0).instance, 0U);
    for (const auto &[from, to] : std::vector<std::pair<std::string, std::string>>{
             {"box: [0.012, 0.055, 0.003]", "box: [0, 1, 1]"},
             {"box: [0.012, 0.055, 0.003]", "box: [1, 1, 1]\n        mesh: file.glb"},
             {"    source: live\n", ""},
             {"color_channel: status", "color_channel: ''"}}) {
        auto text = valid;
        text.replace(text.find(from), from.size(), to);
        {
            std::ofstream(temp) << text;
        }
        EXPECT_THROW(s::load(temp), std::invalid_argument);
    }
    std::filesystem::remove(temp);
}
