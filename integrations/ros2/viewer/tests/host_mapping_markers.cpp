#include "mapping_markers.hpp"

#include <gtest/gtest.h>

#include <fstream>

using nereus::ros_viewer::host::hideMappingMarkers;
using nereus::ros_viewer::host::loadMappingMarkers;
using nereus::ros_viewer::host::mappingMarkerMatches;
using nereus::ros_viewer::host::sortMappingMarkers;

namespace {
std::filesystem::path write(const std::string &text) {
    const auto path =
        std::filesystem::temp_directory_path() / ("nereus_markers_" + std::to_string(::getpid()) + ".yaml");
    std::ofstream(path) << text;
    return path;
}
// Same shape as riptide_rviz config/markers.yaml (MarkerPublisher parameters).
const char *kMarkers = R"(/**/marker_publisher:
  ros__parameters:
    mesh_pkg: riptide_meshes
    mesh_directory: meshes
    markers:
      marker0: {mesh: gate, frame: gate_frame, scale: [1.0, 1.0, 1.0], pose: [0.0, -1.5, 0.0, 0.0, 0.0, 1.5707963], color: [0.0, 0.0, 0.0, 0.0]}
      marker1: {mesh: arrow, frame: pre_gate_frame, scale: [1.0, 1.0, 1.0], pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], color: [1.0, 0.0, 0.0, 1.0]}
      marker2: {mesh: table, frame: table_frame, scale: [2.0, 2.0, 2.0], pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], color: [0.0, 0.0, 0.0, 0.0]}
      marker3: {mesh: "", frame: ignored_frame}
      marker4: {mesh: bin, frame: bin_frame}
)";
} // namespace

TEST(MappingMarkers, ReadsMeshMarkersInOrderLikeMarkerPublisher) {
    const auto file = write(kMarkers);
    std::string asked;
    const auto markers = loadMappingMarkers(file, [&](const std::string &package) {
        asked = package;
        return std::filesystem::path("/share") / package;
    });
    std::filesystem::remove(file);
    EXPECT_EQ(asked, "riptide_meshes");
    // The arrow is a builtin shape (skipped); marker3 is incomplete, so marker4 is never read.
    ASSERT_EQ(markers.size(), 2u);
    EXPECT_EQ(markers[0].frame, "gate_frame");
    EXPECT_EQ(markers[0].path, std::filesystem::path("/share/riptide_meshes/meshes/gate/model.dae"));
    // pose xyz then yaw 90 deg: the local +X axis maps to +Y.
    const glm::vec4 x = markers[0].local * glm::vec4(1, 0, 0, 0);
    EXPECT_NEAR(x.y, 1, 1e-5);
    EXPECT_NEAR(markers[0].local[3].y, -1.5, 1e-6);
    // scale applies after the pose: a unit point at x=1 lands at 1 + 2.
    const glm::vec4 p = markers[1].local * glm::vec4(1, 0, 0, 1);
    EXPECT_NEAR(p.x, 3, 1e-6);
}

TEST(MappingMarkers, HidesByFrameLabelOrMesh) {
    const auto file = write(R"(/**/marker_publisher:
  ros__parameters:
    mesh_pkg: riptide_meshes
    markers:
      marker0: {mesh: slalom, frame: slalom_front_frame}
      marker1: {mesh: slalom, frame: slalom_back_frame}
      marker2: {mesh: table, frame: table_frame}
      marker3: {mesh: bin, frame: bin}
)");
    auto markers = loadMappingMarkers(file, [](const std::string &) { return std::filesystem::path("/share"); });
    std::filesystem::remove(file);
    ASSERT_EQ(markers.size(), 4u);
    EXPECT_EQ(markers[0].label, "slalom_front");
    EXPECT_EQ(markers[3].label, "bin"); // no _frame suffix: the frame itself
    for (const auto &marker : markers)
        EXPECT_TRUE(marker.visible);
    EXPECT_EQ(hideMappingMarkers(markers, {"slalom", "nope"}), std::vector<std::string>{"nope"});
    EXPECT_FALSE(markers[0].visible); // a mesh name hides every copy
    EXPECT_FALSE(markers[1].visible);
    EXPECT_TRUE(markers[2].visible);
    EXPECT_TRUE(hideMappingMarkers(markers, {"table_frame", "bin"}).empty()); // frame, label
    EXPECT_FALSE(markers[2].visible);
    EXPECT_FALSE(markers[3].visible);
}

TEST(MappingMarkers, SortsByLabelAndFiltersIgnoringCase) {
    std::vector<nereus::ros_viewer::host::MappingMarker> markers(4);
    const char *rows[][3] = {{"table", "table", "table_frame"},
                             {"slalom_front", "slalom", "slalom_front_frame"},
                             {"Gate", "gate", "gate_frame"},
                             {"bin_vinyl1", "bin_vinyl", "bin_vinyl1_frame"}};
    for (std::size_t i = 0; i < markers.size(); ++i) {
        markers[i].label = rows[i][0];
        markers[i].mesh = rows[i][1];
        markers[i].frame = rows[i][2];
    }
    sortMappingMarkers(markers);
    std::vector<std::string> order;
    for (const auto &marker : markers)
        order.push_back(marker.label);
    EXPECT_EQ(order, (std::vector<std::string>{"bin_vinyl1", "Gate", "slalom_front", "table"}));
    EXPECT_TRUE(mappingMarkerMatches(markers[1], ""));          // empty query: everything
    EXPECT_TRUE(mappingMarkerMatches(markers[1], "GATE"));      // case
    EXPECT_TRUE(mappingMarkerMatches(markers[0], "vinyl1_fr")); // frame
    EXPECT_TRUE(mappingMarkerMatches(markers[2], "slalom"));    // mesh
    EXPECT_FALSE(mappingMarkerMatches(markers[3], "gate"));
}

TEST(MappingMarkers, LocalMeshFolderOverridesThePackage) {
    const auto file = write(kMarkers);
    bool asked = false;
    const auto markers = loadMappingMarkers(
        file, [&](const std::string &) { return asked = true, std::filesystem::path("/unused"); }, "/local/meshes");
    std::filesystem::remove(file);
    EXPECT_FALSE(asked); // no ROS package lookup when a folder is given
    EXPECT_EQ(markers[1].path, std::filesystem::path("/local/meshes/table/model.dae"));
}
