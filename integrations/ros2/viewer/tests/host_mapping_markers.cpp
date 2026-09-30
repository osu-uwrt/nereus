#include "mapping_markers.hpp"

#include <gtest/gtest.h>

#include <fstream>

using nereus::ros_viewer::host::loadMappingMarkers;

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

TEST(MappingMarkers, LocalMeshFolderOverridesThePackage) {
    const auto file = write(kMarkers);
    bool asked = false;
    const auto markers = loadMappingMarkers(
        file, [&](const std::string &) { return asked = true, std::filesystem::path("/unused"); }, "/local/meshes");
    std::filesystem::remove(file);
    EXPECT_FALSE(asked); // no ROS package lookup when a folder is given
    EXPECT_EQ(markers[1].path, std::filesystem::path("/local/meshes/table/model.dae"));
}
