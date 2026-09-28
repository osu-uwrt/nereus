#include <robotics/viewer/session.hpp>

#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <unistd.h>

namespace v = robotics::visualization;
namespace ui = robotics::viewer;
namespace {
v::Pose translated(double x, double y = 0, double z = 0) {
    return {{x, y, z}, Eigen::Quaterniond::Identity()};
}
v::Recording recording() {
    v::Recording result;
    result.data.clock = "test";
    result.duration_ns = 100;
    result.data.frames = std::make_shared<const v::FrameGraph>(
        "world", std::vector<v::FrameEdge>{
                     {"world", "odom", false, {{0, translated(0)}, {100, translated(10)}}}});
    result.data.streams["pose"] = {
        {0, "odom", translated(1)}, {50, "odom", translated(2)}, {100, "odom", translated(3)}};
    return result;
}
v::DisplaySettings settings(const std::string &type) {
    v::DisplaySettings result;
    result.id = "display";
    result.type = type;
    result.source = "a";
    result.stream = "pose";
    return result;
}
class Temporary {
  public:
    Temporary() {
        std::string path = (std::filesystem::temp_directory_path() / "viewer-test-XXXXXX").string();
        if (!mkdtemp(path.data()))
            throw std::runtime_error("temporary directory failed");
        directory = path;
    }
    ~Temporary() {
        std::error_code error;
        std::filesystem::remove_all(directory, error);
    }
    std::filesystem::path directory;
};
} // namespace
TEST(Frames, CompositionInverseAndRotationInterpolation) {
    v::Pose turned{
        {2, 0, 0},
        Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793 / 2, Eigen::Vector3d::UnitZ()))};
    v::FrameGraph frames("world", {{"world", "body", false, {{0, translated(0)}, {100, turned}}},
                                   {"body", "sensor", true, {{0, translated(1)}}}});
    const auto end = frames.lookup("world", "sensor", 100);
    ASSERT_TRUE(end.pose);
    EXPECT_TRUE(end.pose->translation.isApprox(Eigen::Vector3d(2, 1, 0), 1e-12));
    const auto middle = frames.lookup("world", "body", 50);
    ASSERT_TRUE(middle.pose);
    EXPECT_NEAR(middle.pose->translation.x(), 1, 1e-12);
    EXPECT_TRUE((middle.pose->rotation * Eigen::Vector3d::UnitX())
                    .isApprox(Eigen::Vector3d(std::sqrt(0.5), std::sqrt(0.5), 0), 1e-12));
    const auto back = frames.lookup("sensor", "world", 100);
    ASSERT_TRUE(back.pose);
    EXPECT_TRUE(v::compose(*end.pose, *back.pose).translation.isZero(1e-12));
    EXPECT_FALSE(frames.lookup("world", "sensor", 101).pose);
    EXPECT_FALSE(frames.lookup("world", "missing", 50).pose);
    EXPECT_FALSE(frames.lookup("world", "body", -1).pose);
}
TEST(Frames, CommonAncestorDoesNotRequireUnavailableUpstreamHistory) {
    v::FrameGraph frames("world", {{"world", "body", false, {{10, translated(10)}}},
                                   {"body", "a", true, {{0, translated(1)}}},
                                   {"body", "b", true, {{0, translated(3)}}}});
    const auto found = frames.lookup("a", "b", 0);
    ASSERT_TRUE(found.pose);
    EXPECT_DOUBLE_EQ(found.pose->translation.x(), 2);
    EXPECT_FALSE(frames.lookup("world", "b", 0).pose);
}
TEST(Frames, ExactIntegerTimeAndQuaternionSign) {
    constexpr v::Time large = 9007199254740993;
    v::Pose negative = translated(4);
    negative.rotation.coeffs() *= -1;
    v::FrameGraph frames(
        "world", {{"world", "body", false, {{large, translated(0)}, {large + 2, negative}}}});
    const auto found = frames.lookup("world", "body", large + 1);
    ASSERT_TRUE(found.pose);
    EXPECT_DOUBLE_EQ(found.pose->translation.x(), 2);
    EXPECT_TRUE(
        (found.pose->rotation * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitX()));
}
TEST(Frames, RejectsCyclesDuplicatesUnknownParentsAndInvalidSamples) {
    const auto sample = std::vector<v::TransformSample>{{0, translated(0)}};
    EXPECT_THROW((v::FrameGraph("world", {{"b", "a", true, sample}, {"a", "b", true, sample}})),
                 std::invalid_argument);
    EXPECT_THROW((v::FrameGraph("world", {{"missing", "a", true, sample}})), std::invalid_argument);
    EXPECT_THROW(
        (v::FrameGraph("world", {{"world", "a", true, sample}, {"world", "a", true, sample}})),
        std::invalid_argument);
    EXPECT_THROW((v::FrameGraph("world", {{"world", "a", false, {{0, {}}, {0, {}}}}})),
                 std::invalid_argument);
    auto bad = translated(0);
    bad.rotation.coeffs() *= 2;
    EXPECT_THROW((v::FrameGraph("world", {{"world", "a", true, {{0, bad}}}})),
                 std::invalid_argument);
}
TEST(Sources, RewindDisconnectReconnectAndRetainedSnapshotOwnership) {
    v::LocalSource source("a", recording());
    source.seek(100);
    const auto retained = source.snapshot();
    source.seek(50);
    EXPECT_EQ(source.snapshot().generation, retained.generation + 1);
    source.disconnect();
    EXPECT_FALSE(source.snapshot().data);
    EXPECT_THROW(source.seek(0), std::logic_error);
    source.reconnect();
    EXPECT_EQ(source.snapshot().time_ns, 0);
    EXPECT_GT(source.snapshot().generation, retained.generation);
    EXPECT_EQ(retained.time_ns, 100);
    ASSERT_TRUE(retained.data);
    EXPECT_EQ(retained.data->streams.at("pose").size(), 3);
    const auto before = source.snapshot();
    EXPECT_THROW(source.seek(101), std::out_of_range);
    EXPECT_EQ(source.snapshot().generation, before.generation);
}
TEST(Sources, BoundedValidatedHistoryAndIndependentIdentity) {
    auto bad = recording();
    bad.data.streams["pose"][1].time_ns = 0;
    EXPECT_THROW(v::LocalSource("a", bad), std::invalid_argument);
    bad = recording();
    bad.data.streams["pose"].resize(10001);
    EXPECT_THROW(v::LocalSource("a", bad), std::invalid_argument);
    v::LocalSource a("a", recording()), b("b", recording());
    a.seek(100);
    a.disconnect();
    EXPECT_TRUE(b.snapshot().data);
    EXPECT_EQ(b.snapshot().time_ns, 0);
}
TEST(Displays, TrajectoryUsesEachMeasurementTimeAndRewindsWithoutFutureTrail) {
    v::LocalSource source("a", recording());
    source.seek(100);
    auto displays = v::standardDisplays();
    auto config = settings("trajectory");
    const std::string fixed = "world";
    auto snapshot = source.snapshot();
    auto result = displays.draw({&snapshot, fixed, config});
    ASSERT_EQ(result.lines.size(), 2);
    EXPECT_DOUBLE_EQ(result.lines[0].from.x(), 1);
    EXPECT_DOUBLE_EQ(result.lines[0].to.x(), 7);
    EXPECT_DOUBLE_EQ(result.lines[1].to.x(), 13);
    source.seek(50);
    snapshot = source.snapshot();
    result = displays.draw({&snapshot, fixed, config});
    ASSERT_EQ(result.lines.size(), 1);
    EXPECT_DOUBLE_EQ(result.lines[0].to.x(), 7);
    config.history_limit = 1;
    EXPECT_TRUE(displays.draw({&snapshot, fixed, config}).lines.empty());
    const std::string other = "odom";
    config.history_limit = 10;
    result = displays.draw({&snapshot, other, config});
    ASSERT_EQ(result.lines.size(), 1);
    EXPECT_DOUBLE_EQ(result.lines[0].to.x(), 2);
}
TEST(Displays, MissingTransformsCreateGapsAndNeverOriginFallback) {
    auto data = recording();
    data.data.streams["pose"][1].frame = "missing";
    v::LocalSource source("a", data);
    source.seek(100);
    auto snapshot = source.snapshot();
    auto config = settings("trajectory");
    const std::string fixed = "world";
    auto result = v::standardDisplays().draw({&snapshot, fixed, config});
    EXPECT_TRUE(result.lines.empty());
    EXPECT_EQ(result.level, v::Level::warning);
    source.seek(50);
    snapshot = source.snapshot();
    config.type = "pose";
    result = v::standardDisplays().draw({&snapshot, fixed, config});
    EXPECT_TRUE(result.lines.empty());
    EXPECT_EQ(result.level, v::Level::warning);
}
TEST(Displays, StalePosesDisconnectedSourcesAndUnknownDisplaysAreVisibleStatuses) {
    v::LocalSource source("a", recording());
    source.seek(75);
    auto snapshot = source.snapshot();
    auto config = settings("pose");
    config.max_age_ns = 10;
    const std::string fixed = "world";
    auto displays = v::standardDisplays();
    auto result = displays.draw({&snapshot, fixed, config});
    EXPECT_TRUE(result.lines.empty());
    EXPECT_NE(result.status.find("Stale"), std::string::npos);
    source.disconnect();
    snapshot = source.snapshot();
    EXPECT_TRUE(displays.draw({&snapshot, fixed, config}).lines.empty());
    config.type = "external";
    EXPECT_EQ(displays.draw({&snapshot, fixed, config}).level, v::Level::error);
    config.enabled = false;
    EXPECT_EQ(displays.draw({&snapshot, fixed, config}).status, "Hidden");
}
TEST(Session, ComposesExternalSourceAndDisplayAndIsolatesFailures) {
    auto factories = ui::localSources();
    factories["fixture"] = [](const std::filesystem::path &, const std::string &id) {
        auto source = std::make_shared<v::LocalSource>(id, recording());
        return ui::Connection{source, source, 100, [source] { source->reconnect(); }};
    };
    auto displays = v::standardDisplays();
    displays.add("custom", [](const v::DisplayContext &) {
        return v::DisplayResult{
            {{{0, 0, 0}, {1, 1, 1}, {255, 255, 255}}}, v::Level::ready, "Custom"};
    });
    ui::Session session(factories, displays);
    auto workspace = ui::emptyWorkspace();
    workspace.sources = {
        {"a", "fixture", {}}, {"b", "fixture", {}}, {"broken", "unregistered", {}}};
    workspace.selected_source = "a";
    workspace.displays.push_back(settings("trajectory"));
    auto external = settings("custom");
    external.id = "external";
    workspace.displays.push_back(external);
    session.open(workspace);
    ASSERT_EQ(session.errors().size(), 1);
    session.advance(std::numeric_limits<v::Time>::max());
    EXPECT_EQ(session.snapshot()->time_ns, 100);
    EXPECT_EQ(session.scene().status.at("external").second, "Custom");
    session.disconnect();
    EXPECT_FALSE(session.snapshot()->data);
    session.workspace().selected_source = "b";
    ASSERT_TRUE(session.snapshot()->data);
    EXPECT_EQ(session.snapshot()->time_ns, 0);
    session.workspace().selected_source = "a";
    session.reconnect();
    EXPECT_EQ(session.snapshot()->time_ns, 0);
    session.open(ui::emptyWorkspace());
    EXPECT_FALSE(session.snapshot());
    EXPECT_FALSE(session.scene().lines.empty());
}
TEST(Workspace, InstalledStylePathsRoundTripAndAtomicSave) {
    Temporary temporary;
    const auto source = std::filesystem::path(RP_WORKSPACES);
    std::filesystem::copy(source, temporary.directory / "input",
                          std::filesystem::copy_options::recursive);
    auto workspace = ui::loadWorkspace(temporary.directory / "input/local_demo.yaml");
    ASSERT_EQ(workspace.sources.size(), 2);
    EXPECT_TRUE(workspace.sources[0].file.is_absolute());
    workspace.camera.distance = 25;
    workspace.displays[0].enabled = false;
    std::filesystem::create_directory(temporary.directory / "saved");
    const auto saved = temporary.directory / "saved/workspace.yaml";
    ui::saveWorkspace(workspace, saved);
    const auto reopened = ui::loadWorkspace(saved);
    EXPECT_EQ(reopened.sources[0].file, workspace.sources[0].file);
    EXPECT_DOUBLE_EQ(reopened.camera.distance, 25);
    EXPECT_FALSE(reopened.displays[0].enabled);
    const auto data = ui::loadRecording(reopened.sources[0].file);
    EXPECT_EQ(data.duration_ns, 12000000000);
    EXPECT_EQ(data.data.streams.at("pose").size(), 61);
    workspace.camera.distance = -1;
    EXPECT_THROW(ui::saveWorkspace(workspace, saved), std::invalid_argument);
    EXPECT_DOUBLE_EQ(ui::loadWorkspace(saved).camera.distance, 25);
}
TEST(Workspace, RejectsUnknownDuplicateMalformedAndUnsupportedFields) {
    Temporary temporary;
    const auto path = temporary.directory / "invalid.yaml";
    for (const auto *text : {"version: 2", "version: 1\nversion: 1", "version: 1\nextra: true",
                             "version: 1\nsources: wrong"}) {
        {
            std::ofstream file(path);
            file << text;
        }
        EXPECT_THROW(ui::loadWorkspace(path), std::invalid_argument);
        EXPECT_THROW(ui::loadRecording(path), std::invalid_argument);
    }
}
TEST(Camera, TargetProjectsToCentreAndInvalidInputsFail) {
    v::Camera camera;
    camera.target = {3, 2, 1};
    const auto matrix = v::viewProjection(camera, 1.5);
    const Eigen::Vector4f clip = matrix * Eigen::Vector4f(3, 2, 1, 1);
    EXPECT_NEAR(clip.x() / clip.w(), 0, 1e-6);
    EXPECT_NEAR(clip.y() / clip.w(), 0, 1e-6);
    EXPECT_GT(clip.w(), 0);
    EXPECT_THROW(v::viewProjection(camera, 0), std::invalid_argument);
    camera.pitch = 2;
    EXPECT_THROW(v::viewProjection(camera, 1), std::invalid_argument);
}
TEST(Session, BadExtensionsDoNotHideOtherDisplays) {
    auto displays = v::standardDisplays();
    displays.add("bad", [](const v::DisplayContext &) {
        return v::DisplayResult{{{{0, 0, 0}, {1, 1, 1}, {-1, 255, 255}}}, v::Level::ready, {}};
    });
    displays.add("oversized", [](const v::DisplayContext &) {
        v::DisplayResult result;
        result.lines.resize(20001);
        return result;
    });
    auto workspace = ui::emptyWorkspace();
    for (const auto *type : {"bad", "oversized", "missing"}) {
        v::DisplaySettings display;
        display.id = type;
        display.type = type;
        workspace.displays.push_back(display);
    }
    ui::Session session({}, std::move(displays));
    session.open(workspace);
    const auto scene = session.scene();
    EXPECT_EQ(scene.lines.size(), 45);
    EXPECT_EQ(scene.status.at("Grid").first, v::Level::ready);
    for (const auto *id : {"bad", "oversized", "missing"})
        EXPECT_EQ(scene.status.at(id).first, v::Level::error);
}
TEST(Workspace, RejectsInvalidNestedValuesAndRetainsResolvedData) {
    Temporary temporary;
    auto workspace = ui::loadWorkspace(std::filesystem::path(RP_WORKSPACES) / "local_demo.yaml");
    const auto file = temporary.directory / "workspace.yaml";
    auto text = ui::serializeWorkspace(workspace, file);
    const auto position = text.find("distance:");
    ASSERT_NE(position, std::string::npos);
    text.replace(position, text.size() - position, "distance: .nan\n");
    {
        std::ofstream output(file);
        output << text;
    }
    EXPECT_THROW(ui::loadWorkspace(file), std::invalid_argument);
    const auto recording_path = temporary.directory / "recording.yaml";
    std::filesystem::copy_file(workspace.sources[0].file, recording_path);
    v::LocalSource source("resolved", ui::loadRecording(recording_path));
    std::filesystem::remove(recording_path);
    source.seek(source.duration());
    ASSERT_TRUE(source.snapshot().data);
    EXPECT_EQ(source.snapshot().data->streams.at("pose").size(), 61);
}
