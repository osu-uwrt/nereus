#include <nereus/datasets/environment.hpp>
#include <nereus/datasets/generator.hpp>
#include <nereus/datasets/job.hpp>
#include <nereus/datasets/mesh_parts.hpp>
#include <nereus/datasets/output.hpp>
#include <nereus/datasets/sampling.hpp>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace ds = nereus::datasets;
namespace r = nereus::rendering;
using ds::Json;
using ds::Pose;

namespace {
constexpr double kDeg = M_PI / 180;

ds::PoolFrame flatPool() {
    ds::PoolFrame pool;
    pool.length = 50;
    pool.width = 25;
    pool.surface_z = 2.1336;
    pool.floor = nereus::simulation::PoolFloor::flat(2.1336, 50);
    pool.world_from_pool.translation = {-10, -5, -2.1336}; // world z = 0 at the surface
    return pool;
}

// A forward camera 0.3 m ahead of and 0.05 m above the root, optical +Z along root +X (+X right, +Y down).
Pose forwardMount() {
    Pose mount;
    Eigen::Matrix3d axes;
    axes.col(0) = Eigen::Vector3d(0, -1, 0);
    axes.col(1) = Eigen::Vector3d(0, 0, -1);
    axes.col(2) = Eigen::Vector3d(1, 0, 0);
    mount.rotation = Eigen::Quaterniond(axes);
    mount.translation = {.3, 0, .05};
    return mount;
}
// A down camera: optical +Z along root -Z, slightly off 90 deg.
Pose downMount() {
    Pose mount;
    Eigen::Matrix3d axes;
    axes.col(0) = Eigen::Vector3d(0, -1, 0);
    axes.col(1) = Eigen::Vector3d(-1, 0, 0);
    axes.col(2) = Eigen::Vector3d(0, 0, -1);
    mount.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(3 * kDeg, Eigen::Vector3d::UnitY()) * axes);
    mount.translation = {.1, .02, -.15};
    return mount;
}

Json minimalJob() {
    return Json::parse(R"({
      "format": "nereus.dataset_job.v1", "dataset": "t", "seed": 7, "output": "/tmp/x",
      "scenarios": [{"id": "s", "resolved": "/tmp/s.json"}],
      "camera": {"sensor": "ffc", "resolution_px": [320, 200], "format": "png"},
      "parts": {"textures": [], "visuals": [
        {"task": "slalom", "asset": "pole", "prop": null, "frame": null, "part": null,
         "materials": {"Red": "pole_red", "White": "pole_white"}, "split": "connected", "indicator": null}]},
      "labelled": [{"task": "slalom", "part": "pole_red"}],
      "acceptance": {"max_range_m": 5.0, "min_target_px": 150, "near_m": 0.2, "max_near_fraction": 0.02,
                     "max_attempts": 50, "background_max_labelled_px": 0},
      "samples": [
        {"task": "torpedo", "count": 3, "sampler": {"type": "approach", "range_m": [1, 2], "bearing_deg": 30}},
        {"task": null, "count": 2, "sampler": {"type": "free", "depth_m": [0.5, 1.5]}}],
      "randomize": {"water": {"tint_scale": [0.9, 1.1]}, "time_s": [0, 10]}
    })");
}

// Two unit quads side by side sharing an edge by position only (duplicated vertices), plus one disjoint quad.
r::Submesh threeQuads() {
    r::Submesh submesh;
    const auto quad = [&](float x0, float y0) {
        const auto base = static_cast<std::uint32_t>(submesh.vertices.size());
        for (const auto &[dx, dy] : {std::pair{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}})
            submesh.vertices.push_back({{x0 + dx, y0 + dy, 0}, {0, 0, 1}, {dx, dy}});
        for (const auto i : {0u, 1u, 2u, 0u, 2u, 3u})
            submesh.indices.push_back(base + i);
    };
    quad(0, 0);
    quad(5, 0); // disjoint
    quad(1, 0); // touches the first quad's x = 1 edge
    return submesh;
}
} // namespace

TEST(DatasetSeeding, SplitmixReferenceAndStreams) {
    EXPECT_EQ(ds::splitmix64(0), 0xe220a8397b1dcdafull);
    ds::Stream a(7, 42), b(7, 42), c(7, 43), d(8, 42);
    const double first = a.uniform();
    EXPECT_EQ(first, b.uniform());
    EXPECT_NE(first, c.uniform());
    EXPECT_NE(first, d.uniform());
    // Sample k's stream depends only on (seed, k): drawing other samples first changes nothing.
    ds::Stream other(7, 3);
    for (int i = 0; i < 1000; ++i)
        other.uniform();
    ds::Stream again(7, 42);
    EXPECT_EQ(first, again.uniform());
    for (int i = 0; i < 10000; ++i) {
        const double u = a.uniform();
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
    }
}

TEST(DatasetJob, ParsesBlocksNamesAndParts) {
    const auto job = ds::parseJob(minimalJob());
    EXPECT_EQ(job.sampleCount(), 5);
    EXPECT_EQ(job.name(0), "torpedo_000000");
    EXPECT_EQ(job.name(3), "background_000003");
    EXPECT_EQ(job.block(4).sampler.type, "free");
    EXPECT_DOUBLE_EQ(job.block(0).sampler.bearing_deg, 30);
    EXPECT_EQ(job.camera.width, 320);
    ASSERT_EQ(job.visuals.size(), 1u);
    EXPECT_TRUE(job.visuals[0].split);
    EXPECT_EQ(job.visuals[0].materials.at("Red"), "pole_red");
    EXPECT_TRUE(job.labelled.count({"slalom", "pole_red"}));
    // A pre-§10 job: the top-level water/lighting/image/time_s become the one environment.
    ASSERT_EQ(job.randomize.environments.size(), 1u);
    EXPECT_FALSE(job.randomize.environments[0].exposure.has_value());
    EXPECT_DOUBLE_EQ(job.randomize.environments[0].tint_scale->hi, 1.1);
    EXPECT_TRUE(job.acceptance.reject_fragments);
    EXPECT_EQ(job.acceptance.min_fragment_px, 25);
    auto bad = minimalJob();
    bad["samples"][0]["sampler"]["type"] = "orbit";
    EXPECT_THROW(ds::parseJob(bad), std::runtime_error);
}

TEST(DatasetJob, PlacementGroups) {
    auto document = minimalJob();
    document["randomize"]["placement"] = Json::parse(R"({"task_yaw_deg": 10, "groups": [["surface", "table"]]})");
    const auto job = ds::parseJob(document);
    ASSERT_EQ(job.randomize.placement_groups.size(), 1u);
    EXPECT_EQ(job.randomize.placement_groups[0][1], "table");
    document["randomize"]["placement"]["groups"] = Json::parse(R"([["surface", "table"], ["table"]])");
    EXPECT_THROW(ds::parseJob(document), std::runtime_error);
}

TEST(DatasetEnvironment, WeightedSelectionUsesTheFirstDraw) {
    auto document = minimalJob();
    document["randomize"]["environments"] = Json::parse(R"([{"id": "clear", "weight": 3}, {"id": "murky", "weight": 1},
                                                            {"id": "never", "weight": 0}])");
    const auto job = ds::parseJob(document);
    ASSERT_EQ(job.randomize.environments.size(), 3u);
    EXPECT_FALSE(job.randomize.sweep);
    int counts[3] = {};
    for (std::int64_t k = 0; k < 8000; ++k) {
        ds::Stream rng(job.seed, k), fresh(job.seed, k);
        const auto picked = ds::selectEnvironment(job, k, rng);
        ++counts[picked];
        // Exactly one draw, the stream's first: the next draw is the fresh stream's second.
        const double first = fresh.uniform();
        EXPECT_EQ(rng.uniform(), fresh.uniform());
        EXPECT_EQ(picked, first * 4 < 3 ? 0u : 1u);
    }
    EXPECT_NEAR(counts[0] / 8000.0, .75, .02);
    EXPECT_NEAR(counts[1] / 8000.0, .25, .02);
    EXPECT_EQ(counts[2], 0);
}

TEST(DatasetEnvironment, SweepCyclesWithinEachBlock) {
    auto document = minimalJob(); // one scenario; blocks: 3 torpedo (k 0-2), 5 background (k 3-7)
    document["randomize"]["environments"] = Json::parse(R"([{"id": "a"}, {"id": "b"}])");
    document["randomize"]["environment_mode"] = "sweep";
    document["samples"][1]["count"] = 5;
    const auto job = ds::parseJob(document);
    EXPECT_TRUE(job.randomize.sweep);
    const std::vector<std::size_t> expected = {0, 1, 0, 0, 1, 0, 1, 0};
    for (std::int64_t k = 0; k < 8; ++k) {
        ds::Stream rng(job.seed, k), fresh(job.seed, k);
        EXPECT_EQ(ds::selectEnvironment(job, k, rng), expected[std::size_t(k)]) << k;
        EXPECT_EQ(rng.uniform(), fresh.uniform()); // no draw consumed
    }
    document["randomize"]["environment_mode"] = "random";
    EXPECT_THROW(ds::parseJob(document), std::runtime_error);
}

TEST(DatasetEnvironment, SweepGivesEveryScenarioEveryEnvironment) {
    auto document = minimalJob(); // one block of 8 torpedo samples, 2 scenarios, 2 environments
    document["scenarios"].push_back({{"id", "s2"}, {"resolved", "/tmp/s2.json"}});
    document["samples"] = {document["samples"][0]};
    document["samples"][0]["count"] = 8;
    document["randomize"]["environments"] = Json::parse(R"([{"id": "a"}, {"id": "b"}])");
    document["randomize"]["environment_mode"] = "sweep";
    const auto job = ds::parseJob(document);
    std::set<std::pair<std::int64_t, std::size_t>> seen; // (scenario, environment)
    for (std::int64_t k = 0; k < 8; ++k) {
        ds::Stream rng(job.seed, k);
        const auto environment = ds::selectEnvironment(job, k, rng);
        EXPECT_EQ(environment, std::size_t((k / 2) % 2)) << k;
        seen.emplace(k % 2, environment);
    }
    EXPECT_EQ(seen.size(), 4u);
}

TEST(DatasetEnvironment, RejectsNegativeAndOutOfRangeValues) {
    const auto parse = [](const char *environment) {
        auto document = minimalJob();
        document["randomize"]["environments"] = Json::array({Json::parse(environment)});
        return ds::parseJob(document);
    };
    EXPECT_NO_THROW(parse(R"({"id": "ok", "water": {"tint_rgb": [0, [0.2, 1], 0.5], "scattering": [0, 0.3]}})"));
    for (const char *bad :
         {R"({"id": "e", "water": {"scattering_scale": [-0.1, 1]}})",
          R"({"id": "e", "water": {"absorption_per_m_rgb": [0.1, -0.2, 0.3]}})",
          R"({"id": "e", "water": {"tint_rgb": [0.1, [0.2, 1.2], 0.3]}})",
          R"({"id": "e", "water": {"scattering": -0.1}})", R"({"id": "e", "lighting": {"exposure": [-1, 1]}})",
          R"({"id": "e", "lighting": {"glare": -0.5}})",
          R"({"id": "e", "lighting": {"ambient_light_scale": [-0.2, 1]}})",
          R"({"id": "e", "image": {"blur_px": [-1, 0]}})"}) {
        try {
            parse(bad);
            ADD_FAILURE() << "accepted " << bad;
        } catch (const std::runtime_error &error) {
            EXPECT_NE(std::string(error.what()).find("'e'"), std::string::npos) << error.what();
        }
    }
}

TEST(DatasetEnvironment, WaterIsRelativeToThePoolUnlessAbsolute) {
    nereus::rendering::Appearance pool;
    pool.water.tint = {.1f, .5f, .9f};
    pool.water.absorption = {.2f, .1f, .05f};
    pool.water.scattering = .458f;
    pool.water.distance_scale = 2;
    pool.direct_light = 2;
    pool.outdoor = false;
    auto document = minimalJob();
    document["randomize"]["environments"] = Json::parse(R"([
      {"id": "scaled", "water": {"tint_scale": [2, 2], "absorption_scale": 1.5, "scattering_scale": [2, 2],
                                 "distance_scale_scale": 0.5},
       "lighting": {"direct_light_scale": 0.5, "glare": [0.2, 0.2], "profile": "outdoor", "exposure": [0.7, 0.9]},
       "image": {"noise_sigma": 2}, "time_s": [5, 5]},
      {"id": "absolute", "water": {"scattering_scale": 3, "scattering": 0.3, "tint_rgb": [0.2, [0.3, 0.3], 0.4],
                                   "absorption_per_m_rgb": [0.5, 0.6, 0.7]}}])");
    const auto job = ds::parseJob(document);
    ds::Stream rng(1, 0);
    const auto scaled = ds::drawEnvironment(job.randomize.environments[0], pool, rng);
    const auto &w = scaled.appearance.water;
    EXPECT_FLOAT_EQ(w.scattering, .916f);
    EXPECT_FLOAT_EQ(w.tint.x(), .2f);
    EXPECT_FLOAT_EQ(w.tint.z(), 1.f); // clamped
    EXPECT_FLOAT_EQ(w.absorption.x(), .3f);
    EXPECT_FLOAT_EQ(w.distance_scale, 1.f);
    EXPECT_FLOAT_EQ(scaled.appearance.direct_light, 1.f);
    EXPECT_FLOAT_EQ(scaled.appearance.glare, .2f);
    EXPECT_TRUE(scaled.appearance.outdoor);
    EXPECT_GE(scaled.appearance.exposure, .7f);
    EXPECT_LE(scaled.appearance.exposure, .9f);
    EXPECT_DOUBLE_EQ(scaled.noise_sigma, 2);
    EXPECT_FLOAT_EQ(scaled.time, 5);
    EXPECT_EQ(scaled.record.at("lighting").at("profile"), "outdoor");
    const auto absolute = ds::drawEnvironment(job.randomize.environments[1], pool, rng);
    EXPECT_FLOAT_EQ(absolute.appearance.water.scattering, .3f); // the override wins over the scale
    EXPECT_FLOAT_EQ(absolute.appearance.water.tint.y(), .3f);
    EXPECT_FLOAT_EQ(absolute.appearance.water.absorption.z(), .7f);
    EXPECT_FALSE(absolute.appearance.outdoor); // the pool's profile
    EXPECT_FLOAT_EQ(absolute.appearance.direct_light, 2.f);
}

TEST(DatasetIntrinsics, ScalingKeepsFieldOfViewAndCropsCentre) {
    nereus::cameras::Intrinsics native;
    native.width = 1920, native.height = 1200, native.fx = 1864, native.fy = 1864, native.cx = 955.8, native.cy = 584.8;
    const auto half = ds::scaleIntrinsics(native, 960, 600);
    EXPECT_DOUBLE_EQ(half.fx, 932);
    EXPECT_DOUBLE_EQ(half.cx, (955.8 + .5) * .5 - .5);
    EXPECT_DOUBLE_EQ(half.cy, (584.8 + .5) * .5 - .5);
    // 16:9 from 16:10 at the same width: scale 1, 60 rows cropped top and bottom.
    const auto wide = ds::scaleIntrinsics(native, 1920, 1080);
    EXPECT_DOUBLE_EQ(wide.fx, 1864);
    EXPECT_DOUBLE_EQ(wide.cx, 955.8);
    EXPECT_DOUBLE_EQ(wide.cy, 584.8 - 60);
    // A ray lands on the same scene point: pixel edges map through (u + .5) s - .5 - offset.
    const Eigen::Vector3d ray(.3, -.2, 1);
    const double u = native.fx * ray.x() + native.cx, v = native.fy * ray.y() + native.cy;
    const auto square = ds::scaleIntrinsics(native, 600, 600); // s = .5, 180 columns cropped each side
    EXPECT_NEAR(square.fx * ray.x() + square.cx, (u + .5) * .5 - .5 - 180, 1e-9);
    EXPECT_NEAR(square.fy * ray.y() + square.cy, (v + .5) * .5 - .5, 1e-9);
}

TEST(DatasetSampler, ApproachLooksAtTargetWithinJitter) {
    const auto pool = flatPool();
    const auto mount = forwardMount();
    ds::Sampler sampler;
    sampler.type = "approach";
    sampler.frames = {"board"};
    sampler.range_m = {1, 4};
    sampler.bearing_deg = 50;
    sampler.elevation_deg = {-15, 15};
    sampler.aim_jitter_deg = 6;
    sampler.pitch_deg = 3;
    sampler.roll_deg = 4;
    Pose board;
    board.translation = {10, 3, -1.4};
    board.rotation = Eigen::AngleAxisd(30 * kDeg, Eigen::Vector3d::UnitZ());
    const std::map<std::string, Pose> frames{{"board", board}};
    int drawn = 0;
    for (std::int64_t k = 0; k < 300; ++k) {
        ds::Stream rng(1, k);
        const auto draw = ds::samplePose(sampler, rng, frames, pool, mount);
        if (!draw.world_from_root) {
            EXPECT_FALSE(draw.reason.empty());
            continue;
        }
        ++drawn;
        const Pose camera = nereus::spatial::compose(*draw.world_from_root, mount);
        // Root <-> camera mount round trip.
        const Pose back = nereus::spatial::compose(camera, nereus::spatial::inverse(mount));
        EXPECT_NEAR((back.translation - draw.world_from_root->translation).norm(), 0, 1e-9);
        const Eigen::Vector3d toTarget = board.translation - camera.translation;
        EXPECT_GE(toTarget.norm(), 1 - 1e-9);
        EXPECT_LE(toTarget.norm(), 4 + 1e-9);
        // In front of the board (+X side) within the bearing.
        const Eigen::Vector3d facing = board.rotation * Eigen::Vector3d::UnitX();
        const double bearing =
            std::acos(std::clamp((-toTarget).head<2>().normalized().dot(facing.head<2>().normalized()), -1.0, 1.0));
        EXPECT_LE(bearing, 50 * kDeg + 1e-9);
        // Optical axis within aim/pitch/roll jitter of the target direction.
        const Eigen::Vector3d axis = camera.rotation * Eigen::Vector3d::UnitZ();
        const double off = std::acos(std::clamp(axis.dot(toTarget.normalized()), -1.0, 1.0));
        EXPECT_LE(off, (6 + 3 + 1) * kDeg) << "sample " << k;
        // The camera stays in the water column (A8: drawn within the feasible elevations).
        EXPECT_FALSE(pool.checkCamera(camera.translation, .1 - 1e-9, 0, .2 - 1e-9).has_value());
    }
    EXPECT_GT(drawn, 290);
}

TEST(DatasetSampler, OverheadAndFree) {
    const auto pool = flatPool();
    const auto mount = downMount();
    ds::Sampler overhead;
    overhead.type = "overhead";
    overhead.frames = {"task"};
    overhead.altitude_m = {.8, 3.0}; // more than the water column allows: clamped
    overhead.radius_m = .3;
    overhead.roll_deg = overhead.pitch_deg = 2;
    Pose bin;
    bin.translation = {5, 5, -1.8};
    const std::map<std::string, Pose> frames{{"task", bin}};
    for (std::int64_t k = 0; k < 100; ++k) {
        ds::Stream rng(3, k);
        const auto draw = ds::samplePose(overhead, rng, frames, pool, mount);
        ASSERT_TRUE(draw.world_from_root) << draw.reason;
        const Pose camera = nereus::spatial::compose(*draw.world_from_root, mount);
        const Eigen::Vector3d offset = camera.translation - bin.translation;
        EXPECT_LE(offset.head<2>().norm(), .3 + 1e-9);
        EXPECT_GE(offset.z(), .8 - 1e-9);
        EXPECT_LE(camera.translation.z(), -.1 + 1e-9);
        const Eigen::Vector3d axis = camera.rotation * Eigen::Vector3d::UnitZ();
        EXPECT_LT(axis.z(), -std::cos((3 + 2 + 2 + .5) * kDeg));
    }
    ds::Sampler free;
    free.type = "free";
    free.depth_m = {.5, 1.5};
    for (std::int64_t k = 0; k < 100; ++k) {
        ds::Stream rng(3, k);
        const auto draw = ds::samplePose(free, rng, {}, pool, mount);
        ASSERT_TRUE(draw.world_from_root);
        const auto local = pool.toPool(draw.world_from_root->translation);
        EXPECT_GE(local.x(), .5);
        EXPECT_LE(local.y(), 24.5);
        EXPECT_GE(pool.surface_z - local.z(), .5 - 1e-9);
        EXPECT_LE(pool.surface_z - local.z(), 1.5 + 1e-9);
    }
}

TEST(DatasetPool, PoolLocalChecks) {
    auto pool = flatPool();
    pool.world_from_pool.rotation = Eigen::AngleAxisd(-90 * kDeg, Eigen::Vector3d::UnitZ());
    pool.world_from_pool.translation = {0, 19.5, 0}; // surface at world z = 2.1336
    const Eigen::Vector3d inside = pool.toWorld({10, 5, 1.0});
    EXPECT_NEAR(inside.x(), 5, 1e-9);
    EXPECT_NEAR(inside.y(), 9.5, 1e-9);
    EXPECT_FALSE(pool.checkCamera(inside).has_value());
    EXPECT_EQ(*pool.checkCamera(pool.toWorld({10, 5, 2.1})), "above_surface");
    EXPECT_EQ(*pool.checkCamera(pool.toWorld({10, 5, .1})), "near_floor");
    EXPECT_EQ(*pool.checkCamera(pool.toWorld({.2, 5, 1})), "outside_pool");
}

TEST(DatasetParts, ConnectedPiecesWeldPositions) {
    const auto submesh = threeQuads();
    const auto pieces = ds::connectedPieces(submesh);
    ASSERT_EQ(pieces.size(), 2u);
    EXPECT_EQ(pieces[0], (std::vector<std::size_t>{0, 1, 4, 5})); // first and third quads (shared edge)
    EXPECT_EQ(pieces[1], (std::vector<std::size_t>{2, 3}));
    const auto part = ds::subset(submesh, pieces[1]);
    EXPECT_EQ(part.vertices.size(), 4u);
    EXPECT_EQ(part.indices.size(), 6u);
    EXPECT_FLOAT_EQ(part.vertices[0].position.x(), 5);
}

TEST(DatasetParts, MaterialRulesSplitAndCache) {
    auto job = ds::parseJob(minimalJob());
    r::MeshAsset mesh;
    auto red = threeQuads();
    red.material.name = "Red";
    auto white = threeQuads();
    white.material.name = "White";
    auto other = threeQuads();
    other.material.name = "Other";
    mesh.submeshes = {red, white, other};
    const auto shared = std::make_shared<const r::MeshAsset>(mesh);
    ds::SplitCache cache;
    const auto parts = ds::resolveParts(shared, {"slalom", "poles", "pole", "slalom_front"}, job, &cache);
    ASSERT_TRUE(parts.rule.has_value());
    ASSERT_EQ(parts.mesh->submeshes.size(), 5u); // red: 2 pieces, white: 2 pieces, other unchanged
    ASSERT_EQ(parts.submeshes.size(), 5u);
    EXPECT_EQ(parts.submeshes[0].part, 1);
    EXPECT_EQ(parts.submeshes[1].part, 2);
    EXPECT_EQ(parts.submeshes[4].part, 0);
    ASSERT_EQ(parts.parts.size(), 4u);
    EXPECT_EQ(parts.parts[1].part, "pole_red");
    EXPECT_EQ(parts.parts[1].piece, 1);
    EXPECT_EQ(parts.parts[2].part, "pole_white");
    EXPECT_EQ(parts.parts[2].piece, 0);
    // A second instance of the same mesh reuses the rewrite.
    const auto again = ds::resolveParts(shared, {"slalom", "poles", "pole", "slalom_back"}, job, &cache);
    EXPECT_EQ(again.mesh, parts.mesh);
    // Unsplit: one value per part name.
    job.visuals[0].split = false;
    const auto whole = ds::resolveParts(shared, {"slalom", "poles", "pole", "x"}, job);
    EXPECT_EQ(whole.mesh, shared);
    EXPECT_EQ(whole.parts.size(), 2u);
    // Other tasks/assets: no parts.
    EXPECT_TRUE(ds::resolveParts(shared, {"gate", "g", "pole", "x"}, job).empty());
}

TEST(DatasetParts, TwoMatchingRulesThrow) {
    auto job = ds::parseJob(minimalJob());
    auto second = job.visuals[0];
    second.frame = "slalom_front";
    job.visuals.push_back(second);
    r::MeshAsset mesh;
    mesh.submeshes = {threeQuads()};
    const auto shared = std::make_shared<const r::MeshAsset>(mesh);
    EXPECT_THROW(ds::resolveParts(shared, {"slalom", "poles", "pole", "slalom_front"}, job), std::runtime_error);
    EXPECT_NO_THROW(ds::resolveParts(shared, {"slalom", "poles", "pole", "slalom_back"}, job));
}

TEST(DatasetParts, PartMasksHoldOnlyDeclaredValues) {
    const auto dir = std::filesystem::temp_directory_path() / "nereus_datasets_masks";
    std::filesystem::create_directories(dir);
    cv::Mat mask(4, 4, CV_8UC1, cv::Scalar(0));
    mask.at<std::uint8_t>(1, 1) = 1;
    mask.at<std::uint8_t>(2, 2) = 9;
    cv::imwrite((dir / "m.png").string(), mask);
    ds::Job job;
    ds::TexturePart texture;
    texture.mask = dir / "m.png";
    texture.values = {{1, "a"}};
    job.textures = {texture};
    EXPECT_THROW(ds::validatePartMasks(job), std::runtime_error);
    job.textures[0].values[9] = "b";
    EXPECT_NO_THROW(ds::validatePartMasks(job));
    std::filesystem::remove_all(dir);
}

TEST(DatasetParts, TexturePartMaps) {
    auto job = ds::parseJob(minimalJob());
    ds::TexturePart texture;
    texture.texture = "/abs/board.png";
    texture.mask = "/abs/board_parts.png";
    texture.values = {{1, "icon_fire"}, {5, "ring"}, {6, "ring"}};
    job.textures.push_back(texture);
    r::MeshAsset mesh;
    auto front = threeQuads();
    front.material.diffuse_texture = std::filesystem::path("/abs/board.png");
    mesh.submeshes = {threeQuads(), front};
    const auto parts =
        ds::resolveParts(std::make_shared<const r::MeshAsset>(mesh), {"torpedo", "board", "board_mesh", "task"}, job);
    ASSERT_EQ(parts.submeshes.size(), 2u);
    EXPECT_FALSE(parts.submeshes[0].part_map.has_value());
    EXPECT_EQ(parts.submeshes[0].part, 0);
    EXPECT_EQ(*parts.submeshes[1].part_map, std::filesystem::path("/abs/board_parts.png"));
    ASSERT_EQ(parts.parts.size(), 3u);
    EXPECT_EQ(parts.parts[2].value, 6);
    EXPECT_EQ(parts.parts[2].piece, 1);
}

TEST(DatasetOutput, ComponentsAndFragments) {
    r::LabelCapture capture;
    capture.width = 20, capture.height = 10;
    capture.ids.assign(200, 0);
    capture.depth.assign(200, .9755f);
    const std::uint32_t a = 1u << 8 | 1, b = 2u << 8 | 1;
    const auto paint = [&](std::uint32_t key, int x0, int y0, int w, int h) {
        for (int y = y0; y < y0 + h; ++y)
            for (int x = x0; x < x0 + w; ++x)
                capture.ids[std::size_t(y) * 20 + std::size_t(x)] = key;
    };
    paint(a, 0, 0, 6, 5);  // 30 px
    paint(a, 8, 0, 6, 5);  // 30 px, separated by a 2 px gap
    paint(a, 16, 8, 2, 1); // 2 px crumb
    paint(b, 0, 6, 3, 3);  // 9 px
    paint(b, 3, 9, 3, 1);  // touches the first block diagonally only: one 8-connected component
    auto stats = ds::analyzeLabels(capture, .05f, 100.f, .2f);
    ds::measureComponents(capture, stats);
    EXPECT_EQ(stats.keys.at(a).components, (std::vector<std::int64_t>{30, 30, 2}));
    EXPECT_EQ(stats.keys.at(b).components, (std::vector<std::int64_t>{12}));
    EXPECT_TRUE(stats.keys.at(a).fragmented(25, 25));
    EXPECT_FALSE(stats.keys.at(a).fragmented(25, 31));  // pieces below min_fragment_px are crumbs
    EXPECT_FALSE(stats.keys.at(a).fragmented(100, 25)); // below min_visible_px: ignored
    EXPECT_FALSE(stats.keys.at(b).fragmented(1, 1));
}

TEST(DatasetOutput, LabelStatsIdMapAndPng16) {
    r::LabelCapture capture;
    capture.width = 4, capture.height = 3;
    // Bottom-up rows: row 0 is the image bottom.
    capture.ids = {0, 0, 0, 0, /**/ 0, 1u << 8 | 3, 1u << 8 | 3, 0, /**/ 2u << 8, 0, 0, 1u << 8 | 3};
    capture.depth.assign(12, .9755f); // about 2 m
    capture.depth[0] = .0001f;        // near
    const auto stats = ds::analyzeLabels(capture, .05f, 100.f, .2f);
    EXPECT_EQ(stats.near_pixels, 1);
    EXPECT_EQ(stats.counted_pixels, 12);
    ASSERT_EQ(stats.keys.size(), 1u); // part 0 never counts
    const auto &item = stats.keys.at(1u << 8 | 3);
    EXPECT_EQ(item.pixels, 3);
    EXPECT_EQ(item.min_x, 1);
    EXPECT_EQ(item.max_x, 3);
    EXPECT_EQ(item.min_y, 0); // top row (bottom-up row 2)
    EXPECT_EQ(item.max_y, 1);
    EXPECT_TRUE(item.truncated);
    EXPECT_NEAR(item.medianDepth(), ds::linearDepth(.9755f, .05f, 100.f), 1e-6);
    EXPECT_NEAR(item.medianDepth(), 2.0, .01);
    const auto ids = ds::idMap(capture, {{1u << 8 | 3, 7}});
    EXPECT_EQ(ids[3], 7);     // top-right
    EXPECT_EQ(ids[4 + 1], 7); // middle row
    EXPECT_EQ(ids[0], 0);     // part-0 key
    const auto dir = std::filesystem::temp_directory_path() / "nereus_datasets_test";
    std::filesystem::create_directories(dir);
    ds::writeAtomic(dir / "ids.png", ds::encodePng16(ids, 4, 3));
    int w = 0, h = 0;
    EXPECT_EQ(ds::decodePng16(dir / "ids.png", w, h), ids);
    EXPECT_EQ(w, 4);
    for (const auto &entry : std::filesystem::directory_iterator(dir))
        EXPECT_EQ(entry.path().string().find(".tmp."), std::string::npos);
    std::filesystem::remove_all(dir);
}

// End to end on the real Talos/UWRT scenario: 2 torpedo samples at 320x200, rendered as one shard and as two
// single-sample generators, must be byte-identical.
namespace {
std::string slurp(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    return text.str();
}

Json talosJob(const std::filesystem::path &out) {
    const std::string tasks = std::string(NEREUS_SOURCE_DIR) + "/content/packs/tasks/robosub_2026/assets/torpedo/";
    Json job = {
        {"format", "nereus.dataset_job.v1"},
        {"dataset", "test"},
        {"seed", 11},
        {"output", out.string()},
        {"scenarios", {{{"id", "talos"}, {"resolved", NEREUS_RESOLVED_TALOS}}}},
        {"camera", {{"sensor", "ffc"}, {"resolution_px", {320, 200}}, {"format", "jpg"}, {"jpeg_quality", 90}}},
        {"parts",
         {{"textures",
           {{{"texture", tasks + "Task4_ver1_Fixed.png"},
             {"mask", tasks + "Task4_ver1_parts.png"},
             {"task", nullptr},
             {"values",
              {{"1", "icon_fire"},
               {"2", "icon_blood"},
               {"3", "icon_ambulance"},
               {"4", "icon_fire_engine"},
               {"5", "ring"},
               {"6", "ring"},
               {"7", "ring"},
               {"8", "ring"}}}}}},
          {"visuals", Json::array()}}},
        {"labelled", {{{"task", "torpedo"}, {"part", "icon_fire"}}, {{"task", "torpedo"}, {"part", "ring"}}}},
        {"acceptance", {{"max_range_m", 5.0}, {"min_target_px", 20}, {"max_attempts", 50}}},
        {"samples",
         {{{"task", "torpedo"},
           {"count", 2},
           {"sampler", {{"type", "approach"}, {"range_m", {1.0, 2.5}}, {"bearing_deg", 30}, {"aim_jitter_deg", 5}}}}}},
        {"randomize",
         {{"water", {{"tint_scale", {.9, 1.1}}}},
          {"time_s", {0, 100}},
          {"image", {{"noise_sigma", {0, 3}}, {"blur_px", {0, .5}}}}}}};
    return job;
}

std::unique_ptr<ds::Generator> makeGenerator(const Json &job, ds::GeneratorOptions options = {}) {
    try {
        return std::make_unique<ds::Generator>(ds::parseJob(job), options);
    } catch (const std::exception &error) {
        if (std::string(error.what()).find("EGL") != std::string::npos ||
            std::string(error.what()).find("GL") != std::string::npos)
            return nullptr;
        throw;
    }
}

void expectSameOutputs(const std::filesystem::path &a, const std::filesystem::path &b) {
    std::size_t files = 0;
    for (const char *folder : {"images", "ids", "records"})
        for (const auto &entry : std::filesystem::directory_iterator(a / folder)) {
            const auto other = b / folder / entry.path().filename();
            ASSERT_TRUE(std::filesystem::exists(other)) << other;
            EXPECT_EQ(slurp(entry.path()), slurp(other)) << entry.path().filename();
            ++files;
        }
    for (const char *folder : {"images", "ids", "records"})
        EXPECT_EQ(std::distance(std::filesystem::directory_iterator(a / folder), {}),
                  std::distance(std::filesystem::directory_iterator(b / folder), {}));
    EXPECT_GT(files, 0u);
}

std::filesystem::path freshOutput(const std::filesystem::path &dir) {
    std::filesystem::remove_all(dir);
    for (const char *folder : {"images", "ids", "records"})
        std::filesystem::create_directories(dir / folder);
    return dir;
}
} // namespace

TEST(DatasetEndToEnd, TalosTorpedoDeterministicAcrossShards) {
    if (!std::filesystem::exists(NEREUS_RESOLVED_TALOS))
        GTEST_SKIP() << "resolved Talos fixture missing (run the session_resolve_talos test)";
    const auto root = std::filesystem::temp_directory_path() / "nereus_datasets_e2e";
    std::filesystem::remove_all(root);
    const auto one = root / "one", two = root / "two";
    for (const auto &dir : {one, two})
        for (const char *folder : {"images", "ids", "records"})
            std::filesystem::create_directories(dir / folder);
    auto single = makeGenerator(talosJob(one));
    if (!single)
        GTEST_SKIP() << "no EGL";
    for (std::int64_t k = 0; k < 2; ++k)
        EXPECT_EQ(single->render(k).at("status"), "accepted");
    // Shard split: sample 1 first in its own generator, then sample 0 in another.
    for (const std::int64_t k : {1, 0}) {
        auto shard = makeGenerator(talosJob(two));
        EXPECT_EQ(shard->render(k).at("status"), "accepted");
    }
    for (const char *name : {"torpedo_000000", "torpedo_000001"}) {
        const auto a = Json::parse(slurp(one / "records" / (std::string(name) + ".json")));
        auto b = Json::parse(slurp(two / "records" / (std::string(name) + ".json")));
        EXPECT_EQ(a, b) << name;
        EXPECT_EQ(slurp(one / "images" / (std::string(name) + ".jpg")),
                  slurp(two / "images" / (std::string(name) + ".jpg")))
            << name;
        EXPECT_EQ(slurp(one / "ids" / (std::string(name) + ".png")), slurp(two / "ids" / (std::string(name) + ".png")))
            << name;
        // The record's instances are the id map's ids, and the target is in view.
        int w = 0, h = 0;
        const auto ids = ds::decodePng16(one / "ids" / (std::string(name) + ".png"), w, h);
        EXPECT_EQ(w, 320);
        EXPECT_EQ(h, 200);
        std::map<int, std::int64_t> counts;
        for (const auto id : ids)
            if (id)
                ++counts[id];
        ASSERT_FALSE(a.at("instances").empty()) << name;
        for (const auto &instance : a.at("instances")) {
            EXPECT_EQ(instance.at("task"), "torpedo");
            EXPECT_EQ(counts[instance.at("id").get<int>()], instance.at("pixels").get<std::int64_t>());
            EXPECT_LE(instance.at("depth_m").at("median").get<double>(), 5.0);
        }
        EXPECT_EQ(a.at("camera").at("width"), 320);
        EXPECT_EQ(a.at("environment"), "default");
        EXPECT_EQ(a.at("environment_index"), 0);
        for (const auto &instance : a.at("instances")) {
            EXPECT_GE(instance.at("components").get<int>(), 1);
            EXPECT_LE(instance.at("largest_component_px").get<std::int64_t>(),
                      instance.at("pixels").get<std::int64_t>());
        }
    }
    // Resume: a complete sample is skipped without rendering; one missing its image is rendered again, identically.
    EXPECT_EQ(single->render(0).at("status"), "existing");
    const auto image = one / "images" / "torpedo_000000.jpg";
    const auto before = slurp(image);
    std::filesystem::remove(image);
    EXPECT_EQ(single->render(0).at("status"), "accepted");
    EXPECT_EQ(slurp(image), before);
    std::ofstream(one / "ids" / "torpedo_000001.png", std::ios::trunc).flush(); // empty id map
    EXPECT_EQ(single->render(1).at("status"), "accepted");
    std::filesystem::remove_all(root);
}

// The reduced-resolution prefilter changes cost only: acceptance scale 0.25 and 1 give identical outputs, on
// views that hit every rejection rule (far range, slalom rows beyond the label range, thin poles).
TEST(DatasetEndToEnd, AcceptanceScaleDoesNotChangeOutput) {
    if (!std::filesystem::exists(NEREUS_RESOLVED_TALOS))
        GTEST_SKIP() << "resolved Talos fixture missing (run the session_resolve_talos test)";
    const auto root = std::filesystem::temp_directory_path() / "nereus_datasets_scale";
    const auto jobFor = [&](const std::filesystem::path &out) {
        auto job = talosJob(out);
        job["acceptance"] = {{"max_range_m", 5.0}, {"min_target_px", 150}, {"max_attempts", 60}};
        job["samples"][0]["count"] = 3;
        job["samples"][0]["sampler"]["range_m"] = {0.8, 6.0};
        job["samples"].push_back({{"task", "slalom"},
                                  {"count", 3},
                                  {"sampler",
                                   {{"type", "approach"},
                                    {"frame", {"slalom_front", "slalom_middle", "slalom_back"}},
                                    {"both_sides", true},
                                    {"range_m", {0.8, 4.0}},
                                    {"bearing_deg", 80},
                                    {"aim_jitter_deg", 20}}}});
        job["samples"].push_back({{"task", nullptr}, {"count", 2}, {"sampler", {{"type", "free"}}}});
        job["parts"]["visuals"] = {{{"task", "slalom"},
                                    {"asset", "slalom_mesh"},
                                    {"materials", {{"Material.001", "pole_red"}, {"Material.002", "pole_white"}}}}};
        job["labelled"].push_back({{"task", "slalom"}, {"part", "pole_red"}});
        return job;
    };
    const auto quarter = freshOutput(root / "quarter"), full = freshOutput(root / "full");
    ds::GeneratorOptions reduced, exact;
    reduced.acceptance_scale = .25;
    exact.acceptance_scale = 1;
    auto a = makeGenerator(jobFor(quarter), reduced);
    if (!a)
        GTEST_SKIP() << "no EGL";
    auto b = makeGenerator(jobFor(full), exact);
    for (std::int64_t k = 0; k < 8; ++k) {
        const auto x = a->render(k), y = b->render(k);
        EXPECT_EQ(x.at("status"), y.at("status")) << k;
        EXPECT_EQ(x.value("attempts", 0), y.value("attempts", 0)) << k;
    }
    expectSameOutputs(quarter, full);
    std::filesystem::remove_all(root);
}

// Grouped placement jitter: tasks in one group share a draw and keep their relative pose (the table stays
// under the octagon).
TEST(DatasetEndToEnd, PlacementGroupsMoveTogether) {
    if (!std::filesystem::exists(NEREUS_RESOLVED_TALOS))
        GTEST_SKIP() << "resolved Talos fixture missing (run the session_resolve_talos test)";
    const auto out = freshOutput(std::filesystem::temp_directory_path() / "nereus_datasets_groups");
    auto job = talosJob(out);
    job["samples"] = {{{"task", nullptr}, {"count", 1}, {"sampler", {{"type", "free"}}}}};
    job["randomize"]["placement"] =
        Json::parse(R"({"task_yaw_deg": 30, "task_offset_m": 0.5, "groups": [["surface", "table"]]})");
    auto generator = makeGenerator(job);
    if (!generator)
        GTEST_SKIP() << "no EGL";
    ASSERT_EQ(generator->render(0).at("status"), "accepted");
    const auto record = Json::parse(slurp(out / "records" / "background_000000.json"));
    const auto &placement = record.at("randomization").at("placement");
    EXPECT_EQ(placement.at("surface").at("yaw_deg"), placement.at("table").at("yaw_deg"));
    EXPECT_EQ(placement.at("table").at("pivot_task"), "surface");
    EXPECT_NE(placement.at("gate").at("yaw_deg"), placement.at("surface").at("yaw_deg"));
    const auto pose = [](const Json &item) {
        Pose p;
        const auto &t = item.at("position_m"), &q = item.at("orientation_wxyz");
        p.translation = {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()};
        p.rotation = Eigen::Quaterniond(q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>());
        return p;
    };
    const auto &tasks = record.at("tasks");
    const auto surfaceFromTable =
        nereus::spatial::compose(nereus::spatial::inverse(pose(tasks.at("surface").at("camera_from_task"))),
                                 pose(tasks.at("table").at("camera_from_task")));
    // Unjittered relative pose from the scenario's placements.
    const auto scenario = Json::parse(slurp(NEREUS_RESOLVED_TALOS));
    std::map<std::string, Pose> placed;
    for (const auto &item : scenario.at("scenario").at("task_placements")) {
        Pose p;
        const auto &t = item.at("position_m");
        p.translation = {t[0].get<double>(), t[1].get<double>(), t[2].get<double>()};
        p.rotation = Eigen::AngleAxisd(item.at("yaw_deg").get<double>() * kDeg, Eigen::Vector3d::UnitZ());
        placed[item.at("task").get<std::string>()] = p;
    }
    const auto expected = nereus::spatial::compose(nereus::spatial::inverse(placed.at("surface")), placed.at("table"));
    EXPECT_NEAR((surfaceFromTable.translation - expected.translation).norm(), 0, 1e-5);
    EXPECT_NEAR(surfaceFromTable.rotation.angularDistance(expected.rotation), 0, 1e-5);
    std::filesystem::remove_all(out);
}
