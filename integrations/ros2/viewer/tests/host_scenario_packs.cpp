// The scenario packs offered by View > Pool, and resolving one with the project's pack tools.
#include "scenario_packs.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

using namespace nereus::ros_viewer::host;

TEST(ScenarioPacks, ListsEveryPoolWithItsLabel) {
    const auto packs = scenarioPacks();
    ASSERT_GE(packs.size(), 2u);
    bool robosub = false, rpac = false;
    for (const auto &pack : packs) {
        EXPECT_EQ(pack.robot, "talos");
        robosub = robosub || (pack.poolId == "robosub_2026" && pack.folder.filename() == "talos_uwrt" &&
                              pack.poolLabel == "RoboSub 2026 competition pool");
        rpac = rpac || (pack.poolId == "rpac_divewell" && pack.poolLabel == "Ohio State RPAC dive well");
    }
    EXPECT_TRUE(robosub && rpac);
}

TEST(ScenarioPacks, Labels) {
    EXPECT_EQ(poolLabel("Ohio State RPAC dive well, 25 m x 56 ft.", "rpac"), "Ohio State RPAC dive well");
    EXPECT_EQ(poolLabel("RoboSub 2026 competition pool.", "robosub"), "RoboSub 2026 competition pool");
    EXPECT_EQ(poolLabel("", "plain"), "plain");
}

TEST(ScenarioPacks, ResolvesAPack) {
    const auto packs = scenarioPacks();
    const auto rpac = std::find_if(packs.begin(), packs.end(), [](const auto &p) { return p.poolId == "rpac_divewell"; });
    ASSERT_NE(rpac, packs.end());
    std::string resolved;
    try {
        resolved = resolveScenarioPack(rpac->folder);
    } catch (const std::runtime_error &error) {
        if (std::string(error.what()).find("No module named") != std::string::npos)
            GTEST_SKIP() << "the pack tools are not installed (./build.sh sets up .venv): " << error.what();
        throw;
    }
    const auto document = YAML::Load(resolved);
    EXPECT_EQ(document["pool"]["id"].as<std::string>(), "rpac_divewell");
    EXPECT_TRUE(document["asset_paths"]);
    EXPECT_THROW(resolveScenarioPack(rpac->folder / "missing"), std::runtime_error);
}
