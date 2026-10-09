// The scenario packs offered by View > Pool, and resolving one (or writing its course) with the project's pack
// tools.
#include "scenario_packs.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

using namespace nereus::ros_viewer::host;

// Both shipped pools (RoboSub and the RPAC dive well) are listed for Talos with their menu labels.
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

// The menu label is the pool description up to its first comma, minus trailing periods; the id when empty.
TEST(ScenarioPacks, Labels) {
    EXPECT_EQ(poolLabel("Ohio State RPAC dive well, 25 m x 56 ft.", "rpac"), "Ohio State RPAC dive well");
    EXPECT_EQ(poolLabel("RoboSub 2026 competition pool.", "robosub"), "RoboSub 2026 competition pool");
    EXPECT_EQ(poolLabel("", "plain"), "plain");
}

// Resolving a pack runs the Python pack tools and yields the scenario YAML (skipped when they aren't installed).
TEST(ScenarioPacks, ResolvesAPack) {
    const auto packs = scenarioPacks();
    const auto rpac =
        std::find_if(packs.begin(), packs.end(), [](const auto &p) { return p.poolId == "rpac_divewell"; });
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

// Packs list their scenario's id (the viewer finds the pack it runs by scenario and pool).
TEST(ScenarioPacks, KnowTheirScenarioId) {
    for (const auto &pack : scenarioPacks())
        if (pack.folder.filename() == "talos_uwrt")
            EXPECT_EQ(pack.scenarioId, "talos_uwrt_repair_2026");
}

// A course edit goes through the pack tools' set-course: written when it resolves, else the tools' reason (a temp
// copy of the Talos scenario, its pack paths re-pointed at the shipped packs).
TEST(ScenarioPacks, SetsACourse) {
    namespace fs = std::filesystem;
    const auto talos = packContent() / "scenarios" / "talos_uwrt";
    const auto folder = fs::temp_directory_path() / ("nereus_course_" + std::to_string(getpid())) / "talos";
    fs::create_directories(folder);
    std::ifstream in(talos / "scenario.yaml");
    std::ofstream out(folder / "scenario.yaml");
    for (std::string line; std::getline(in, line);) {
        for (const char *role : {"robot: ", "pool: ", "tasks: ", "bridge: ", "equipment: "})
            if (line.rfind(role, 0) == 0) {
                const auto target = fs::weakly_canonical(talos / line.substr(std::strlen(role)));
                line = role + fs::relative(target, folder).string();
            }
        out << line << '\n';
    }
    out.close();

    try {
        setScenarioCourse(folder, R"({"run_options": {"bin_vinyl1_class": "fire"}})");
    } catch (const std::runtime_error &error) {
        if (std::string(error.what()).find("No module named") != std::string::npos)
            GTEST_SKIP() << "the pack tools are not installed (./build.sh sets up .venv): " << error.what();
        throw;
    }
    std::stringstream text;
    text << std::ifstream(folder / "scenario.yaml").rdbuf();
    EXPECT_NE(text.str().find("bin_vinyl1_class: fire"), std::string::npos);

    try {
        setScenarioCourse(folder, R"({"run_options": {"bin_vinyl1_class": "ink"}})");
        ADD_FAILURE() << "an option outside its choices was written";
    } catch (const std::runtime_error &error) {
        EXPECT_NE(std::string(error.what()).find("must be one of ['blood', 'fire']"), std::string::npos)
            << error.what();
    }
    fs::remove_all(folder.parent_path());
}
