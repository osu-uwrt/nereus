// Scenario pack discovery for View > Pool, and running the Python pack resolver on one.
#include "scenario_packs.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::host {
namespace fs = std::filesystem;
namespace {
// Single-quote for the shell, escaping embedded quotes as '\''.
std::string quoted(const std::string &text) { // for /bin/sh
    std::string out = "'";
    for (const char c : text)
        out += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return out + "'";
}
} // namespace

fs::path packContent() {
#ifdef NEREUS_PACK_CONTENT
    return NEREUS_PACK_CONTENT;
#else
    return {};
#endif
}

std::string poolLabel(const std::string &description, const std::string &id) {
    std::string label = description.substr(0, description.find(','));
    while (!label.empty() && (label.back() == '.' || label.back() == ' '))
        label.pop_back();
    return label.empty() ? id : label;
}

std::vector<ScenarioPack> scenarioPacks(const fs::path &scenarios) {
    const fs::path root = scenarios.empty() ? packContent() / "scenarios" : scenarios;
    std::vector<ScenarioPack> packs;
    std::error_code error;
    if (!fs::is_directory(root, error))
        return packs;

    // Each subfolder with a scenario.yaml: read its pool and robot packs for the menu entry.
    for (const auto &entry : fs::directory_iterator(root, error)) {
        const auto file = entry.path() / "scenario.yaml";
        if (!fs::is_regular_file(file, error))
            continue;
        try {
            const auto scenario = YAML::LoadFile(file.string());
            ScenarioPack pack;
            pack.folder = entry.path();
            const auto poolDir = entry.path() / scenario["pool"].as<std::string>();
            const auto pool = YAML::LoadFile((poolDir / "pool.yaml").string());
            pack.poolId = pool["id"].as<std::string>(poolDir.filename().string());
            pack.poolDescription = pool["description"].as<std::string>("");
            pack.poolLabel = poolLabel(pack.poolDescription, pack.poolId);
            const auto robotDir = entry.path() / scenario["robot"].as<std::string>();
            pack.robot = robotDir.filename().string();
            if (fs::is_regular_file(robotDir / "robot.yaml", error))
                pack.robot = YAML::LoadFile((robotDir / "robot.yaml").string())["id"].as<std::string>(pack.robot);
            packs.push_back(std::move(pack));
        } catch (const std::exception &) {
            // not a pack this viewer can offer (malformed or missing pool / robot)
        }
    }

    // By pool label, then folder.
    std::sort(packs.begin(), packs.end(), [](const ScenarioPack &a, const ScenarioPack &b) {
        return a.poolLabel != b.poolLabel ? a.poolLabel < b.poolLabel : a.folder < b.folder;
    });
    return packs;
}

std::string resolveScenarioPack(const fs::path &folder) {
    const fs::path root = packContent().parent_path().parent_path(); // <source>/content/packs
    if (root.empty())
        throw std::runtime_error("this viewer was built without the pack content folder");

    // The project's virtualenv Python when present.
    const fs::path venv = root / ".venv/bin/python";
    std::error_code error;
    const std::string python = fs::exists(venv, error) ? venv.string() : "python3";

    // A unique temporary file for the resolver's output (mkstemp creates it; the resolver overwrites it).
    std::array<char, 64> name{};
    std::snprintf(name.data(), name.size(), "nereus_viewer_%d_XXXXXX", int(getpid()));
    const std::string pattern = (fs::temp_directory_path() / name.data()).string();
    std::vector<char> path(pattern.begin(), pattern.end());
    path.push_back('\0');
    const int fd = mkstemp(path.data());
    if (fd < 0)
        throw std::runtime_error("cannot create a temporary file for the resolved scenario");
    close(fd);
    const fs::path out(path.data());

    // cd <source> && PYTHONPATH=<source>/python/src[:existing] python -m nereus.packs resolve <folder> -o <out>
    const char *existing = std::getenv("PYTHONPATH");
    const std::string command =
        "cd " + quoted(root.string()) +
        " && PYTHONPATH=" + quoted((root / "python/src").string() + (existing ? std::string(":") + existing : "")) +
        " " + quoted(python) + " -m nereus.packs resolve " + quoted(folder.string()) + " -o " + quoted(out.string()) +
        " 2>&1";

    // Run it, keeping the combined output; on failure report its last non-empty line.
    std::string output;
    if (FILE *pipe = popen(command.c_str(), "r")) {
        std::array<char, 512> buffer{};
        while (std::fgets(buffer.data(), int(buffer.size()), pipe))
            output += buffer.data();
        const int status = pclose(pipe);
        if (status != 0) {
            fs::remove(out, error);
            std::string last;
            std::istringstream lines(output);
            for (std::string line; std::getline(lines, line);)
                if (!line.empty())
                    last = line;
            throw std::runtime_error(last.empty() ? "the pack resolver failed" : last);
        }
    } else
        throw std::runtime_error("cannot run the pack resolver");

    // Read the resolved JSON and remove the temporary file.
    std::ifstream in(out);
    std::stringstream text;
    text << in.rdbuf();
    fs::remove(out, error);
    if (text.str().empty())
        throw std::runtime_error("the pack resolver wrote nothing");
    return text.str();
}
} // namespace nereus::ros_viewer::host
