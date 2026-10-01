#pragma once
// Renders a planner job's samples: one OffscreenRenderer, one PackScene per scenario, part labels and split
// rewrites resolved once at construction; per sample a randomized appearance, robot poses drawn until a view
// passes the label-pass acceptance, then RGB through the robot pack's camera model, a 16-bit id map and a
// record (§3, §4 of the dataset contract).
#include <nereus/datasets/job.hpp>
#include <nereus/datasets/mesh_parts.hpp>
#include <nereus/datasets/sampling.hpp>
#include <nereus/pack_scene/pack_scene.hpp>
#include <nereus/rendering/offscreen.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace nereus::datasets {
struct GeneratorOptions {
    std::filesystem::path shader_directory; // empty: $NEREUS_SHADER_DIR, then the build tree's shaders
    // Prefilter label pass resolution as a fraction of the output (A9; 1 = no prefilter). It only rejects views
    // whose target is far below the threshold; the full-resolution pass decides everything else, so the scale
    // trades rejected-attempt cost, not output.
    double acceptance_scale = .25;
    bool resume = true; // skip samples whose record, image and id map exist (non-empty)
};

// A scene instance with its parts (static, robot or prop).
struct SceneEntry {
    std::string role; // pool | task | equipment | robot | prop
    InstanceOrigin origin;
    std::string source; // task/prop/asset#visual-index, task/prop/asset#prop, pool/asset#i, ...
    InstanceParts parts;
    std::uint32_t label_id = 0; // composed scene index + 1 when it has parts, else 0
};

class Generator {
  public:
    Generator(Job, GeneratorOptions = {});
    ~Generator();
    const Job &job() const {
        return job_;
    }
    // Every scene instance of the first scenario: source, submesh materials / diffuse textures / parts.
    Json describe();
    // Renders sample k (if not already present) and returns its log line: status accepted | skipped | existing,
    // attempts, rejection reason counts, timings.
    Json render(std::int64_t k);
    const std::string &device() const;

  private:
    struct Scenario;
    Job job_;
    GeneratorOptions options_;
    std::unique_ptr<rendering::OffscreenRenderer> host_;
    std::vector<std::unique_ptr<Scenario>> scenarios_;
    SplitCache split_cache_;

    Scenario &scenario(std::size_t index);
};
} // namespace nereus::datasets
