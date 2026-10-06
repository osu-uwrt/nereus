#pragma once
// Parts of one scene instance: which submeshes carry which part (fixed values or texture part maps), the mesh
// rewrite a `split: connected` rule needs, and the label-pass submesh labels. Pure CPU; no GL.
#include <nereus/datasets/job.hpp>
#include <nereus/rendering/renderer.hpp>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace nereus::datasets {
// Triangles of a submesh grouped into connected pieces: triangles sharing a vertex position (welded at
// `tolerance` metres) belong to one piece. Pieces are ordered by their first triangle; each lists triangle
// indices (into indices / 3) in increasing order.
std::vector<std::vector<std::size_t>> connectedPieces(const rendering::Submesh &, double tolerance = 1e-6);

// The submesh restricted to some triangles, with its vertices compacted (first-use order) and material kept.
rendering::Submesh subset(const rendering::Submesh &, const std::vector<std::size_t> &triangles);

// One labelled thing in an instance: a part-map value or a fixed part value.
struct PartInstance {
    std::uint8_t value = 0;
    std::string part;
    int piece = 0;         // index among this instance's instances of the same part
    std::string indicator; // indicator region id (rule `indicator`), or empty
};

// Where a scene instance came from, for matching `parts.visuals` / `parts.textures` rules.
struct InstanceOrigin {
    std::string task, prop, asset, frame; // task empty: pool, equipment or robot (texture rules without task only)
};

// Result of resolveParts: the mesh to draw and its per-submesh labels.
struct InstanceParts {
    std::shared_ptr<const rendering::MeshAsset> mesh; // the input mesh, or its split rewrite
    std::vector<rendering::SubmeshLabel> submeshes;   // empty when the instance has no parts
    std::vector<PartInstance> parts;                  // by increasing value
    std::optional<std::size_t> rule;                  // index of the matching parts.visuals rule

    bool empty() const {
        return parts.empty();
    }
};

// Throws when a part map contains a nonzero value its texture entry does not declare (or is not an 8-bit PNG).
// With that guaranteed, rule-assigned values allocated above the declared maximum never collide with map values.
void validatePartMasks(const Job &);

// Canonical diffuse texture path of a submesh (empty when untextured).
std::filesystem::path canonicalTexture(const rendering::Submesh &);

// A mesh rewritten for a `split: connected` rule: each selected submesh replaced by its connected pieces.
struct SplitMesh {
    std::shared_ptr<const rendering::MeshAsset> mesh;
    std::vector<std::size_t> origin; // per rewritten submesh: the input submesh it came from
};

// Split rewrites keyed by (input mesh, parts.visuals rule index).
using SplitCache = std::map<std::pair<const rendering::MeshAsset *, std::size_t>, SplitMesh>;

// Resolves an instance's parts. Visual rules take precedence on the submeshes they select (materials listed,
// or every submesh for `part`); remaining textured submeshes take the part map of their texture. Fixed rule
// values are allocated after the largest declared part-map value used. Throws when two visual rules match,
// when two part maps of one instance share a value or more than 255 values are needed. `split_cache` (optional) shares
// split rewrites of one mesh under one rule across instances (rewrites are made once, never per sample).
InstanceParts resolveParts(const std::shared_ptr<const rendering::MeshAsset> &, const InstanceOrigin &, const Job &,
                           SplitCache *split_cache = nullptr);
} // namespace nereus::datasets
