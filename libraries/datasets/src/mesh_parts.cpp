#include <nereus/datasets/mesh_parts.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace nereus::datasets {
namespace r = nereus::rendering;
namespace {
struct Key {
    std::int64_t x, y, z;
    bool operator==(const Key &o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};
struct KeyHash {
    std::size_t operator()(const Key &k) const {
        std::uint64_t h = static_cast<std::uint64_t>(k.x) * 0x9e3779b97f4a7c15ull;
        h ^= static_cast<std::uint64_t>(k.y) + 0x7f4a7c15ull + (h << 6) + (h >> 2);
        h ^= static_cast<std::uint64_t>(k.z) + 0x94d049bbull + (h << 6) + (h >> 2);
        return static_cast<std::size_t>(h);
    }
};

std::size_t findRoot(std::vector<std::size_t> &parent, std::size_t i) {
    while (parent[i] != i)
        i = parent[i] = parent[parent[i]];
    return i;
}

bool matches(const VisualPart &rule, const InstanceOrigin &origin) {
    return !origin.task.empty() && rule.task == origin.task && rule.asset == origin.asset &&
           (!rule.prop || *rule.prop == origin.prop) && (!rule.frame || *rule.frame == origin.frame);
}

SplitMesh split(const r::MeshAsset &mesh, const std::vector<bool> &selected) {
    r::MeshAsset out;
    out.minimum = mesh.minimum;
    out.maximum = mesh.maximum;
    out.dependencies = mesh.dependencies;
    SplitMesh result;
    for (std::size_t s = 0; s < mesh.submeshes.size(); ++s) {
        if (!selected[s]) {
            out.submeshes.push_back(mesh.submeshes[s]);
            result.origin.push_back(s);
            continue;
        }
        for (const auto &piece : connectedPieces(mesh.submeshes[s])) {
            out.submeshes.push_back(subset(mesh.submeshes[s], piece));
            result.origin.push_back(s);
        }
    }
    result.mesh = std::make_shared<const r::MeshAsset>(std::move(out));
    return result;
}
} // namespace

std::vector<std::vector<std::size_t>> connectedPieces(const r::Submesh &submesh, double tolerance) {
    const std::size_t vertices = submesh.vertices.size(), triangles = submesh.indices.size() / 3;
    // Weld vertex positions on a `tolerance` grid, then union the three vertices of every triangle.
    std::vector<std::size_t> parent(vertices);
    std::iota(parent.begin(), parent.end(), std::size_t(0));
    std::unordered_map<Key, std::size_t, KeyHash> welded;
    welded.reserve(vertices);
    const auto unite = [&](std::size_t a, std::size_t b) {
        a = findRoot(parent, a), b = findRoot(parent, b);
        if (a != b)
            parent[std::max(a, b)] = std::min(a, b);
    };
    for (std::size_t v = 0; v < vertices; ++v) {
        const auto &p = submesh.vertices[v].position;
        const Key key{std::llround(double(p.x()) / tolerance), std::llround(double(p.y()) / tolerance),
                      std::llround(double(p.z()) / tolerance)};
        const auto [found, inserted] = welded.emplace(key, v);
        if (!inserted)
            unite(found->second, v);
    }
    for (std::size_t t = 0; t < triangles; ++t) {
        const auto *index = &submesh.indices[3 * t];
        if (index[0] >= vertices || index[1] >= vertices || index[2] >= vertices)
            throw std::invalid_argument("submesh index out of range");
        unite(index[0], index[1]);
        unite(index[0], index[2]);
    }
    std::vector<std::vector<std::size_t>> pieces;
    std::unordered_map<std::size_t, std::size_t> pieceOf;
    for (std::size_t t = 0; t < triangles; ++t) {
        const auto root = findRoot(parent, submesh.indices[3 * t]);
        const auto [found, inserted] = pieceOf.emplace(root, pieces.size());
        if (inserted)
            pieces.emplace_back();
        pieces[found->second].push_back(t);
    }
    return pieces;
}

r::Submesh subset(const r::Submesh &submesh, const std::vector<std::size_t> &triangles) {
    r::Submesh out;
    out.material = submesh.material;
    std::unordered_map<std::uint32_t, std::uint32_t> remap;
    out.indices.reserve(triangles.size() * 3);
    for (const auto t : triangles)
        for (std::size_t c = 0; c < 3; ++c) {
            const auto index = submesh.indices.at(3 * t + c);
            const auto [found, inserted] = remap.emplace(index, static_cast<std::uint32_t>(out.vertices.size()));
            if (inserted)
                out.vertices.push_back(submesh.vertices.at(index));
            out.indices.push_back(found->second);
        }
    return out;
}

void validatePartMasks(const Job &job) {
    std::set<std::filesystem::path> checked;
    for (const auto &texture : job.textures) {
        if (!checked.insert(texture.mask).second)
            continue;
        const cv::Mat image = cv::imread(texture.mask.string(), cv::IMREAD_UNCHANGED);
        if (image.empty() || image.depth() != CV_8U)
            throw std::runtime_error("part map " + texture.mask.string() + " is not an 8-bit PNG");
        bool present[256] = {};
        const int channels = image.channels();
        for (int y = 0; y < image.rows; ++y) {
            const auto *row = image.ptr<std::uint8_t>(y);
            for (int x = 0; x < image.cols; ++x)
                present[row[x * channels]] = true; // the label pass reads the first channel
        }
        for (int v = 1; v < 256; ++v)
            if (present[v] && !texture.values.count(v))
                throw std::runtime_error("part map " + texture.mask.string() + " contains value " + std::to_string(v) +
                                         ", which parts.textures does not declare");
    }
}

std::filesystem::path canonicalTexture(const r::Submesh &submesh) {
    if (!submesh.material.diffuse_texture)
        return {};
    std::error_code error;
    auto path = std::filesystem::weakly_canonical(*submesh.material.diffuse_texture, error);
    return error ? submesh.material.diffuse_texture->lexically_normal() : path;
}

InstanceParts resolveParts(const std::shared_ptr<const r::MeshAsset> &mesh, const InstanceOrigin &origin,
                           const Job &job, SplitCache *cache) {
    InstanceParts out;
    out.mesh = mesh;
    if (!mesh)
        return out;
    for (std::size_t i = 0; i < job.visuals.size(); ++i)
        if (matches(job.visuals[i], origin)) {
            if (out.rule)
                throw std::runtime_error("parts.visuals rules " + std::to_string(*out.rule) + " and " +
                                         std::to_string(i) + " both match " + origin.task + "/" + origin.prop + "/" +
                                         origin.asset + (origin.frame.empty() ? "" : "@" + origin.frame));
            out.rule = i;
        }
    const VisualPart *rule = out.rule ? &job.visuals[*out.rule] : nullptr;
    const std::size_t n = mesh->submeshes.size();
    const std::string where = (origin.task.empty() ? std::string("?") : origin.task) + "/" + origin.prop + "/" +
                              origin.asset + (origin.frame.empty() ? "" : "@" + origin.frame);

    std::vector<std::optional<std::string>> ruled(n);
    std::vector<const TexturePart *> mapped(n, nullptr);
    std::map<int, std::pair<std::string, const TexturePart *>> textureValues;
    for (std::size_t s = 0; s < n; ++s) {
        const auto &submesh = mesh->submeshes[s];
        if (rule) {
            if (rule->part)
                ruled[s] = rule->part;
            else if (const auto found = rule->materials.find(submesh.material.name); found != rule->materials.end())
                ruled[s] = found->second;
        }
        if (ruled[s])
            continue;
        const auto texture = canonicalTexture(submesh);
        if (texture.empty())
            continue;
        for (const auto &candidate : job.textures)
            if (candidate.texture == texture && (!candidate.task || *candidate.task == origin.task)) {
                mapped[s] = &candidate;
                break;
            }
        if (!mapped[s])
            continue;
        for (const auto &[value, part] : mapped[s]->values) {
            const auto [found, inserted] = textureValues.emplace(value, std::pair{part, mapped[s]});
            if (!inserted && found->second.second != mapped[s])
                throw std::runtime_error(where + ": part maps " + found->second.second->mask.string() + " and " +
                                         mapped[s]->mask.string() + " share value " + std::to_string(value));
        }
    }

    // Split rewrite (once per mesh and rule when a cache is given).
    std::vector<std::size_t> originOf(n);
    std::iota(originOf.begin(), originOf.end(), std::size_t(0));
    bool anyRuled = false;
    for (const auto &item : ruled)
        anyRuled = anyRuled || item.has_value();
    if (rule && rule->split && anyRuled) {
        std::vector<bool> selected(n);
        for (std::size_t s = 0; s < n; ++s)
            selected[s] = ruled[s].has_value();
        const std::pair<const r::MeshAsset *, std::size_t> key{mesh.get(), *out.rule};
        SplitMesh rewritten;
        if (cache && cache->count(key))
            rewritten = cache->at(key);
        else {
            rewritten = split(*mesh, selected);
            if (cache)
                (*cache)[key] = rewritten;
        }
        out.mesh = rewritten.mesh;
        originOf = rewritten.origin;
    }

    int next = textureValues.empty() ? 1 : textureValues.rbegin()->first + 1;
    std::map<std::string, int> pieces;
    std::map<std::string, std::uint8_t> fixed; // unsplit rule part -> its value
    const auto allocate = [&](const std::string &part) {
        if (next > 255)
            throw std::runtime_error(where + ": more than 255 part values");
        out.parts.push_back({static_cast<std::uint8_t>(next), part, pieces[part]++,
                             rule && rule->indicator ? *rule->indicator : std::string()});
        return static_cast<std::uint8_t>(next++);
    };
    out.submeshes.resize(originOf.size());
    for (std::size_t j = 0; j < originOf.size(); ++j) {
        const auto s = originOf[j];
        auto &label = out.submeshes[j];
        if (ruled[s]) {
            if (rule->split)
                label.part = allocate(*ruled[s]);
            else {
                const auto found = fixed.find(*ruled[s]);
                label.part = found != fixed.end() ? found->second : (fixed[*ruled[s]] = allocate(*ruled[s]));
            }
        } else if (mapped[s])
            label.part_map = mapped[s]->mask;
    }
    std::map<std::string, int> texturePieces;
    for (const auto &[value, entry] : textureValues)
        out.parts.push_back({static_cast<std::uint8_t>(value), entry.first, texturePieces[entry.first]++, {}});
    std::sort(out.parts.begin(), out.parts.end(),
              [](const PartInstance &a, const PartInstance &b) { return a.value < b.value; });
    if (out.parts.empty())
        out.submeshes.clear();
    return out;
}
} // namespace nereus::datasets
