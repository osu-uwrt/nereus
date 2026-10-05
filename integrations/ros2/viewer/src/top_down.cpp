#include "top_down.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace nereus::ros_viewer::host {
namespace {
struct Texture {
    std::vector<unsigned char> rgba;
    int width = 0, height = 0;
};
// Linear base colour to the sRGB the textures and the screen use.
float toSrgb(float c) {
    return std::pow(std::clamp(c, 0.f, 1.f), 1 / 2.2f);
}
} // namespace

std::pair<glm::vec2, glm::vec2> topDownBounds(const std::vector<TopDownPart> &parts) {
    glm::vec2 low(std::numeric_limits<float>::max()), high(std::numeric_limits<float>::lowest());
    for (const auto &part : parts)
        if (part.mesh)
            for (int corner = 0; corner < 8; ++corner) {
                const auto &a = part.mesh->minimum, &b = part.mesh->maximum;
                const glm::vec4 p = part.planeFromAsset * glm::vec4(corner & 1 ? b.x() : a.x(), corner & 2 ? b.y() : a.y(),
                                                                    corner & 4 ? b.z() : a.z(), 1);
                low = glm::min(low, glm::vec2(p));
                high = glm::max(high, glm::vec2(p));
            }
    return {low, high};
}

TopDownImage bakeTopDown(const std::vector<TopDownPart> &parts, glm::vec2 low, glm::vec2 high, float pixelsPerMetre,
                         const ImageReader &readImage, int maximumSide) {
    TopDownImage image;
    const glm::vec2 size = high - low;
    if (size.x <= 0 || size.y <= 0)
        return image;
    const float scale = std::min(pixelsPerMetre, float(maximumSide) / std::max(size.x, size.y));
    image.width = std::max(1, int(std::ceil(size.x * scale)));
    image.height = std::max(1, int(std::ceil(size.y * scale)));
    image.low = low;
    image.high = {low.x + float(image.width) / scale, low.y + float(image.height) / scale};
    const std::size_t pixels = std::size_t(image.width) * std::size_t(image.height);
    image.rgba.assign(pixels * 4, 0);
    std::vector<float> depth(pixels, std::numeric_limits<float>::lowest());
    std::map<std::filesystem::path, Texture> textures;
    const auto texture = [&](const std::filesystem::path &path) -> const Texture * {
        auto [found, fresh] = textures.try_emplace(path);
        if (fresh && (!readImage || !readImage(path, found->second.width, found->second.height, found->second.rgba)))
            found->second = {};
        return found->second.width > 0 ? &found->second : nullptr;
    };
    for (const auto &part : parts) {
        if (!part.mesh)
            continue;
        for (const auto &submesh : part.mesh->submeshes) {
            const Texture *map = submesh.material.diffuse_texture ? texture(*submesh.material.diffuse_texture) : nullptr;
            const auto &base = submesh.material.base_color;
            // a diffuse texture replaces the base colour (as the renderer draws it)
            const glm::vec3 flat = map ? glm::vec3(1) : glm::vec3(toSrgb(base.x()), toSrgb(base.y()), toSrgb(base.z()));
            if (!map && base.w() < .05f)
                continue;
            std::vector<glm::vec3> placed(submesh.vertices.size()), plane(submesh.vertices.size());
            for (std::size_t i = 0; i < placed.size(); ++i) {
                const auto &p = submesh.vertices[i].position;
                plane[i] = glm::vec3(part.planeFromAsset * glm::vec4(p.x(), p.y(), p.z(), 1));
                placed[i] = {(plane[i].x - low.x) * scale, (image.high.y - plane[i].y) * scale, plane[i].z}; // pixels
            }
            for (std::size_t t = 0; t + 2 < submesh.indices.size(); t += 3) {
                const std::uint32_t ia = submesh.indices[t], ib = submesh.indices[t + 1], ic = submesh.indices[t + 2];
                if (ia >= placed.size() || ib >= placed.size() || ic >= placed.size())
                    continue;
                const glm::vec3 a = placed[ia], b = placed[ib], c = placed[ic];
                const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
                if (std::abs(area) < 1e-9f)
                    continue; // edge-on from above: no footprint
                // shade by the face's slope: tops full, walls darker (the plane's z is "up")
                const glm::vec3 normal =
                    glm::normalize(glm::cross(plane[ib] - plane[ia], plane[ic] - plane[ia]));
                const float shade = .62f + .38f * std::abs(normal.z);
                const int x0 = std::max(0, int(std::floor(std::min({a.x, b.x, c.x})))),
                          x1 = std::min(image.width - 1, int(std::ceil(std::max({a.x, b.x, c.x})))),
                          y0 = std::max(0, int(std::floor(std::min({a.y, b.y, c.y})))),
                          y1 = std::min(image.height - 1, int(std::ceil(std::max({a.y, b.y, c.y}))));
                const auto &ua = submesh.vertices[ia].uv, &ub = submesh.vertices[ib].uv, &uc = submesh.vertices[ic].uv;
                for (int y = y0; y <= y1; ++y)
                    for (int x = x0; x <= x1; ++x) {
                        const float px = float(x) + .5f, py = float(y) + .5f;
                        const float wa = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) / area,
                                    wb = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) / area, wc = 1 - wa - wb;
                        if (wa < 0 || wb < 0 || wc < 0)
                            continue;
                        const float z = wa * a.z + wb * b.z + wc * c.z;
                        const std::size_t at = std::size_t(y) * std::size_t(image.width) + std::size_t(x);
                        if (z <= depth[at])
                            continue;
                        glm::vec3 colour = flat;
                        if (map) { // the texture through the UVs (v up, as the renderer flips its rows)
                            const float u = wa * ua.x() + wb * ub.x() + wc * uc.x(),
                                        v = wa * ua.y() + wb * ub.y() + wc * uc.y();
                            const int tx = ((int(std::floor((u - std::floor(u)) * float(map->width))) % map->width) +
                                            map->width) % map->width;
                            const int ty = ((int(std::floor((1 - (v - std::floor(v))) * float(map->height))) %
                                             map->height) + map->height) % map->height;
                            const auto *texel = &map->rgba[(std::size_t(ty) * std::size_t(map->width) + std::size_t(tx)) * 4];
                            if (texel[3] < 128)
                                continue;
                            colour = glm::vec3(texel[0], texel[1], texel[2]) / 255.f;
                        }
                        depth[at] = z;
                        colour *= shade;
                        for (int k = 0; k < 3; ++k)
                            image.rgba[at * 4 + k] = std::uint8_t(std::lround(std::clamp(colour[k], 0.f, 1.f) * 255));
                        image.rgba[at * 4 + 3] = 255;
                    }
            }
        }
    }
    return image;
}

namespace {
// A 1 px darker edge where a prop meets the floor (the outline that keeps small props crisp); thin features are
// left their own colour, as darkening them would dim them whole.
void darkenEdges(TopDownImage &image) {
    std::vector<std::uint8_t> edged = image.rgba;
    const auto opaque = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < image.width && y < image.height &&
               image.rgba[(std::size_t(y) * std::size_t(image.width) + std::size_t(x)) * 4 + 3] > 0;
    };
    for (int y = 0; y < image.height; ++y)
        for (int x = 0; x < image.width; ++x)
            // the border of something with a body; a feature only a pixel thin (a pole drawn small) keeps its colour
            if (opaque(x, y) && (!opaque(x + 1, y) || !opaque(x - 1, y) || !opaque(x, y + 1) || !opaque(x, y - 1)) &&
                (opaque(x + 1, y) || opaque(x - 1, y)) && (opaque(x, y + 1) || opaque(x, y - 1))) {
                const std::size_t at = (std::size_t(y) * std::size_t(image.width) + std::size_t(x)) * 4;
                for (int k = 0; k < 3; ++k)
                    edged[at + std::size_t(k)] = std::uint8_t(edged[at + std::size_t(k)] * 55 / 100);
            }
    image.rgba = std::move(edged);
}
// Half the size: opaque when any of the (up to) four pixels under it is, their opaque colours averaged.
TopDownImage halve(const TopDownImage &image) {
    TopDownImage half;
    half.width = std::max(1, image.width / 2);
    half.height = std::max(1, image.height / 2);
    half.low = image.low;
    half.high = image.high;
    half.rgba.assign(std::size_t(half.width) * std::size_t(half.height) * 4, 0);
    for (int y = 0; y < half.height; ++y)
        for (int x = 0; x < half.width; ++x) {
            int sum[3] = {0, 0, 0}, count = 0;
            for (int dy = 0; dy < 2; ++dy)
                for (int dx = 0; dx < 2; ++dx) {
                    const int sx = std::min(image.width - 1, 2 * x + dx), sy = std::min(image.height - 1, 2 * y + dy);
                    const auto *p = &image.rgba[(std::size_t(sy) * std::size_t(image.width) + std::size_t(sx)) * 4];
                    if (!p[3])
                        continue;
                    for (int k = 0; k < 3; ++k)
                        sum[k] += p[k];
                    ++count;
                }
            if (!count)
                continue;
            auto *q = &half.rgba[(std::size_t(y) * std::size_t(half.width) + std::size_t(x)) * 4];
            for (int k = 0; k < 3; ++k)
                q[k] = std::uint8_t(sum[k] / count);
            q[3] = 255;
        }
    return half;
}
} // namespace

std::vector<TopDownImage> topDownLevels(TopDownImage image) {
    std::vector<TopDownImage> levels;
    if (image.width == 0)
        return levels;
    levels.push_back(std::move(image));
    while (levels.back().width > 1 || levels.back().height > 1)
        levels.push_back(halve(levels.back())); // from the plain image (edges are added per level below)
    for (auto &level : levels)
        darkenEdges(level);
    return levels;
}
} // namespace nereus::ros_viewer::host
