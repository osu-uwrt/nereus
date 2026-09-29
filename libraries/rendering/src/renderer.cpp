#include "robotics/rendering/renderer.hpp"
#include <Eigen/LU>
#include <GL/glew.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <limits>
#include <map>
#include <png.h>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace robotics::rendering {
namespace {
glm::mat4 matrix(const Eigen::Matrix4f &m) {
    return glm::make_mat4(m.data());
}
glm::vec3 vector(const Eigen::Vector3f &v) {
    return {v.x(), v.y(), v.z()};
}
struct InternalView {
    glm::mat4 view, projection;
    glm::vec3 eye;
};
struct Look {
    struct Water {
        glm::vec3 tint, absorption;
        float scattering, distanceScale, distancePower, clearDistance;
    } water;
    float caustics, exposure;
    bool surface, shadows, surfaceReflections, outdoor;
    float sunAzimuth, sunElevation, directLight, ambientLight, glare;
    glm::vec3 sunDirection() const {
        float a = glm::radians(sunAzimuth), e = glm::radians(sunElevation);
        return {cos(e) * cos(a), cos(e) * sin(a), sin(e)};
    }
};
struct Bounds {
    glm::vec3 min{std::numeric_limits<float>::max()}, max{std::numeric_limits<float>::lowest()};
    void include(const glm::vec3 &p) {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
};
class Frustum {
  public:
    explicit Frustum(const glm::mat4 &clip) {
        const auto rows = glm::transpose(clip);
        for (int axis = 0; axis < 3; ++axis) {
            planes[2 * axis] = rows[3] + rows[axis];
            planes[2 * axis + 1] = rows[3] - rows[axis];
        }
    }
    bool intersects(const Bounds &b) const {
        if (glm::any(glm::greaterThan(b.min, b.max)))
            return false;
        for (const auto &p : planes) {
            const glm::vec3 n(p);
            const glm::vec3 s(n.x >= 0 ? b.max.x : b.min.x, n.y >= 0 ? b.max.y : b.min.y,
                              n.z >= 0 ? b.max.z : b.min.z);
            const float tolerance =
                1e-5f * (glm::dot(glm::abs(n), glm::abs(s)) + glm::abs(p.w) + 1.f);
            if (glm::dot(n, s) + p.w < -tolerance)
                return false;
        }
        return true;
    }

  private:
    std::array<glm::vec4, 6> planes;
};
GLuint program(const std::filesystem::path &root, const std::string &name) {
    const GLuint p = glCreateProgram();
    try {
        for (auto type : {GL_VERTEX_SHADER, GL_FRAGMENT_SHADER}) {
            const auto path = root / (name + (type == GL_VERTEX_SHADER ? ".vert" : ".frag"));
            std::ifstream input(path);
            if (!input)
                throw std::runtime_error("missing shader: " + path.string());
            const std::string source((std::istreambuf_iterator<char>(input)), {});
            const char *text = source.c_str();
            const GLuint shader = glCreateShader(type);
            glShaderSource(shader, 1, &text, nullptr);
            glCompileShader(shader);
            GLint ok = 0;
            glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
            if (!ok) {
                std::array<char, 4096> log{};
                glGetShaderInfoLog(shader, log.size(), nullptr, log.data());
                glDeleteShader(shader);
                throw std::runtime_error(path.string() + ": " + log.data());
            }
            glAttachShader(p, shader);
            glDeleteShader(shader);
        }
        glLinkProgram(p);
        GLint ok = 0;
        glGetProgramiv(p, GL_LINK_STATUS, &ok);
        if (!ok) {
            std::array<char, 4096> log{};
            glGetProgramInfoLog(p, log.size(), nullptr, log.data());
            throw std::runtime_error(name + ": " + log.data());
        }
        return p;
    } catch (...) {
        glDeleteProgram(p);
        throw;
    }
}
void uniform(GLuint p, const char *n, const glm::mat4 &v) {
    glUniformMatrix4fv(glGetUniformLocation(p, n), 1, GL_FALSE, glm::value_ptr(v));
}
void uniform(GLuint p, const char *n, const glm::vec3 &v) {
    glUniform3fv(glGetUniformLocation(p, n), 1, glm::value_ptr(v));
}
void uniform(GLuint p, const char *n, float v) {
    glUniform1f(glGetUniformLocation(p, n), v);
}
void integer(GLuint p, const char *n, int v) {
    glUniform1i(glGetUniformLocation(p, n), v);
}
void bindTexture(GLuint id, int unit) {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, id);
}
void checkGl(const char *operation) {
    const auto error = glGetError();
    if (error != GL_NO_ERROR)
        throw std::runtime_error(std::string(operation) + " failed with OpenGL error " +
                                 std::to_string(error));
}
constexpr std::uintmax_t maximum_image_bytes = 256u * 1024 * 1024;
constexpr int maximum_image_side = 16384;
constexpr std::size_t maximum_cutouts = 4; // scene.frag / shadow.frag holes[4]

struct PngInput {
    const std::vector<unsigned char> *bytes = nullptr;
    std::size_t offset = 0;
};
struct PngOutput {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgba; // Bottom row first (OpenGL upload order).
    std::vector<png_bytep> rows;
    const char *error = nullptr;
    char message[160] = {}; // libpng's reason, copied before its longjmp.
};
[[noreturn]] void pngError(png_structp png, png_const_charp text) {
    auto *out = static_cast<PngOutput *>(png_get_error_ptr(png));
    std::snprintf(out->message, sizeof out->message, "%s", text ? text : "invalid PNG data");
    out->error = out->message;
    png_longjmp(png, 1);
}
void pngWarning(png_structp, png_const_charp) {} // Ancillary-chunk warnings are not errors.
void readPngBytes(png_structp png, png_bytep data, png_size_t length) {
    auto *input = static_cast<PngInput *>(png_get_io_ptr(png));
    if (length > input->bytes->size() - input->offset)
        png_error(png, "truncated PNG");
    std::copy_n(input->bytes->data() + input->offset, length, data);
    input->offset += length;
}
// No non-trivial locals: libpng errors longjmp back to this frame. Output storage is
// owned by the caller, so every allocation is released by ordinary C++ destruction.
bool decodePng(png_structp png, png_infop info, int maximum_side, PngOutput *out) {
    if (setjmp(png_jmpbuf(png)))
        return false;
    png_read_info(png, info);
    const auto width = png_get_image_width(png, info), height = png_get_image_height(png, info);
    const int depth = png_get_bit_depth(png, info), type = png_get_color_type(png, info);
    if (width == 0 || height == 0 || width > static_cast<png_uint_32>(maximum_side) ||
        height > static_cast<png_uint_32>(maximum_side) ||
        std::uintmax_t{width} * height * 4 > maximum_image_bytes) {
        out->error = "PNG dimensions exceed the texture limits";
        return false;
    }
    if (depth == 16) {
        out->error = "16-bit PNG textures are not supported";
        return false;
    }
    if (type == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);
    if (type == PNG_COLOR_TYPE_GRAY && depth < 8)
        png_set_expand_gray_1_2_4_to_8(png);
    if (png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png);
    if (type == PNG_COLOR_TYPE_GRAY || type == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);
    if (!(type & PNG_COLOR_MASK_ALPHA) && !png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
    png_set_interlace_handling(png);
    png_read_update_info(png, info);
    if (png_get_rowbytes(png, info) != std::size_t{width} * 4) {
        out->error = "unsupported PNG pixel layout";
        return false;
    }
    out->width = static_cast<int>(width);
    out->height = static_cast<int>(height);
    out->rgba.resize(std::size_t{width} * height * 4);
    out->rows.resize(height);
    for (png_uint_32 y = 0; y < height; ++y) // Flip: the original viewer's cv::flip(.., 0).
        out->rows[y] = out->rgba.data() + std::size_t{height - 1 - y} * width * 4;
    png_read_image(png, out->rows.data());
    png_read_end(png, nullptr);
    return true;
}
PngOutput loadPng(const std::filesystem::path &path, int maximum_side) {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if (error || size > maximum_image_bytes)
        throw std::invalid_argument("cannot read bounded texture: " + path.string());
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size));
    std::ifstream input(path, std::ios::binary);
    if (!input.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(size)))
        throw std::invalid_argument("cannot read texture: " + path.string());
    if (bytes.size() < 8 || png_sig_cmp(bytes.data(), 0, 8))
        throw std::invalid_argument("texture is not a PNG image: " + path.string());
    PngOutput output;
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, &output, pngError, pngWarning);
    png_infop info = png ? png_create_info_struct(png) : nullptr;
    if (!info) {
        png_destroy_read_struct(&png, nullptr, nullptr);
        throw std::runtime_error("PNG decoder allocation failed");
    }
    PngInput source{&bytes, 0};
    png_set_read_fn(png, &source, readPngBytes);
    bool ok = false;
    try {
        ok = decodePng(png, info, maximum_side, &output);
    } catch (...) {
        // Vector allocations may throw without passing through libpng's error handler.
        png_destroy_read_struct(&png, &info, nullptr);
        throw;
    }
    png_destroy_read_struct(&png, &info, nullptr);
    if (!ok)
        throw std::invalid_argument(std::string(output.error ? output.error : "invalid PNG data") +
                                    ": " + path.string());
    output.rows.clear();
    return output;
}
struct Texture {
    GLuint id = 0;
    Texture() = default;
    Texture(const Texture &) = delete;
    Texture &operator=(const Texture &) = delete;
    ~Texture() {
        glDeleteTextures(1, &id);
    }
};
std::shared_ptr<Texture> uploadTexture(const std::filesystem::path &path, int maximum_side) {
    const auto image = loadPng(path, maximum_side);
    auto texture = std::make_shared<Texture>();
    glGenTextures(1, &texture->id);
    glBindTexture(GL_TEXTURE_2D, texture->id);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8_ALPHA8, image.width, image.height, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, image.rgba.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    checkGl("texture upload");
    return texture;
}
using TextureLoader = std::function<std::shared_ptr<Texture>(const std::filesystem::path &)>;
struct Buffers {
    GLuint vao = 0, vbo = 0, ebo = 0;
    ~Buffers() {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }
};
struct Mesh {
    Buffers gpu;
    GLsizei count = 0;
    Bounds bounds;
    glm::vec4 color{1};
    GLuint texture = 0;
    std::shared_ptr<Texture> image; // Shared by submeshes using the same file.
    std::vector<glm::vec3> holes;   // (u, v, radius) per cutout.
    Mesh(const Submesh &input, const TextureLoader &load) {
        if (input.vertices.empty() || input.indices.empty() || input.indices.size() % 3 ||
            input.indices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) ||
            input.vertices.size() >
                static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(Vertex))
            throw std::invalid_argument("invalid GPU triangle mesh size");
        if (!input.material.base_color.allFinite() ||
            (input.material.base_color.array() < 0).any() || input.material.base_color.w() > 1)
            throw std::invalid_argument("invalid mesh color/opacity");
        if (input.material.cutouts.size() > maximum_cutouts)
            throw std::invalid_argument("at most four UV cutouts per submesh");
        for (const auto &cutout : input.material.cutouts) {
            if (!cutout.center.allFinite() || !std::isfinite(cutout.radius) || cutout.radius <= 0)
                throw std::invalid_argument("invalid UV cutout");
            holes.emplace_back(cutout.center.x(), cutout.center.y(), cutout.radius);
        }
        color = glm::make_vec4(input.material.base_color.data());
        for (const auto &v : input.vertices)
            if (!v.position.allFinite() || !v.normal.allFinite() || !v.uv.allFinite() ||
                v.normal.squaredNorm() == 0)
                throw std::invalid_argument("invalid GPU vertex");
        for (auto index : input.indices) {
            if (index >= input.vertices.size())
                throw std::invalid_argument("invalid GPU vertex index");
            bounds.include(vector(input.vertices[index].position));
        }
        if (input.material.diffuse_texture) {
            image = load(*input.material.diffuse_texture);
            texture = image->id;
            color = glm::vec4(1); // Original viewer: a diffuse texture replaces the base color.
        }
        count = static_cast<GLsizei>(input.indices.size());
        glGenVertexArrays(1, &gpu.vao);
        glGenBuffers(1, &gpu.vbo);
        glGenBuffers(1, &gpu.ebo);
        glBindVertexArray(gpu.vao);
        glBindBuffer(GL_ARRAY_BUFFER, gpu.vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(input.vertices.size() * sizeof(Vertex)),
                     input.vertices.data(), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpu.ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(input.indices.size() * sizeof(std::uint32_t)),
                     input.indices.data(), GL_STATIC_DRAW);
        for (GLuint i = 0; i < 3; ++i)
            glEnableVertexAttribArray(i);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, position)));
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, normal)));
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<void *>(offsetof(Vertex, uv)));
        glBindVertexArray(0);
        checkGl("mesh upload");
    }
    void draw() const {
        glBindVertexArray(gpu.vao);
        glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
    }
};
struct Object {
    std::vector<std::shared_ptr<Mesh>> meshes;
    glm::mat4 transform{1};
    glm::vec4 tint{1};
    int material = 0;
    float radiance = 60;
    bool castsShadow = true, visible = true;
};
struct Target {
    GLuint fbo = 0, color = 0, depth = 0;
    int width = 0, height = 0;
    bool hdr = true, depthOnly = false;
    Target() = default;
    Target(const Target &) = delete;
    Target &operator=(const Target &) = delete;
    ~Target() {
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &color);
        glDeleteTextures(1, &depth);
    }
    void resize(int w, int h, bool high = true, bool onlyDepth = false) {
        if (width == w && height == h && hdr == high && depthOnly == onlyDepth)
            return;
        Target next;
        next.width = w;
        next.height = h;
        next.hdr = high;
        next.depthOnly = onlyDepth;
        glGenFramebuffers(1, &next.fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, next.fbo);
        if (onlyDepth) {
            glDrawBuffer(GL_NONE);
            glReadBuffer(GL_NONE);
        } else {
            glGenTextures(1, &next.color);
            glBindTexture(GL_TEXTURE_2D, next.color);
            glTexImage2D(GL_TEXTURE_2D, 0, high ? GL_RGBA16F : GL_RGBA8, w, h, 0, GL_RGBA,
                         high ? GL_FLOAT : GL_UNSIGNED_BYTE, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, next.color,
                                   0);
        }
        glGenTextures(1, &next.depth);
        glBindTexture(GL_TEXTURE_2D, next.depth);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
                     nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, next.depth, 0);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            throw std::runtime_error("render framebuffer incomplete");
        std::swap(fbo, next.fbo);
        std::swap(color, next.color);
        std::swap(depth, next.depth);
        std::swap(width, next.width);
        std::swap(height, next.height);
        std::swap(hdr, next.hdr);
        std::swap(depthOnly, next.depthOnly);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
};
struct Frame {
    Target opaque, composite, final;
    std::array<Target, 2> bloom;
    void resize(int w, int h) {
        opaque.resize(w, h);
        composite.resize(w, h);
        final.resize(w, h, false);
        for (auto &b : bloom)
            b.resize(std::max(1, w / 4), std::max(1, h / 4));
    }
};
bool affine(const Eigen::Matrix4f &m) {
    return m.allFinite() && m.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)) &&
           m.topLeftCorner<3, 3>().fullPivLu().isInvertible();
}
} // namespace
struct Renderer::Resources {
    GLuint sceneProgram = 0, waterProgram = 0, shadowProgram = 0, postProgram = 0, bloomProgram = 0,
           quad = 0;
    Frame f;
    Target shadow, reflection;
    glm::mat4 lightMatrix{1}, poolToMap{1}, mapToPool{1};
    glm::vec3 poolSize{1}, center{0};
    float waterLevel = 0, ledRadiance = 60;
    bool hasWater = false, frame_valid = false;
    GLint maximum_texture = 0, clip_distances = 0;
    Object water;
    std::vector<Object> objects;
    struct Cached {
        std::shared_ptr<const MeshAsset> source;
        std::vector<std::shared_ptr<Mesh>> meshes;
        bool used = false;
    };
    std::map<const MeshAsset *, Cached> cache;
    // Deduplicates uploads while any cached mesh still references the image.
    std::map<std::filesystem::path, std::weak_ptr<Texture>> textures;
    std::shared_ptr<Texture> texture(const std::filesystem::path &path) {
        std::error_code error;
        auto key = std::filesystem::weakly_canonical(path, error);
        if (error)
            key = path.lexically_normal();
        if (auto found = textures.find(key); found != textures.end())
            if (auto existing = found->second.lock())
                return existing;
        for (auto it = textures.begin(); it != textures.end();)
            it = it->second.expired() ? textures.erase(it) : std::next(it);
        auto uploaded = uploadTexture(key, std::min(maximum_texture, maximum_image_side));
        textures[key] = uploaded;
        return uploaded;
    }
    ~Resources() {
        for (auto id : {sceneProgram, waterProgram, shadowProgram, postProgram, bloomProgram})
            glDeleteProgram(id);
        glDeleteVertexArrays(1, &quad);
    }
    void initialize(const std::filesystem::path &root) {
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximum_texture);
        glGetIntegerv(GL_MAX_CLIP_DISTANCES, &clip_distances);
        if (maximum_texture < 4096)
            throw std::runtime_error("renderer requires 4096 pixel shadow textures");
        sceneProgram = program(root, "scene");
        waterProgram = program(root, "water");
        shadowProgram = program(root, "shadow");
        postProgram = program(root, "post");
        bloomProgram = program(root, "bloom");
        glGenVertexArrays(1, &quad);
        shadow.resize(4096, 4096, false, true);
        bindTexture(shadow.depth, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        const float border[] = {1, 1, 1, 1};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
        reflection.resize(640, 400);
        checkGl("renderer initialization");
    }
    Object instance(const Instance &input) {
        if (!input.mesh || input.mesh->submeshes.empty() || !affine(input.transform) ||
            !input.tint.allFinite() || (input.tint.array() < 0).any() || input.tint.w() > 1 ||
            !std::isfinite(input.radiance) || input.radiance < 0 ||
            static_cast<int>(input.material) < 0 || static_cast<int>(input.material) > 6)
            throw std::invalid_argument("invalid render instance");
        auto found = cache.find(input.mesh.get());
        if (found == cache.end()) {
            Cached next;
            next.source = input.mesh;
            const TextureLoader load = [this](const std::filesystem::path &path) {
                return texture(path);
            };
            for (const auto &part : input.mesh->submeshes)
                next.meshes.push_back(std::make_shared<Mesh>(part, load));
            found = cache.emplace(input.mesh.get(), std::move(next)).first;
        }
        found->second.used = true;
        return {found->second.meshes,
                matrix(input.transform),
                glm::make_vec4(input.tint.data()),
                static_cast<int>(input.material),
                input.radiance,
                input.casts_shadow,
                input.visible};
    }
    void shadows(const Look &look);
    void drawScene(const InternalView &camera, const Look &look, float time, bool clip = false);
    void render(const InternalView &camera, const Look &look, float time);
};
void Renderer::Resources::shadows(const Look &look) {
    auto sun = look.outdoor ? look.sunDirection() : glm::normalize(glm::vec3(-.2f, -.1f, 1));
    lightMatrix = glm::ortho(-33.f, 33.f, -33.f, 33.f, .1f, 140.f) *
                  glm::lookAt(center + sun * 60.f, center, glm::vec3(0, 1, 0));
    glBindFramebuffer(GL_FRAMEBUFFER, shadow.fbo);
    glViewport(0, 0, shadow.width, shadow.height);
    glDepthMask(GL_TRUE);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (!look.shadows)
        return;
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(2.f, 4.f);
    glUseProgram(shadowProgram);
    uniform(shadowProgram, "lightMatrix", lightMatrix);
    for (const auto &o : objects) {
        if (!o.castsShadow || !o.visible)
            continue;
        uniform(shadowProgram, "model", o.transform);
        // Off-screen objects can still cast visible shadows. Cull against the
        // light's volume, independently of every viewer/sensor camera.
        const Frustum frustum(lightMatrix * o.transform);
        for (const auto &m : o.meshes) {
            // Clear CAD panels must not cast opaque shadows onto internal LEDs.
            if (m->color.a < .999f)
                continue;
            if (!frustum.intersects(m->bounds))
                continue;
            bindTexture(m->texture, 0);
            integer(shadowProgram, "hasTexture", m->texture != 0);
            integer(shadowProgram, "holeCount", int(m->holes.size()));
            if (!m->holes.empty())
                glUniform3fv(glGetUniformLocation(shadowProgram, "holes"),
                             static_cast<GLsizei>(m->holes.size()), glm::value_ptr(m->holes[0]));
            m->draw();
        }
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
}
void Renderer::Resources::drawScene(const InternalView &camera, const Look &look, float time,
                                    bool clip) {
    glUseProgram(sceneProgram);
    integer(sceneProgram, "waterEnabled", hasWater);
    uniform(sceneProgram, "view", camera.view);
    uniform(sceneProgram, "projection", camera.projection);
    uniform(sceneProgram, "eye", camera.eye);
    uniform(sceneProgram, "waterLevel", waterLevel);
    uniform(sceneProgram, "poolSize", glm::vec3(poolSize.x, poolSize.y, poolSize.z));
    uniform(sceneProgram, "lightMatrix", lightMatrix);
    uniform(sceneProgram, "mapToPool", mapToPool);
    uniform(sceneProgram, "time", time);
    uniform(sceneProgram, "ledRadiance", ledRadiance);
    uniform(sceneProgram, "waterTint", look.water.tint);
    uniform(sceneProgram, "waterAbsorption", look.water.absorption);
    uniform(sceneProgram, "waterScattering", look.water.scattering);
    uniform(sceneProgram, "waterDistanceScale", look.water.distanceScale);
    uniform(sceneProgram, "waterDistancePower", look.water.distancePower);
    uniform(sceneProgram, "waterClearDistance", look.water.clearDistance);
    uniform(sceneProgram, "caustics", look.caustics);
    uniform(sceneProgram, "sunDirection",
            look.outdoor ? look.sunDirection() : glm::normalize(glm::vec3(-.2f, -.1f, 1)));
    uniform(sceneProgram, "directLight", look.directLight);
    uniform(sceneProgram, "ambientLight", look.ambientLight);
    integer(sceneProgram, "outdoor", look.outdoor);
    integer(sceneProgram, "useShadow", look.shadows);
    integer(sceneProgram, "clipWater", clip);
    integer(sceneProgram, "albedo", 0);
    integer(sceneProgram, "shadowMap", 1);
    bindTexture(shadow.depth, 1);
    const auto viewProjection = camera.projection * camera.view;
    // Clear polycarbonate is blended after opaque electronics and LED lenses.
    // It writes the front-cover depth, so RGB/depth still describe one enclosure.
    for (int transparent = 0; transparent < 2; ++transparent) {
        if (transparent) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
        for (const auto &o : objects) {
            if (!o.visible)
                continue;
            const Frustum frustum(viewProjection * o.transform);
            bool modelBound = false;
            for (const auto &m : o.meshes) {
                const int material = o.material == 0 && m->color.a < .999f ? 5 : o.material;
                if ((material == 5) != bool(transparent))
                    continue;
                if (!frustum.intersects(m->bounds))
                    continue;
                if (!modelBound) {
                    uniform(sceneProgram, "model", o.transform);
                    uniform(sceneProgram, "ledRadiance", o.radiance < 0 ? ledRadiance : o.radiance);
                    modelBound = true;
                }
                integer(sceneProgram, "material", material);
                const auto tint = m->color * o.tint;
                glUniform4fv(glGetUniformLocation(sceneProgram, "tint"), 1, glm::value_ptr(tint));
                integer(sceneProgram, "hasTexture", m->texture != 0);
                bindTexture(m->texture, 0);
                integer(sceneProgram, "holeCount", int(m->holes.size()));
                if (!m->holes.empty())
                    glUniform3fv(glGetUniformLocation(sceneProgram, "holes"),
                                 static_cast<GLsizei>(m->holes.size()),
                                 glm::value_ptr(m->holes[0]));
                m->draw();
            }
        }
    }
    glDisable(GL_BLEND);
}
void Renderer::Resources::render(const InternalView &camera, const Look &look, float time) {
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    const bool surfaceVisible = hasWater && water.visible && look.surface &&
                                Frustum(camera.projection * camera.view * water.transform)
                                    .intersects(water.meshes.front()->bounds);
    const bool reflectionActive =
        look.surfaceReflections && surfaceVisible && camera.eye.z > waterLevel;
    if (reflectionActive) {
        reflection.resize(std::max(160, f.opaque.width / 2), std::max(100, f.opaque.height / 2));
        glBindFramebuffer(GL_FRAMEBUFFER, reflection.fbo);
        glViewport(0, 0, reflection.width, reflection.height);
        glClearColor(look.outdoor ? .30f : .13f, look.outdoor ? .48f : .16f,
                     look.outdoor ? .68f : .18f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glm::mat4 mirror = glm::translate(glm::mat4(1), {0, 0, 2 * waterLevel}) *
                           glm::scale(glm::mat4(1), glm::vec3(1, 1, -1));
        InternalView reflected = camera;
        reflected.eye.z = 2 * waterLevel - reflected.eye.z;
        reflected.view = camera.view * mirror;
        drawScene(reflected, look, time, true);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, f.opaque.fbo);
    glViewport(0, 0, f.opaque.width, f.opaque.height);
    glClearColor(look.outdoor ? .30f : .13f, look.outdoor ? .48f : .16f, look.outdoor ? .68f : .18f,
                 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawScene(camera, look, time);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, f.opaque.fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, f.composite.fbo);
    glBlitFramebuffer(0, 0, f.opaque.width, f.opaque.height, 0, 0, f.opaque.width, f.opaque.height,
                      GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, f.composite.fbo);
    if (surfaceVisible) {
        glUseProgram(waterProgram);
        uniform(waterProgram, "model", water.transform);
        uniform(waterProgram, "view", camera.view);
        uniform(waterProgram, "projection", camera.projection);
        uniform(waterProgram, "eye", camera.eye);
        uniform(waterProgram, "waterLevel", waterLevel);
        uniform(waterProgram, "poolSize", glm::vec3(poolSize.x, poolSize.y, poolSize.z));
        uniform(waterProgram, "time", time);
        uniform(waterProgram, "sunDirection", look.sunDirection());
        uniform(waterProgram, "directLight", look.directLight);
        uniform(waterProgram, "glare", look.outdoor ? look.glare : 0.f);
        uniform(waterProgram, "ambientLight", look.ambientLight);
        uniform(waterProgram, "waterTint", look.water.tint);
        integer(waterProgram, "outdoor", look.outdoor);
        uniform(waterProgram, "mapToPool", mapToPool);
        integer(waterProgram, "sceneColor", 0);
        integer(waterProgram, "reflectionColor", 1);
        integer(waterProgram, "sceneDepth", 2);
        integer(waterProgram, "hasReflection", reflectionActive);
        integer(waterProgram, "surfaceReflections", look.surfaceReflections);
        bindTexture(f.opaque.color, 0);
        bindTexture(reflection.color, 1);
        bindTexture(f.opaque.depth, 2);
        glUniform2f(glGetUniformLocation(waterProgram, "resolution"),
                    static_cast<float>(f.opaque.width), static_cast<float>(f.opaque.height));
        glDepthMask(GL_FALSE);
        water.meshes.front()->draw();
        glDepthMask(GL_TRUE);
    }
    // Filter the HDR bright pass before tone mapping. A continuous low-resolution
    // blur avoids the replicated bars produced by sparse full-resolution rings.
    glDisable(GL_DEPTH_TEST);
    glUseProgram(bloomProgram);
    integer(bloomProgram, "source", 0);
    glBindVertexArray(quad);
    glViewport(0, 0, f.bloom[0].width, f.bloom[0].height);
    for (int pass = 0; pass < 3; ++pass) {
        glBindFramebuffer(GL_FRAMEBUFFER, f.bloom[pass % 2].fbo);
        integer(bloomProgram, "extractBright", pass == 0);
        bindTexture(pass == 0 ? f.composite.color : f.bloom[(pass - 1) % 2].color, 0);
        glUniform2f(glGetUniformLocation(bloomProgram, "stepSize"),
                    pass == 0 ? 1.f / static_cast<float>(f.composite.width)
                              : (pass == 1 ? 1.f / static_cast<float>(f.bloom[0].width) : 0.f),
                    pass == 0 ? 1.f / static_cast<float>(f.composite.height)
                              : (pass == 2 ? 1.f / static_cast<float>(f.bloom[0].height) : 0.f));
        glDrawArrays(GL_TRIANGLES, 0, 3);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, f.final.fbo);
    glViewport(0, 0, f.final.width, f.final.height);
    glUseProgram(postProgram);
    integer(postProgram, "sceneColor", 0);
    uniform(postProgram, "exposure", look.exposure);
    bindTexture(f.composite.color, 0);
    bindTexture(f.composite.depth, 1);
    bindTexture(f.bloom[0].color, 2);
    integer(postProgram, "bloomColor", 2);
    integer(postProgram, "sceneDepth", 1);
    uniform(postProgram, "inverseViewProjection", glm::inverse(camera.projection * camera.view));
    uniform(postProgram, "eye", camera.eye);
    uniform(postProgram, "sunDirection", look.sunDirection());
    uniform(postProgram, "glare", look.outdoor ? look.glare * look.directLight : 0.f);
    glUniform2f(glGetUniformLocation(postProgram, "texel"),
                1.f / static_cast<float>(f.opaque.width),
                1.f / static_cast<float>(f.opaque.height));
    glBindVertexArray(quad);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
Renderer::Renderer(const std::filesystem::path &root) {
    if (!GLEW_VERSION_3_3)
        throw std::runtime_error("rendering requires initialized GLEW and OpenGL 3.3");
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    resources_ = std::make_unique<Resources>();
    resources_->initialize(root);
}
Renderer::~Renderer() = default;
RenderedFrame Renderer::draw(const Scene &scene, const View &view, const Appearance &a, float time,
                             int width, int height) {
    auto &r = *resources_;
    r.frame_valid = false;
    struct Guard {
        Resources &r;
        bool success = false;
        ~Guard() {
            if (!success) {
                r.objects.clear();
                r.water = {};
                r.cache.clear();
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
            }
        }
    } guard{r};
    const auto maximum = r.maximum_texture;
    if (width <= 0 || height <= 0 || width > maximum || height > maximum || !std::isfinite(time) ||
        !view.eye.allFinite() || !affine(view.view) || !view.projection.allFinite() ||
        !view.projection.fullPivLu().isInvertible() || !scene.lighting_center.allFinite())
        throw std::invalid_argument("invalid render view, size or time");
    for (float value : {a.caustics, a.exposure, a.direct_light, a.ambient_light, a.glare,
                        a.water.scattering, a.water.distance_scale, a.water.clear_distance})
        if (!std::isfinite(value) || value < 0)
            throw std::invalid_argument("invalid render appearance");
    if (!std::isfinite(a.sun_azimuth) || !std::isfinite(a.sun_elevation) ||
        !std::isfinite(a.water.distance_power) || a.water.distance_power < .25f ||
        a.water.distance_power > 3 || !a.water.tint.allFinite() ||
        (a.water.tint.array() < 0).any() || (a.water.tint.array() > 1).any() ||
        !a.water.absorption.allFinite() || (a.water.absorption.array() < 0).any())
        throw std::invalid_argument("invalid sun or water appearance");
    glDisable(GL_COLOR_LOGIC_OP);
    for (GLint i = 0; i < r.clip_distances; ++i)
        glDisable(GL_CLIP_DISTANCE0 + i);
    glEnable(GL_DITHER);
    glDisable(GL_DEPTH_CLAMP);
    glFrontFace(GL_CCW);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glDisable(GL_RASTERIZER_DISCARD);
    glDisable(GL_PRIMITIVE_RESTART);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthFunc(GL_LESS);
    glDepthRange(0, 1);
    glClearDepth(1);
    glBlendEquation(GL_FUNC_ADD);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    for (GLuint unit = 0; unit < 3; ++unit)
        glBindSampler(unit, 0);
    for (auto &item : r.cache)
        item.second.used = false;
    r.objects.clear();
    r.water = {};
    r.hasWater = scene.water.has_value();
    for (const auto &item : scene.instances)
        r.objects.push_back(r.instance(item));
    r.center = vector(scene.lighting_center);
    r.poolToMap = glm::mat4(1);
    r.mapToPool = glm::mat4(1);
    r.poolSize = glm::vec3(1);
    r.waterLevel = 0;
    if (scene.water) {
        const auto &water = *scene.water;
        // Reuse pool frame validation; do not allocate pool geometry to validate.
        const auto &m = water.local_to_world;
        if (!affine(m) || !water.dimensions.allFinite() || (water.dimensions.array() <= 0).any() ||
            !std::isfinite(water.level) || !m.col(2).isApprox(Eigen::Vector4f(0, 0, 1, 0)) ||
            !m.row(2).isApprox(Eigen::RowVector4f(0, 0, 1, 0)) ||
            !(m.topLeftCorner<3, 3>().transpose() * m.topLeftCorner<3, 3>())
                 .isApprox(Eigen::Matrix3f::Identity(), 1e-5f) ||
            std::abs(m.topLeftCorner<3, 3>().determinant() - 1) > 1e-5f)
            throw std::invalid_argument(
                "water requires positive dimensions and a horizontal rigid frame");
        if (!water.surface.mesh || !affine(water.surface.transform))
            throw std::invalid_argument("invalid water surface instance");
        for (const auto &part : water.surface.mesh->submeshes)
            for (const auto &vertex : part.vertices) {
                const Eigen::Vector4f position =
                    water.surface.transform * Eigen::Vector4f(vertex.position.x(),
                                                              vertex.position.y(),
                                                              vertex.position.z(), 1);
                if (!position.allFinite() ||
                    std::abs(position.z() - water.level) > 1e-5f * (1 + std::abs(water.level)))
                    throw std::invalid_argument(
                        "water surface vertices must lie on the declared horizontal plane");
            }
        r.water = r.instance(water.surface);
        if (r.water.meshes.size() != 1)
            throw std::invalid_argument("water surface requires one submesh");
        r.poolSize = vector(water.dimensions);
        r.waterLevel = water.level;
        r.poolToMap = matrix(m);
        r.mapToPool = glm::inverse(r.poolToMap);
    }
    for (auto it = r.cache.begin(); it != r.cache.end();) {
        if (!it->second.used)
            it = r.cache.erase(it);
        else
            ++it;
    }
    r.f.resize(width, height);
    const Look look{{vector(a.water.tint), vector(a.water.absorption), a.water.scattering,
                     a.water.distance_scale, a.water.distance_power, a.water.clear_distance},
                    a.caustics,
                    a.exposure,
                    a.surface,
                    a.shadows,
                    a.reflections,
                    a.outdoor,
                    a.sun_azimuth,
                    a.sun_elevation,
                    a.direct_light,
                    a.ambient_light,
                    a.glare};
    r.shadows(look);
    r.render({matrix(view.view), matrix(view.projection), vector(view.eye)}, look, time);
    checkGl("scene rendering");
    r.frame_valid = true;
    guard.success = true;
    return {r.f.final.color, width, height, r.f.composite.depth};
}
Capture Renderer::capture() const {
    const auto &f = resources_->f;
    if (!resources_->frame_valid)
        throw std::logic_error("capture requires a rendered frame");
    Capture result;
    result.width = f.final.width;
    result.height = f.final.height;
    const auto pixels = static_cast<std::size_t>(result.width) * result.height;
    result.rgba.resize(pixels * 4);
    result.opaque_rgba.resize(pixels * 4);
    result.composite_rgba.resize(pixels * 4);
    result.opaque_depth.resize(pixels);
    result.composite_depth.resize(pixels);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_SWAP_BYTES, GL_FALSE);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    const auto read = [&](const Target &target, std::vector<float> &color,
                          std::vector<float> &depth) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, target.fbo);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, result.width, result.height, GL_RGBA, GL_FLOAT, color.data());
        glReadPixels(0, 0, result.width, result.height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());
    };
    read(f.opaque, result.opaque_rgba, result.opaque_depth);
    read(f.composite, result.composite_rgba, result.composite_depth);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, f.final.fbo);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, result.width, result.height, GL_RGBA, GL_UNSIGNED_BYTE, result.rgba.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    checkGl("frame capture");
    return result;
}
ImageCapture Renderer::captureImage(bool color, bool depth) const {
    const auto &f = resources_->f;
    if (!resources_->frame_valid)
        throw std::logic_error("capture requires a rendered frame");
    ImageCapture result;
    result.width = f.final.width;
    result.height = f.final.height;
    const auto pixels = static_cast<std::size_t>(result.width) * result.height;
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_SWAP_BYTES, GL_FALSE);
    glPixelStorei(GL_PACK_ALIGNMENT, 1); // RGB8 rows are not 4-byte aligned in general.
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    if (color) { // Same source as the original sensor: tone-mapped final color.
        result.rgb.resize(pixels * 3);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, f.final.fbo);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, result.width, result.height, GL_RGB, GL_UNSIGNED_BYTE,
                     result.rgb.data());
    }
    if (depth) { // Opaque pass depth: the water surface never occludes sensor depth.
        result.depth.resize(pixels);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, f.opaque.fbo);
        glReadPixels(0, 0, result.width, result.height, GL_DEPTH_COMPONENT, GL_FLOAT,
                     result.depth.data());
    }
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    checkGl("image capture");
    return result;
}
} // namespace robotics::rendering
