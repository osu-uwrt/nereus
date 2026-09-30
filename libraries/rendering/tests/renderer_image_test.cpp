// GPU contracts for texture upload, UV cutouts and sensor-sized readback. Requires a
// display for a hidden GLFW OpenGL 3.3 context; tests skip when none can be created.
#include <Eigen/Geometry>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <gtest/gtest.h>

#include <png.h>
#include <robotics/rendering/renderer.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace r = robotics::rendering;
namespace fs = std::filesystem;

namespace {
struct Context {
    GLFWwindow *window = nullptr;
    Context() {
        if (!glfwInit())
            return;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(64, 64, "renderer image tests", nullptr, nullptr);
        if (!window)
            return;
        glfwMakeContextCurrent(window);
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK) {
            glfwDestroyWindow(window);
            window = nullptr;
            return;
        }
        while (glGetError() != GL_NO_ERROR) {
        }
    }
    ~Context() {
        if (window)
            glfwDestroyWindow(window);
        glfwTerminate();
    }
};

// Writes an 8-bit PNG; rows are given top row first, as stored in the file.
void writePng(const fs::path &path, int width, int height, int channels, const std::vector<std::uint8_t> &pixels,
              int bit_depth = 8) {
    FILE *file = std::fopen(path.c_str(), "wb");
    if (!file)
        throw std::runtime_error("cannot write test PNG");
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png_create_info_struct(png);
    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        std::fclose(file);
        throw std::runtime_error("test PNG encoding failed");
    }
    png_init_io(png, file);
    png_set_IHDR(png, info, width, height, bit_depth, channels == 4 ? PNG_COLOR_TYPE_RGBA : PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    const std::size_t stride = std::size_t(width) * channels * (bit_depth / 8);
    for (int y = 0; y < height; ++y)
        png_write_row(png, const_cast<png_bytep>(pixels.data() + y * stride));
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    std::fclose(file);
}

// Unit quad in the YZ plane at x = 0 facing +X, with uv (0,0) at (y=-.5, z=-.5).
std::shared_ptr<r::MeshAsset> quad(std::optional<fs::path> texture = {}, std::vector<r::UvCutout> cutouts = {}) {
    auto mesh = std::make_shared<r::MeshAsset>();
    r::Submesh part;
    const float c[4][2] = {{-.5f, -.5f}, {.5f, -.5f}, {.5f, .5f}, {-.5f, .5f}};
    for (const auto &corner : c)
        part.vertices.push_back({{0, corner[0], corner[1]}, {1, 0, 0}, {corner[0] + .5f, corner[1] + .5f}});
    part.indices = {0, 1, 2, 0, 2, 3};
    part.material.diffuse_texture = texture;
    part.material.cutouts = std::move(cutouts);
    mesh->submeshes.push_back(part);
    mesh->minimum = {0, -.5f, -.5f};
    mesh->maximum = {0, .5f, .5f};
    return mesh;
}

// Camera on +X looking back at the quad; image up is world +Z, image right is world -Y.
r::View view(float distance = 1.2f, float aspect = 1.f) {
    r::View v;
    const Eigen::Vector3f eye(distance, 0, 0);
    const Eigen::Vector3f forward = -eye.normalized(), up(0, 0, 1);
    const Eigen::Vector3f right = forward.cross(up).normalized();
    const Eigen::Vector3f true_up = right.cross(forward);
    v.view.setIdentity();
    v.view.block<1, 3>(0, 0) = right.transpose();
    v.view.block<1, 3>(1, 0) = true_up.transpose();
    v.view.block<1, 3>(2, 0) = -forward.transpose();
    v.view(0, 3) = -right.dot(eye);
    v.view(1, 3) = -true_up.dot(eye);
    v.view(2, 3) = forward.dot(eye);
    const float n = .05f, f = 100.f, t = std::tan(.5f * 1.2f);
    v.projection.setZero();
    v.projection(0, 0) = 1 / (aspect * t);
    v.projection(1, 1) = 1 / t;
    v.projection(2, 2) = -(f + n) / (f - n);
    v.projection(2, 3) = -2 * f * n / (f - n);
    v.projection(3, 2) = -1;
    v.eye = eye;
    return v;
}

r::Scene scene(std::shared_ptr<const r::MeshAsset> mesh) {
    r::Scene s;
    r::Instance instance;
    instance.mesh = std::move(mesh);
    s.instances.push_back(instance);
    return s;
}

r::Appearance plain() {
    r::Appearance a;
    a.caustics = 0;
    a.shadows = false;
    a.reflections = false;
    return a;
}

std::size_t index(const r::ImageCapture &image, int column, int row_from_bottom) {
    return std::size_t(row_from_bottom) * image.width + column;
}
} // namespace

class RendererImage : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        context = std::make_unique<Context>();
        if (context->window)
            renderer = std::make_unique<r::Renderer>(NEREUS_RENDERING_SHADERS);
    }
    static void TearDownTestSuite() {
        renderer.reset();
        context.reset();
    }
    void SetUp() override {
        if (!renderer)
            GTEST_SKIP() << "no OpenGL 3.3 context (DISPLAY unavailable)";
        directory = fs::temp_directory_path() / ("nereus_renderer_image_" + std::to_string(::getpid()));
        fs::create_directories(directory);
    }
    void TearDown() override {
        if (!directory.empty())
            fs::remove_all(directory);
    }
    static inline std::unique_ptr<Context> context;
    static inline std::unique_ptr<r::Renderer> renderer;
    fs::path directory;
};

TEST_F(RendererImage, CaptureImageRequiresAFrameAndOmitsUnrequestedOutputs) {
    r::Renderer fresh(NEREUS_RENDERING_SHADERS);
    EXPECT_THROW(fresh.captureImage(), std::logic_error);
    fresh.draw(scene(quad()), view(), plain(), 0, 33, 17);
    const auto rgb_only = fresh.captureImage(true, false);
    EXPECT_EQ(rgb_only.width, 33);
    EXPECT_EQ(rgb_only.height, 17);
    EXPECT_EQ(rgb_only.rgb.size(), 33u * 17 * 3);
    EXPECT_TRUE(rgb_only.depth.empty());
    const auto depth_only = fresh.captureImage(false, true);
    EXPECT_TRUE(depth_only.rgb.empty());
    EXPECT_EQ(depth_only.depth.size(), 33u * 17);
}

TEST_F(RendererImage, CaptureImageMatchesFullCaptureFinalRgbAndOpaqueDepth) {
    renderer->draw(scene(quad()), view(1.2f, 33.f / 17), plain(), 3, 33, 17);
    glPixelStorei(GL_PACK_ALIGNMENT, 8); // Hostile caller state must not skew RGB8 rows.
    const auto image = renderer->captureImage();
    const auto full = renderer->capture();
    ASSERT_EQ(image.rgb.size() / 3, full.rgba.size() / 4);
    for (std::size_t i = 0; i < image.rgb.size() / 3; ++i)
        for (int c = 0; c < 3; ++c)
            ASSERT_EQ(image.rgb[3 * i + c], full.rgba[4 * i + c]) << "pixel " << i;
    EXPECT_EQ(image.depth, full.opaque_depth);
    GLint alignment = 0;
    glGetIntegerv(GL_PACK_ALIGNMENT, &alignment);
    EXPECT_EQ(alignment, 1);
}

TEST_F(RendererImage, TextureRowsAreFlippedLikeTheOriginalAndReadBackBottomUp) {
    // File rows: top half red, bottom half blue. After the original flip-on-upload, uv v=1
    // samples the top of the image file, which the quad maps to world +Z (image top).
    const int size = 8;
    std::vector<std::uint8_t> pixels(size * size * 3);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            auto *p = &pixels[(y * size + x) * 3];
            p[0] = y < size / 2 ? 255 : 0;
            p[2] = y < size / 2 ? 0 : 255;
        }
    const auto file = directory / "halves.png";
    writePng(file, size, size, 3, pixels);
    renderer->draw(scene(quad(file)), view(), plain(), 0, 64, 64);
    const auto image = renderer->captureImage(true, false);
    const auto top = &image.rgb[3 * index(image, 32, 44)];
    const auto bottom = &image.rgb[3 * index(image, 32, 20)];
    EXPECT_GT(top[0], top[2]) << "top of file must appear at the top of the image";
    EXPECT_GT(bottom[2], bottom[0]);
}

TEST_F(RendererImage, UvCutoutsRemoveColorAndDepthUsingTheUnchangedShaders) {
    renderer->draw(scene(quad({}, {{{.5f, .5f}, .2f}})), view(), plain(), 0, 64, 64);
    const auto cut = renderer->captureImage(false, true);
    EXPECT_EQ(cut.depth[index(cut, 32, 32)], 1.f) << "hole centre must show background depth";
    EXPECT_LT(cut.depth[index(cut, 32, 46)], 1.f) << "solid panel outside the hole";
    renderer->draw(scene(quad()), view(), plain(), 0, 64, 64);
    const auto solid = renderer->captureImage(false, true);
    EXPECT_LT(solid.depth[index(solid, 32, 32)], 1.f);
    // Shadow pass also honours cutouts; it must render without GL errors.
    auto shadowed = plain();
    shadowed.shadows = true;
    EXPECT_NO_THROW(renderer->draw(scene(quad({}, {{{.5f, .5f}, .2f}})), view(), shadowed, 0, 32, 32));
}

TEST_F(RendererImage, TransparentTexelsBelowTheOriginalThresholdAreDiscarded) {
    std::vector<std::uint8_t> pixels(4 * 4 * 4, 255);
    for (std::size_t i = 3; i < pixels.size(); i += 4)
        pixels[i] = 50; // alpha .2 < .4
    const auto file = directory / "clear.png";
    writePng(file, 4, 4, 4, pixels);
    renderer->draw(scene(quad(file)), view(), plain(), 0, 32, 32);
    const auto image = renderer->captureImage(false, true);
    EXPECT_EQ(image.depth[index(image, 16, 16)], 1.f);
}

TEST_F(RendererImage, InvalidCutoutsAndTexturesAreRejectedAndTheRendererRecovers) {
    const auto rejects = [&](std::shared_ptr<r::MeshAsset> mesh) {
        EXPECT_THROW(renderer->draw(scene(mesh), view(), plain(), 0, 16, 16), std::invalid_argument);
        EXPECT_THROW(renderer->captureImage(), std::logic_error) << "failed draw invalidates frame";
    };
    rejects(quad({}, std::vector<r::UvCutout>(5, {{.5f, .5f}, .1f})));
    rejects(quad({}, {{{.5f, .5f}, 0.f}}));
    rejects(quad({}, {{{NAN, .5f}, .1f}}));
    rejects(quad({}, {{{.5f, .5f}, INFINITY}}));
    rejects(quad(directory / "missing.png"));
    const auto text = directory / "not.png";
    {
        std::FILE *f = std::fopen(text.c_str(), "wb");
        std::fputs("not a png", f);
        std::fclose(f);
    }
    rejects(quad(text));
    const auto deep = directory / "deep.png";
    writePng(deep, 2, 2, 3, std::vector<std::uint8_t>(2 * 2 * 3 * 2, 7), 16);
    rejects(quad(deep));
    const auto wide = directory / "wide.png";
    writePng(wide, 16385, 1, 3, std::vector<std::uint8_t>(16385 * 3, 9));
    rejects(quad(wide));
    const auto truncated = directory / "truncated.png";
    writePng(truncated, 8, 8, 3, std::vector<std::uint8_t>(8 * 8 * 3, 1));
    fs::resize_file(truncated, fs::file_size(truncated) / 2);
    rejects(quad(truncated));
    EXPECT_NO_THROW(renderer->draw(scene(quad()), view(), plain(), 0, 16, 16));
    EXPECT_NO_THROW(renderer->captureImage());
}

TEST_F(RendererImage, SharedTexturesAreReusedAndReleasedWithTheirMeshes) {
    std::vector<std::uint8_t> pixels(2 * 2 * 3, 128);
    const auto file = directory / "shared.png";
    writePng(file, 2, 2, 3, pixels);
    auto first = quad(file), second = quad(file);
    r::Scene both = scene(first);
    both.instances.push_back(both.instances.front());
    both.instances.back().mesh = second;
    renderer->draw(both, view(), plain(), 0, 16, 16);
    // After a scene without textures the texture objects must be deleted (not leaked).
    std::vector<GLuint> live;
    for (GLuint id = 1; id < 4096; ++id)
        if (glIsTexture(id))
            live.push_back(id);
    renderer->draw(scene(quad()), view(), plain(), 0, 16, 16);
    int released = 0;
    for (GLuint id : live)
        released += !glIsTexture(id);
    EXPECT_EQ(released, 1) << "exactly one shared upload, released with its last mesh";
}

TEST_F(RendererImage, ImportedTaskAssetWithTextureRenders) {
    const fs::path asset = fs::path(NEREUS_PACK_CONTENT) / "tasks/robosub_2026/assets/torpedo/model.dae";
    if (!fs::exists(asset))
        GTEST_SKIP() << "imported torpedo asset unavailable";
    auto mesh = std::make_shared<r::MeshAsset>(r::loadMesh(asset));
    bool textured = false;
    for (const auto &part : mesh->submeshes)
        textured |= part.material.diffuse_texture.has_value();
    ASSERT_TRUE(textured);
    renderer->draw(scene(mesh), view(2.5f), plain(), 0, 96, 60);
    const auto image = renderer->captureImage();
    EXPECT_EQ(image.rgb.size(), 96u * 60 * 3);
    EXPECT_LT(image.depth[index(image, 48, 30)], 1.f) << "board must occupy the image centre";
}

TEST_F(RendererImage, AbandonContextReleasesCpuOwnersWithoutAContextAndIsTerminal) {
    r::Renderer lost(NEREUS_RENDERING_SHADERS);
    auto mesh = quad();
    std::weak_ptr<const r::MeshAsset> retained = mesh;
    lost.draw(scene(mesh), view(), plain(), 0, 16, 16);
    mesh.reset();
    EXPECT_FALSE(retained.expired());
    glfwMakeContextCurrent(nullptr);
    lost.abandonContext();
    EXPECT_TRUE(retained.expired());
    EXPECT_NO_THROW(lost.abandonContext());
    EXPECT_THROW(lost.capture(), std::logic_error);
    EXPECT_THROW(lost.captureImage(), std::logic_error);
    EXPECT_THROW(lost.draw(scene(quad()), view(), plain(), 0, 16, 16), std::logic_error);
    // This test deliberately abandons GPU names. The fixture destroys their owning
    // context after the suite, just as the offscreen host does after a lost context.
    glfwMakeContextCurrent(context->window);
}
