// GPU contracts for texture upload, UV cutouts and sensor-sized readback. Requires a
// display for a hidden GLFW OpenGL 3.3 context; tests skip when none can be created.
#include <Eigen/Geometry>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <gtest/gtest.h>

#include <nereus/rendering/renderer.hpp>
#include <png.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace r = nereus::rendering;
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

// Camera at `eye` looking straight down; image up is world +X, image right is world -Y.
r::View lookDown(const Eigen::Vector3f &eye) {
    r::View v = view();
    const Eigen::Vector3f forward(0, 0, -1), right(0, -1, 0), up(1, 0, 0);
    v.view.setIdentity();
    v.view.block<1, 3>(0, 0) = right.transpose();
    v.view.block<1, 3>(1, 0) = up.transpose();
    v.view.block<1, 3>(2, 0) = -forward.transpose();
    v.view(0, 3) = -right.dot(eye);
    v.view(1, 3) = -up.dot(eye);
    v.view(2, 3) = forward.dot(eye);
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

// The unit quad turned 20 degrees about the view axis (edges cross the pixel grid obliquely), tinted.
r::Scene tiltedQuad(const Eigen::Vector4f &tint) {
    auto s = scene(quad());
    s.instances[0].transform.topLeftCorner<3, 3>() =
        Eigen::AngleAxisf(20 * float(M_PI) / 180, Eigen::Vector3f::UnitX()).toRotationMatrix();
    s.instances[0].tint = tint;
    return s;
}

r::Appearance supersampled(int n, r::Appearance a = plain()) {
    a.supersample = n;
    return a;
}

// post.frag's tone curve and gamma on one linear HDR channel, as an 8-bit value.
int toneMapped(float c, float exposure = 1) {
    c *= exposure;
    c = std::clamp((c * (2.51f * c + .03f)) / (c * (2.43f * c + .59f) + .14f), 0.f, 1.f);
    return int(std::lround(std::pow(c, 1 / 2.2f) * 255));
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

TEST_F(RendererImage, PoolStripesArePaintedFromDataNotTheShader) {
    // 4 m square pool, 2 m deep; the camera is 1 m under water looking down at the floor centre.
    r::PoolGeometry pool;
    pool.dimensions = {4, 4, 2};
    const auto floorAt = [&](int column, int row) {
        renderer->draw(r::makePoolScene(pool), lookDown({2, 2, -1}), plain(), 0, 64, 64);
        const auto image = renderer->captureImage(true, false);
        const auto i = 3 * index(image, column, row);
        return image.rgb[i] + image.rgb[i + 1] + image.rgb[i + 2];
    };
    const int bare = floorAt(32, 32);
    EXPECT_NEAR(bare, floorAt(10, 32), 30) << "no markings: the floor is plain tile everywhere";
    // A 0.3 m stripe along x through the centre: an image column (image right is world -Y).
    pool.markings.push_back({r::PoolSide::Floor, {1, 2}, {3, 2}, .3f, {0, 0, 0}});
    EXPECT_LT(floorAt(32, 32), bare / 2) << "stripe centre is dark";
    EXPECT_NEAR(floorAt(10, 32), bare, 30) << "0.47 m off the stripe the floor is unchanged";
    // A white card lying 1 mm above the floor over the stripe (like the calibration board on a wall line)
    // covers the stripe even though the decal sits 2 mm off the floor.
    auto card = std::make_shared<r::MeshAsset>();
    r::Submesh face;
    for (const auto &c : {Eigen::Vector2f(-.2f, -.2f), Eigen::Vector2f(.2f, -.2f), Eigen::Vector2f(.2f, .2f),
                          Eigen::Vector2f(-.2f, .2f)})
        face.vertices.push_back({{2 + c.x(), 2 + c.y(), -2 + .001f}, {0, 0, 1}, {0, 0}});
    face.indices = {0, 1, 2, 0, 2, 3};
    card->submeshes.push_back(face);
    card->minimum = {1.8f, 1.8f, -2};
    card->maximum = {2.2f, 2.2f, -2};
    auto covered = r::makePoolScene(pool);
    r::Instance instance;
    instance.mesh = card;
    covered.instances.push_back(instance);
    renderer->draw(covered, lookDown({2, 2, -1}), plain(), 0, 64, 64);
    const auto image = renderer->captureImage(true, false);
    const auto i = 3 * index(image, 32, 32);
    EXPECT_GT(image.rgb[i] + image.rgb[i + 1] + image.rgb[i + 2], bare) << "the card, not the stripe";
    pool.markings.push_back({r::PoolSide::Floor, {2, 2}, {2, 2}, .3f, {0, 0, 0}});
    EXPECT_THROW(r::makePoolScene(pool), std::invalid_argument) << "zero-length stripe";
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

TEST_F(RendererImage, LabelPassBetweenDrawsLeavesFramesAndCapturesUnchanged) {
    std::vector<std::uint8_t> pixels = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
    const auto file = directory / "quadrants.png";
    writePng(file, 2, 2, 3, pixels);
    r::Scene both = scene(quad(file));
    r::Instance box;
    box.mesh = r::makeBoxMesh();
    box.transform(0, 3) = -1.5f;
    both.instances.push_back(box);
    auto shadowed = plain();
    shadowed.shadows = true;
    r::Renderer fresh(NEREUS_RENDERING_SHADERS);
    // A label pass needs no frame and does not make one.
    const auto first = fresh.drawLabels(both, {{1, {}}, {2, {}}}, view(), 48, 40);
    EXPECT_EQ(first.ids[20 * 48 + 24], 1u << 8);
    EXPECT_THROW(fresh.captureImage(), std::logic_error);
    fresh.draw(both, view(1.2f, 48.f / 40), shadowed, 2, 48, 40);
    const auto image = fresh.captureImage();
    const auto full = fresh.capture();
    // Different size and scene subset: the label target is separate from the colour frame.
    const auto labels = fresh.drawLabels(scene(quad()), {{7, {{3, std::nullopt}}}}, view(), 17, 9);
    EXPECT_EQ(labels.ids.size(), 17u * 9);
    EXPECT_EQ(labels.ids[4 * 17 + 8], 7u << 8 | 3);
    const auto again = fresh.captureImage();
    EXPECT_EQ(again.rgb, image.rgb);
    EXPECT_EQ(again.depth, image.depth);
    EXPECT_EQ(fresh.capture().rgba, full.rgba);
    fresh.draw(both, view(1.2f, 48.f / 40), shadowed, 2, 48, 40);
    const auto redrawn = fresh.captureImage();
    EXPECT_EQ(redrawn.rgb, image.rgb);
    EXPECT_EQ(redrawn.depth, image.depth);
    // Against a renderer that never ran a label pass (textured quad, shadows on).
    r::Renderer baseline(NEREUS_RENDERING_SHADERS);
    baseline.draw(both, view(1.2f, 48.f / 40), shadowed, 2, 48, 40);
    const auto expected = baseline.captureImage();
    EXPECT_EQ(image.rgb, expected.rgb);
    EXPECT_EQ(image.depth, expected.depth);
}

TEST_F(RendererImage, AbandonContextReleasesCpuOwnersWithoutAContextAndIsTerminal) {
    r::Renderer lost(NEREUS_RENDERING_SHADERS);
    auto mesh = quad();
    std::weak_ptr<const r::MeshAsset> retained = mesh;
    lost.draw(scene(mesh), view(), plain(), 0, 16, 16);
    const auto parts = directory / "parts.png";
    writePng(parts, 2, 2, 3, std::vector<std::uint8_t>(2 * 2 * 3, 3));
    lost.drawLabels(scene(mesh), {{1, {{0, parts}}}}, view(), 16, 16); // label target and a part map loaded
    mesh.reset();
    EXPECT_FALSE(retained.expired());
    glfwMakeContextCurrent(nullptr);
    lost.abandonContext();
    EXPECT_TRUE(retained.expired());
    EXPECT_NO_THROW(lost.abandonContext());
    EXPECT_THROW(lost.capture(), std::logic_error);
    EXPECT_THROW(lost.captureImage(), std::logic_error);
    EXPECT_THROW(lost.draw(scene(quad()), view(), plain(), 0, 16, 16), std::logic_error);
    EXPECT_THROW(lost.drawLabels(scene(quad()), {{}}, view(), 16, 16), std::logic_error);
    // This test deliberately abandons GPU names. The fixture destroys their owning
    // context after the suite, just as the offscreen host does after a lost context.
    glfwMakeContextCurrent(context->window);
}

TEST_F(RendererImage, SupersamplingAveragesEachBlockInLinearHdrBeforeToneMapping) {
    const int w = 40, h = 30;
    const auto frame =
        renderer->draw(tiltedQuad(Eigen::Vector4f::Ones()), view(1.2f, 4.f / 3), supersampled(2), 0, w, h);
    EXPECT_EQ(frame.width, w);
    EXPECT_EQ(frame.depth_width, 2 * w);
    EXPECT_EQ(frame.depth_height, 2 * h);
    const auto full = renderer->capture();
    ASSERT_EQ(full.width, w);
    ASSERT_EQ(full.scene_width, 2 * w);
    ASSERT_EQ(full.scene_height, 2 * h);
    ASSERT_EQ(full.rgba.size(), std::size_t(w) * h * 4);
    ASSERT_EQ(full.composite_rgba.size(), std::size_t(4 * w) * h * 4);
    int mixed = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            for (int c = 0; c < 3; ++c) {
                float sum = 0;
                for (int dy = 0; dy < 2; ++dy)
                    for (int dx = 0; dx < 2; ++dx)
                        sum += full.composite_rgba[(std::size_t(2 * y + dy) * 2 * w + 2 * x + dx) * 4 + c];
                const int expected = toneMapped(sum / 4);
                ASSERT_NEAR(full.rgba[(std::size_t(y) * w + x) * 4 + c], expected, 1) << x << "," << y;
                float lo = 1e9f, hi = 0;
                for (int dy = 0; dy < 2; ++dy)
                    for (int dx = 0; dx < 2; ++dx) {
                        const float v = full.composite_rgba[(std::size_t(2 * y + dy) * 2 * w + 2 * x + dx) * 4 + c];
                        lo = std::min(lo, v);
                        hi = std::max(hi, v);
                    }
                mixed += c == 0 && hi - lo > .1f;
            }
    EXPECT_GT(mixed, 20) << "the quad's edges cover part of many blocks";
}

TEST_F(RendererImage, SupersamplingGivesHardEdgesIntermediateValues) {
    // A low-contrast quad (below the single-sample edge smoothing threshold): at 1x every pixel is either
    // the quad or the background; at 2x its edge pixels take values in between.
    const auto tint = Eigen::Vector4f(.22f, .22f, .22f, 1);
    const auto between = [&](int n) {
        renderer->draw(tiltedQuad(tint), view(), supersampled(n), 0, 64, 64);
        const auto image = renderer->captureImage(true, false);
        const int background = image.rgb[3 * index(image, 0, 0)], inside = image.rgb[3 * index(image, 32, 32)];
        EXPECT_GT(std::abs(inside - background), 12) << "quad and background must differ";
        int count = 0;
        for (std::size_t i = 0; i < image.rgb.size(); i += 3) {
            const int v = image.rgb[i];
            const double t = double(v - background) / (inside - background);
            count += t > .2 && t < .8;
        }
        return count;
    };
    EXPECT_EQ(between(1), 0);
    EXPECT_GT(between(2), 40);
}

TEST_F(RendererImage, SupersamplingKeepsSmoothShadingWithinOneLevel) {
    r::PoolGeometry pool;
    pool.dimensions = {4, 4, 2};
    pool.tile_size = 0; // plain floor: nothing finer than a pixel
    const auto render = [&](int n) {
        renderer->draw(r::makePoolScene(pool), lookDown({2, 2, -1}), supersampled(n), 0, 64, 48);
        return renderer->captureImage(true, false);
    };
    const auto one = render(1), two = render(2);
    ASSERT_EQ(one.rgb.size(), two.rgb.size());
    int largest = 0;
    for (std::size_t i = 0; i < one.rgb.size(); ++i)
        largest = std::max(largest, std::abs(int(one.rgb[i]) - int(two.rgb[i])));
    EXPECT_LE(largest, 1);
}

TEST_F(RendererImage, SupersampledDepthIsOneSampleNearestThePixelCentre) {
    // Fronto-parallel quad: its depth is the same everywhere, so away from its edges 2x and 3x read exactly
    // the single-sample depth.
    renderer->draw(scene(quad()), view(), plain(), 0, 48, 40);
    const auto single = renderer->captureImage(false, true);
    for (const int n : {2, 3, 4}) {
        renderer->draw(scene(quad()), view(), supersampled(n), 0, 48, 40);
        const auto image = renderer->captureImage(false, true);
        ASSERT_EQ(image.width, 48);
        ASSERT_EQ(image.depth.size(), 48u * 40);
        EXPECT_EQ(image.depth[index(image, 24, 20)], single.depth[index(single, 24, 20)]) << n;
        EXPECT_LT(image.depth[index(image, 24, 20)], 1.f);
        EXPECT_EQ(image.depth[index(image, 1, 1)], 1.f) << "background";
        // Every pixel, edges included, is exactly the opaque sample at block offset (n/2, n/2): one of the
        // block's own values, never a blend of the two surfaces.
        const auto full = renderer->capture();
        ASSERT_EQ(full.scene_width, n * 48);
        for (int y = 0; y < 40; ++y)
            for (int x = 0; x < 48; ++x)
                ASSERT_EQ(image.depth[index(image, x, y)],
                          full.opaque_depth[std::size_t(n * y + n / 2) * full.scene_width + n * x + n / 2])
                    << n << ": " << x << "," << y;
    }
    // A sloped plane's window depth is affine in the pixel position, so at 2x the sample a quarter pixel right
    // of and above each centre reads the 1x depth interpolated a quarter of the way to the next pixels.
    auto turned = scene(quad());
    turned.instances[0].transform.topLeftCorner<3, 3>() =
        Eigen::AngleAxisf(50 * float(M_PI) / 180, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    renderer->draw(turned, view(), plain(), 0, 48, 40);
    const auto flat = renderer->captureImage(false, true);
    renderer->draw(turned, view(), supersampled(2), 0, 48, 40);
    const auto fine = renderer->captureImage(false, true);
    for (const auto &[x, y] : {std::pair{24, 20}, std::pair{22, 18}, std::pair{26, 23}}) {
        const float d = flat.depth[index(flat, x, y)];
        const float dx = flat.depth[index(flat, x + 1, y)] - d, dy = flat.depth[index(flat, x, y + 1)] - d;
        ASSERT_LT(d, 1.f);
        ASSERT_GT(std::abs(dx), 1e-5f) << "the plane must be sloped";
        // Within 5% of a pixel's depth step (rasterizer snapping); the centre itself is 25% away.
        EXPECT_NEAR(fine.depth[index(fine, x, y)], d + .25f * (dx + dy), .05f * (std::abs(dx) + std::abs(dy)))
            << x << "," << y;
    }
}

TEST_F(RendererImage, SupersampleIsValidatedAndPreviewsHonourIt) {
    GLint maximum = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximum);
    for (const int n : {0, 5, -1}) {
        EXPECT_THROW(renderer->draw(scene(quad()), view(), supersampled(n), 0, 16, 16), std::invalid_argument) << n;
        EXPECT_THROW(renderer->captureImage(), std::logic_error) << "failed draw invalidates frame";
    }
    EXPECT_THROW(renderer->draw(scene(quad()), view(), supersampled(2), 0, maximum / 2 + 1, 1), std::invalid_argument);
    EXPECT_THROW(renderer->draw(scene(quad()), view(), supersampled(4), 0, 1, maximum / 4 + 1), std::invalid_argument);
    auto preview = supersampled(3);
    preview.preview = true;
    const auto frame = renderer->draw(scene(quad()), view(), preview, 0, 20, 10);
    EXPECT_EQ(frame.width, 20);
    EXPECT_EQ(frame.height, 10);
    EXPECT_EQ(frame.depth_width, 60);
    EXPECT_EQ(frame.depth_height, 30);
    const auto plainFrame = renderer->draw(scene(quad()), view(), plain(), 0, 20, 10);
    EXPECT_EQ(plainFrame.depth_width, 20) << "the full frame keeps its own size";
}
