// Tests for the headless EGL OffscreenRenderer: colour/depth captures, EGL context hygiene and threading, and
// the label pass (ids + depth) used for dataset export. Label ids are (instance id << 8) | part value.
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <future>
#include <gtest/gtest.h>
#include <map>
#include <nereus/rendering/offscreen.hpp>
#include <png.h>
#include <set>
#include <unistd.h>

namespace r = nereus::rendering;
namespace {
// Unit box 3 m in front of the default camera (camera looks down -Z).
r::Scene scene() {
    r::Scene result;
    r::Instance box;
    box.mesh = r::makeBoxMesh();
    box.transform(2, 3) = -3;
    result.instances.push_back(box);
    return result;
}

// Perspective projection with focal 2 (about 53 deg FOV), near 0.05 m, far 100 m.
r::View view() {
    r::View result;
    result.projection.setZero();
    result.projection(0, 0) = result.projection(1, 1) = 2;
    result.projection(2, 2) = -100.05f / 99.95f;
    result.projection(2, 3) = -10.f / 99.95f;
    result.projection(3, 2) = -1;
    return result;
}

// Lighting effects off so captures are cheap and deterministic.
r::Appearance appearance() {
    r::Appearance result;
    result.shadows = result.reflections = false;
    result.caustics = 0;
    return result;
}

// Captures the box scene at width x 32.
r::ImageCapture capture(r::OffscreenRenderer &host, int width = 32) {
    return host.capture(scene(), view(), appearance(), 0, width, 32);
}
} // namespace

// Pixels are owned by the returned image (a later capture doesn't alias them) and no context stays current.
TEST(Offscreen, CapturesWithoutAWindowAndOwnsPixelsAcrossResize) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(host.device().empty());
    const auto image = capture(host);
    EXPECT_EQ(image.rgb.size(), 32u * 32 * 3);
    EXPECT_EQ(image.depth.size(), 32u * 32);
    EXPECT_LT(image.depth[16 * 32 + 16], 1);
    EXPECT_EQ(image.depth[0], 1);
    const auto saved = image.rgb;
    const auto resized = capture(host, 17);
    EXPECT_EQ(resized.rgb.size(), 17u * 32 * 3);
    EXPECT_EQ(image.rgb, saved);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
}

// Captures may run on other threads; concurrent calls are serialized and give identical results.
TEST(Offscreen, CanMoveCaptureToAWorkerAndSerializeConcurrentCalls) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto expected = capture(host);
    auto first = std::async(std::launch::async, [&] { return capture(host); });
    auto second = std::async(std::launch::async, [&] { return capture(host); });
    EXPECT_EQ(first.get().rgb, expected.rgb);
    EXPECT_EQ(second.get().depth, expected.depth);
    EXPECT_EQ(capture(host).rgb, expected.rgb);
}

// Destroying one host keeps the shared EGL display alive for the others; bad sizes throw and leave no
// context current.
TEST(Offscreen, HostsShareDisplayLifetimeAndRecoverFromInvalidInput) {
    auto first = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    r::OffscreenRenderer second(NEREUS_RENDERING_SHADERS);
    const auto expected = capture(second);
    first.reset();
    EXPECT_EQ(capture(second).depth, expected.depth);
    EXPECT_THROW(capture(second, 0), std::invalid_argument);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    EXPECT_EQ(capture(second).rgb, expected.rgb);
}

TEST(Offscreen, FailedConstructionReleasesItsContext) {
    EXPECT_THROW(r::OffscreenRenderer("/nonexistent/nereus-shaders"), std::runtime_error);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}

// A caller's current EGL context, surfaces and bound API are restored after a capture, even a failed one.
TEST(Offscreen, RestoresCallersEglContextAndApiAfterSuccessAndFailure) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    ASSERT_NE(get, nullptr);
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config;
    EGLint count;
    ASSERT_TRUE(eglChooseConfig(display, attributes, &config, 1, &count));
    ASSERT_EQ(count, 1);
    const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    const auto surface = eglCreatePbufferSurface(display, config, size);
    ASSERT_NE(surface, EGL_NO_SURFACE);
    ASSERT_TRUE(eglBindAPI(EGL_OPENGL_API));
    const auto context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
    ASSERT_NE(context, EGL_NO_CONTEXT);
    ASSERT_TRUE(eglMakeCurrent(display, surface, surface, context));
    EXPECT_NO_THROW(capture(host));
    EXPECT_EQ(eglGetCurrentContext(), context);
    EXPECT_EQ(eglGetCurrentSurface(EGL_DRAW), surface);
    EXPECT_EQ(eglGetCurrentSurface(EGL_READ), surface);
    EXPECT_EQ(eglQueryAPI(), EGL_OPENGL_API);
    EXPECT_THROW(capture(host, -1), std::invalid_argument);
    EXPECT_EQ(eglGetCurrentContext(), context);
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglDestroySurface(display, surface);
    eglBindAPI(EGL_OPENGL_ES_API);
    EXPECT_NO_THROW(capture(host));
    EXPECT_EQ(eglQueryAPI(), EGL_OPENGL_ES_API);
}

// The host can be destroyed on a different thread from the one that created it.
TEST(Offscreen, HostCanBeDestroyedOnAWorkerThread) {
    auto host = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    capture(*host);
    std::async(std::launch::async, [host = std::move(host)]() mutable {
        host.reset();
        EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    }).get();
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}

// Destroying the last host must not terminate the display under an external context that is still current.
TEST(Offscreen, LastHostDoesNotInvalidateAnExternalContextOnTheSameDisplay) {
    auto host = std::make_unique<r::OffscreenRenderer>(NEREUS_RENDERING_SHADERS);
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig config;
    EGLint count;
    ASSERT_TRUE(eglChooseConfig(display, attributes, &config, 1, &count));
    ASSERT_EQ(count, 1);
    const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    const auto surface = eglCreatePbufferSurface(display, config, size);
    ASSERT_NE(surface, EGL_NO_SURFACE);
    ASSERT_TRUE(eglBindAPI(EGL_OPENGL_API));
    const auto context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
    ASSERT_NE(context, EGL_NO_CONTEXT);
    ASSERT_TRUE(eglMakeCurrent(display, surface, surface, context));
    host.reset();
    EXPECT_EQ(eglGetCurrentContext(), context);
    EGLint width = 0;
    EXPECT_TRUE(eglQuerySurface(display, surface, EGL_WIDTH, &width));
    EXPECT_EQ(width, 1);
    eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(display, context);
    eglDestroySurface(display, surface);
    eglBindAPI(EGL_OPENGL_ES_API);
}

// A display terminated by someone else between host lifetimes is reinitialized by the next host.
TEST(Offscreen, ReinitializesDisplayTerminatedBetweenHostLifetimes) {
    {
        r::OffscreenRenderer first(NEREUS_RENDERING_SHADERS);
        capture(first);
    }
    const auto get = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    const auto display = get(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    ASSERT_TRUE(eglTerminate(display));
    r::OffscreenRenderer next(NEREUS_RENDERING_SHADERS);
    EXPECT_FALSE(capture(next).rgb.empty());
}

// Label pass (ids + depth) through the offscreen host.
namespace {
namespace fs = std::filesystem;
// Writes an 8-bit PNG (1 = gray, 3 = RGB, 4 = RGBA); rows top row first, as stored in the file.
void writePng(const fs::path &path, int width, int height, int channels, const std::vector<std::uint8_t> &pixels) {
    FILE *file = std::fopen(path.c_str(), "wb");
    ASSERT_NE(file, nullptr);
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png_create_info_struct(png);
    png_init_io(png, file);
    const int type = channels == 1 ? PNG_COLOR_TYPE_GRAY : channels == 3 ? PNG_COLOR_TYPE_RGB : PNG_COLOR_TYPE_RGBA;
    png_set_IHDR(png, info, width, height, 8, type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    for (int y = 0; y < height; ++y)
        png_write_row(png, const_cast<png_bytep>(pixels.data() + std::size_t(y) * width * channels));
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    std::fclose(file);
}

// Unit quad in the YZ plane facing +X at x, uv (0,0) at (y=-.5, z=-.5), scaled by `size`.
std::shared_ptr<r::MeshAsset> quad(float x = 0, float size = 1, std::optional<fs::path> texture = {},
                                   std::vector<r::UvCutout> cutouts = {}, float alpha = 1) {
    auto mesh = std::make_shared<r::MeshAsset>();
    r::Submesh part;
    const float c[4][2] = {{-.5f, -.5f}, {.5f, -.5f}, {.5f, .5f}, {-.5f, .5f}};
    for (const auto &corner : c)
        part.vertices.push_back(
            {{x, corner[0] * size, corner[1] * size}, {1, 0, 0}, {corner[0] + .5f, corner[1] + .5f}});
    part.indices = {0, 1, 2, 0, 2, 3};
    part.material.diffuse_texture = texture;
    part.material.cutouts = std::move(cutouts);
    part.material.base_color.w() = alpha;
    mesh->submeshes.push_back(part);
    mesh->minimum = {x, -.5f * size, -.5f * size};
    mesh->maximum = {x, .5f * size, .5f * size};
    return mesh;
}

// Camera on +X at 1.2 m looking back at the quads; image up is world +Z, image right is world -Y.
r::View front() {
    r::View v;
    const Eigen::Vector3f eye(1.2f, 0, 0), right(0, -1, 0), up(0, 0, 1), back(1, 0, 0);
    v.view.setIdentity();
    v.view.block<1, 3>(0, 0) = right.transpose();
    v.view.block<1, 3>(1, 0) = up.transpose();
    v.view.block<1, 3>(2, 0) = back.transpose();
    v.view.topRightCorner<3, 1>() = -(v.view.topLeftCorner<3, 3>() * eye);
    const float n = .05f, f = 100.f, t = std::tan(.6f);
    v.projection.setZero();
    v.projection(0, 0) = v.projection(1, 1) = 1 / t;
    v.projection(2, 2) = -(f + n) / (f - n);
    v.projection(2, 3) = -2 * f * n / (f - n);
    v.projection(3, 2) = -1;
    v.eye = eye;
    return v;
}

// Scene instance with the given mesh and surface material.
r::Instance item(std::shared_ptr<const r::MeshAsset> mesh, r::SurfaceMaterial material = r::SurfaceMaterial::Asset) {
    r::Instance result;
    result.mesh = std::move(mesh);
    result.material = material;
    return result;
}

// Label for an instance with whole-instance id and no per-submesh parts.
r::InstanceLabel label(std::uint32_t id) {
    r::InstanceLabel result;
    result.id = id;
    return result;
}

// Per-test temporary directory for generated PNGs; ids and depth are indexed with row 0 at the bottom.
class Labels : public ::testing::Test {
  protected:
    void SetUp() override {
        directory = fs::temp_directory_path() / ("nereus_offscreen_labels_" + std::to_string(::getpid()));
        fs::create_directories(directory);
    }
    void TearDown() override {
        fs::remove_all(directory);
    }

    static std::uint32_t at(const r::LabelCapture &image, int column, int row_from_bottom) {
        return image.ids[std::size_t(row_from_bottom) * image.width + column];
    }
    static float depthAt(const r::LabelCapture &image, int column, int row_from_bottom) {
        return image.depth[std::size_t(row_from_bottom) * image.width + column];
    }

    fs::path directory;
};
} // namespace

// A per-submesh part-map PNG is sampled with the same UVs as the diffuse texture, texel for texel.
TEST_F(Labels, PartMapLinesUpTexelForTexelWithTheDiffuseTexture) {
    // 2x2 texture and part map, file rows top first: red 1 | green 2 / blue 3 | white 4.
    const std::vector<std::uint8_t> colours = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
    const std::vector<std::uint8_t> parts = {1, 2, 3, 4};
    writePng(directory / "diffuse.png", 2, 2, 3, colours);
    writePng(directory / "parts.png", 2, 2, 1, parts);
    r::Scene scene;
    scene.instances.push_back(item(quad(0, 1, directory / "diffuse.png")));
    r::InstanceLabel labelled = label(9);
    labelled.submeshes.push_back({0, directory / "parts.png"});
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto ids = host.captureLabels(scene, {labelled}, front(), 64, 64);
    const auto image = host.capture(scene, front(), appearance(), 0, 64, 64);
    ASSERT_EQ(ids.ids.size(), 64u * 64);
    // Looking at the quad's front from +X: uv v up the image (file top row at the image top) and u to the
    // left (image right is world -Y = u 0). So the file's top-left texel is at the image's top right.
    EXPECT_EQ(at(ids, 42, 42), 9u << 8 | 1);
    EXPECT_EQ(at(ids, 22, 42), 9u << 8 | 2);
    EXPECT_EQ(at(ids, 42, 22), 9u << 8 | 3);
    EXPECT_EQ(at(ids, 22, 22), 9u << 8 | 4);
    EXPECT_EQ(at(ids, 0, 0), 0u) << "background";
    // Every pixel well inside one texel (linear filtering blends across seams and the repeat-wrapped edges):
    // its part is the file texel whose colour the colour pass shows there. Reference colours come from the
    // colour pass at texel centres, named by their colour alone (red, green, blue or white).
    const auto pixel = [&](int column, int row) {
        const auto *rgb = &image.rgb[3 * (std::size_t(row) * 64 + column)];
        return Eigen::Vector3f(rgb[0], rgb[1], rgb[2]);
    };
    std::map<std::uint32_t, Eigen::Vector3f> references;
    for (const auto &[column, row] : {std::pair{42, 42}, {22, 42}, {42, 22}, {22, 22}}) {
        const Eigen::Vector3f colour = pixel(column, row);
        Eigen::Index channel = 0;
        colour.maxCoeff(&channel);
        references[colour.minCoeff() > 128 ? 4u : std::uint32_t(channel) + 1] = colour;
    }
    ASSERT_EQ(references.size(), 4u);
    std::set<std::uint32_t> seen;
    int checked = 0;
    const auto inside = [&](int column, int row) {
        return column >= 0 && row >= 0 && column < 64 && row < 64 && at(ids, column, row) != 0;
    };
    for (int row = 0; row < 64; ++row)
        for (int column = 0; column < 64; ++column) {
            const int margin = 6;
            if (std::abs(row - 32) < margin || std::abs(column - 32) < margin || !inside(column - margin, row) ||
                !inside(column + margin, row) || !inside(column, row - margin) || !inside(column, row + margin))
                continue;
            const auto value = at(ids, column, row);
            ASSERT_EQ(value >> 8, 9u);
            std::uint32_t texel = 0;
            float best = INFINITY;
            for (const auto &[part, colour] : references)
                if ((pixel(column, row) - colour).norm() < best) {
                    best = (pixel(column, row) - colour).norm();
                    texel = part;
                }
            ASSERT_EQ(value & 0xff, texel) << "pixel " << column << ", " << row;
            seen.insert(value & 0xff);
            ++checked;
        }
    EXPECT_EQ(seen, (std::set<std::uint32_t>{1, 2, 3, 4}));
    EXPECT_GT(checked, 200);
    // A fixed part value applies to the whole submesh; empty submeshes = part 0.
    labelled.submeshes = {{7, std::nullopt}};
    EXPECT_EQ(at(host.captureLabels(scene, {labelled}, front(), 64, 64), 32, 32), 9u << 8 | 7);
    EXPECT_EQ(at(host.captureLabels(scene, {label(9)}, front(), 64, 64), 32, 32), 9u << 8);
}

// id 0 marks an occluder that hides labelled geometry; clear or invisible covers occlude only if labelled.
TEST_F(Labels, UnlabelledOccluderHidesALabelledQuadButClearOnlyWhenLabelled) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    r::Scene scene;
    scene.instances.push_back(item(quad()));
    scene.instances.push_back(item(quad(.3f, .4f))); // small quad in front of the centre
    const auto occluded = host.captureLabels(scene, {label(5), label(0)}, front(), 64, 64);
    EXPECT_EQ(at(occluded, 32, 32), 0u) << "id 0 occludes";
    EXPECT_LT(depthAt(occluded, 32, 32), 1.f);
    EXPECT_EQ(at(occluded, 20, 32), 5u << 8);
    EXPECT_EQ(at(host.captureLabels(scene, {label(5), label(6)}, front(), 64, 64), 32, 32), 6u << 8);
    // Clear cover (material Clear, or an Asset submesh with alpha < .999): see-through unless labelled.
    for (const bool material : {true, false}) {
        scene.instances[1] =
            material ? item(quad(.3f, .4f), r::SurfaceMaterial::Clear) : item(quad(.3f, .4f, {}, {}, .5f));
        const auto clear = host.captureLabels(scene, {label(5), label(0)}, front(), 64, 64);
        EXPECT_EQ(at(clear, 32, 32), 5u << 8) << "unlabelled clear cover does not occlude";
        EXPECT_EQ(depthAt(clear, 32, 32), depthAt(clear, 20, 32)) << "depth of the quad behind";
        EXPECT_EQ(at(host.captureLabels(scene, {label(5), label(6)}, front(), 64, 64), 32, 32), 6u << 8)
            << "a labelled clear cover is drawn";
    }
    // Invisible instances are skipped.
    scene.instances[1] = item(quad(.3f, .4f));
    scene.instances[1].visible = false;
    EXPECT_EQ(at(host.captureLabels(scene, {label(5), label(6)}, front(), 64, 64), 32, 32), 5u << 8);
}

// Shader cutouts and low-alpha texels discard fragments in the label pass, as in the colour pass.
TEST_F(Labels, CutoutsAndTransparentTexelsDiscardLikeTheColourPass) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    r::Scene scene;
    scene.instances.push_back(item(quad(0, 1, {}, {{{.5f, .5f}, .2f}})));
    const auto cut = host.captureLabels(scene, {label(3)}, front(), 64, 64);
    EXPECT_EQ(at(cut, 32, 32), 0u);
    EXPECT_EQ(depthAt(cut, 32, 32), 1.f);
    EXPECT_EQ(at(cut, 32, 46), 3u << 8);
    std::vector<std::uint8_t> clear(4 * 4 * 4, 255);
    for (std::size_t i = 3; i < clear.size(); i += 4)
        clear[i] = 50;
    writePng(directory / "clear.png", 4, 4, 4, clear);
    scene.instances[0] = item(quad(0, 1, directory / "clear.png"));
    EXPECT_EQ(at(host.captureLabels(scene, {label(3)}, front(), 64, 64), 32, 32), 0u);
}

TEST_F(Labels, PartMapValuesFillCutoutsAndZeroKeepsThemOpen) {
    // Part map: file column 0 (uv u < .5, the image's right half here) is 5, column 1 is 0. A hole at the
    // centre keeps the fragments labelled 5 (a ring's value fills its hole) and discards the rest.
    writePng(directory / "halves.png", 2, 2, 1, {5, 0, 5, 0});
    r::Scene scene;
    scene.instances.push_back(item(quad(0, 1, {}, {{{.5f, .5f}, .2f}})));
    auto labelled = label(3);
    labelled.submeshes = {{0, directory / "halves.png"}};
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto ids = host.captureLabels(scene, {labelled}, front(), 64, 64);
    EXPECT_EQ(at(ids, 36, 32), 3u << 8 | 5) << "hole fragment with a part value is kept";
    EXPECT_LT(depthAt(ids, 36, 32), 1.f);
    EXPECT_EQ(at(ids, 28, 32), 0u) << "hole fragment with part 0 is discarded";
    EXPECT_EQ(depthAt(ids, 28, 32), 1.f);
    EXPECT_EQ(at(ids, 36, 46), 3u << 8 | 5);
    EXPECT_EQ(at(ids, 28, 46), 3u << 8) << "outside the hole part 0 is the instance without a part";
    // A fixed part value is not a part map: the hole stays open.
    labelled.submeshes = {{5, std::nullopt}};
    EXPECT_EQ(at(host.captureLabels(scene, {labelled}, front(), 64, 64), 36, 32), 0u);
    // The colour pass keeps the whole hole see-through.
    EXPECT_EQ(host.capture(scene, front(), appearance(), 0, 64, 64).depth[32 * 64 + 36], 1.f);
    // Id 0 only occludes: its part map neither fills the hole nor reaches the output.
    labelled = label(0);
    labelled.submeshes = {{0, directory / "halves.png"}};
    const auto occluder = host.captureLabels(scene, {labelled}, front(), 64, 64);
    EXPECT_EQ(at(occluder, 36, 32), 0u);
    EXPECT_EQ(depthAt(occluder, 36, 32), 1.f) << "hole stays see-through";
    EXPECT_EQ(at(occluder, 36, 46), 0u);
    EXPECT_LT(depthAt(occluder, 36, 46), 1.f) << "solid part still occludes";
}

// Part maps are loaded (and validated) even when their submesh is off screen.
TEST_F(Labels, PartMapsOfOffscreenSubmeshesAreStillLoaded) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    r::Scene scene;
    scene.instances.push_back(item(quad(5))); // behind the camera
    auto labelled = label(1);
    labelled.submeshes = {{0, directory / "missing.png"}};
    EXPECT_THROW(host.captureLabels(scene, {labelled}, front(), 16, 16), std::invalid_argument);
    writePng(directory / "parts.png", 1, 1, 1, {4});
    labelled.submeshes = {{0, directory / "parts.png"}};
    const auto ids = host.captureLabels(scene, {labelled}, front(), 16, 16);
    EXPECT_EQ(std::set<std::uint32_t>(ids.ids.begin(), ids.ids.end()), std::set<std::uint32_t>{0});
}

// Label depth equals colour depth; the water surface and Marking stripes never get label ids.
TEST_F(Labels, DepthMatchesTheColourPassAndWaterAndMarkingsAreNotDrawn) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    auto scene = ::scene();                        // box 3 m away
    scene.instances.push_back(item(quad(0, .6f))); // tilted quad in front of the box
    scene.instances.back().transform.block<3, 3>(0, 0) =
        (Eigen::AngleAxisf(-1.2f, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(.3f, Eigen::Vector3f::UnitZ()))
            .toRotationMatrix();
    scene.instances.back().transform.topRightCorner<3, 1>() = Eigen::Vector3f(.25f, .1f, -2);
    const auto image = host.capture(scene, view(), appearance(), 0, 48, 40);
    const auto labels = host.captureLabels(scene, {label(1), label(2)}, view(), 48, 40);
    EXPECT_EQ(labels.width, 48);
    EXPECT_EQ(labels.height, 40);
    EXPECT_EQ(labels.depth, image.depth);
    std::set<std::uint32_t> ids(labels.ids.begin(), labels.ids.end());
    EXPECT_EQ(ids, (std::set<std::uint32_t>{0, 1u << 8, 2u << 8}));
    // A pool scene (water surface + Marking stripes): every instance labelled, no stripe id appears.
    r::PoolGeometry geometry;
    geometry.dimensions = {4, 4, 2};
    geometry.markings.push_back({r::PoolSide::Floor, {1, 2}, {3, 2}, .3f, {0, 0, 0}});
    const auto pool = r::makePoolScene(geometry);
    ASSERT_TRUE(pool.water.has_value());
    std::vector<r::InstanceLabel> each;
    std::set<std::uint32_t> markings;
    for (std::size_t i = 0; i < pool.instances.size(); ++i) {
        each.push_back(label(std::uint32_t(i + 1)));
        if (pool.instances[i].material == r::SurfaceMaterial::Marking)
            markings.insert(std::uint32_t(i + 1) << 8);
    }
    ASSERT_FALSE(markings.empty());
    r::View down = view();
    down.view.setIdentity();
    down.view.topRightCorner<3, 1>() = Eigen::Vector3f(-2, -2, 1); // eye (2, 2, -1) looking down -Z
    down.eye = {2, 2, -1};
    const auto floor = host.captureLabels(pool, each, down, 32, 32);
    EXPECT_NE(floor.ids[16 * 32 + 16], 0u);
    for (auto value : floor.ids)
        ASSERT_EQ(markings.count(value), 0u);
}

// Label captures don't change later colour captures; invalid labels and sizes throw.
TEST_F(Labels, LabelPassesDoNotDisturbCapturesAndInvalidInputIsRejected) {
    r::OffscreenRenderer host(NEREUS_RENDERING_SHADERS);
    const auto expected = capture(host);
    std::vector<std::uint8_t> parts = {1, 2, 3, 4};
    writePng(directory / "parts.png", 2, 2, 1, parts);
    auto labelled = label(1);
    labelled.submeshes = {{2, directory / "parts.png"}};
    EXPECT_FALSE(host.captureLabels(scene(), {labelled}, view(), 40, 24).ids.empty());
    const auto after = capture(host);
    EXPECT_EQ(after.rgb, expected.rgb);
    EXPECT_EQ(after.depth, expected.depth);
    EXPECT_THROW(host.captureLabels(scene(), {}, view(), 32, 32), std::invalid_argument);
    EXPECT_THROW(host.captureLabels(scene(), {label(1u << 24)}, view(), 32, 32), std::invalid_argument);
    auto wrong = label(1);
    wrong.submeshes.resize(2);
    EXPECT_THROW(host.captureLabels(scene(), {wrong}, view(), 32, 32), std::invalid_argument);
    EXPECT_THROW(host.captureLabels(scene(), {label(1)}, view(), 0, 32), std::invalid_argument);
    auto missing = label(1);
    missing.submeshes = {{0, directory / "missing.png"}};
    EXPECT_THROW(host.captureLabels(scene(), {missing}, view(), 32, 32), std::invalid_argument);
    EXPECT_EQ(eglGetCurrentContext(), EGL_NO_CONTEXT);
    EXPECT_EQ(host.captureLabels(scene(), {labelled}, view(), 32, 32).ids[16 * 32 + 16] >> 8, 1u);
    EXPECT_EQ(capture(host).rgb, expected.rgb);
}
