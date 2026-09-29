#include "benchmark.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <robotics/rendering/renderer.hpp>

namespace r = robotics::rendering;
namespace {
struct Window {
    GLFWwindow *window = nullptr;
    Window() {
        if (!glfwInit())
            throw std::runtime_error("GLFW initialization failed");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(640, 400, "Fixed render capture", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            throw std::runtime_error("GL context creation failed");
        }
        glfwMakeContextCurrent(window);
        glewExperimental = GL_TRUE;
        if (glewInit() != GLEW_OK) {
            glfwDestroyWindow(window);
            glfwTerminate();
            throw std::runtime_error("GLEW initialization failed");
        }
        while (glGetError() != GL_NO_ERROR) {
        }
    }
    ~Window() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
};
Eigen::Matrix4f eigen(const glm::mat4 &m) {
    return Eigen::Map<const Eigen::Matrix4f>(glm::value_ptr(m));
}
template <class T> void binary(const std::filesystem::path &path, const std::vector<T> &data) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(data.data()),
              static_cast<std::streamsize>(data.size() * sizeof(T)));
    out.close();
    if (!out)
        throw std::runtime_error("cannot write " + path.string());
}
void checkContracts(r::Renderer &renderer, const r::Scene &scene, const r::View &view,
                    const r::Appearance &appearance) {
    const auto equal = [](const r::Capture &a, const r::Capture &b) {
        return a.rgba == b.rgba && a.opaque_rgba == b.opaque_rgba &&
               a.opaque_depth == b.opaque_depth && a.composite_rgba == b.composite_rgba &&
               a.composite_depth == b.composite_depth;
    };
    renderer.draw(scene, view, appearance, 12.5f, 640, 400);
    const auto baseline = renderer.capture();
    if (baseline.opaque_depth != baseline.composite_depth)
        throw std::runtime_error("water unexpectedly changed scene depth");
    glEnable(GL_SCISSOR_TEST);
    glScissor(0, 0, 0, 0);
    glEnable(GL_STENCIL_TEST);
    glEnable(GL_COLOR_LOGIC_OP);
    glLogicOp(GL_XOR);
    glFrontFace(GL_CW);
    glDepthFunc(GL_GREATER);
    glDepthRange(.2, .8);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_FRAMEBUFFER_SRGB);
    glEnable(GL_DEPTH_CLAMP);
    glEnable(GL_CLIP_DISTANCE0);
    glDisable(GL_DITHER);
    glEnable(GL_RASTERIZER_DISCARD);
    glPrimitiveRestartIndex(0);
    glEnable(GL_PRIMITIVE_RESTART);
    glPixelStorei(GL_PACK_SWAP_BYTES, GL_TRUE);
    glPixelStorei(GL_PACK_ROW_LENGTH, 999);
    GLuint sampler = 0;
    glGenSamplers(1, &sampler);
    glSamplerParameteri(sampler, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    for (int i = 0; i < 3; ++i)
        glBindSampler(i, sampler);
    renderer.draw(scene, view, appearance, 12.5f, 640, 400);
    const auto dirty = renderer.capture();
    glDeleteSamplers(1, &sampler);
    if (!equal(baseline, dirty))
        throw std::runtime_error("caller GL state changed rendered output");
    renderer.draw(scene, view, appearance, 12.5f, 321, 199);
    const auto resized = renderer.capture();
    if (resized.rgba.size() != 321U * 199U * 4U)
        throw std::runtime_error("resize dimensions changed");
    renderer.draw(scene, view, appearance, 12.5f, 640, 400);
    if (!equal(baseline, renderer.capture()))
        throw std::runtime_error("resize/replay changed output");
    renderer.draw(scene, view, appearance, 13.5f, 640, 400);
    if (baseline.rgba == renderer.capture().rgba)
        throw std::runtime_error("explicit animation time had no effect");
    r::Scene dry = scene;
    dry.water.reset();
    renderer.draw(dry, view, appearance, 12.5f, 640, 400);
    const auto dry_baseline = renderer.capture();
    auto different_water = appearance;
    different_water.water.absorption = {4, 3, 2};
    different_water.water.tint = {1, 0, 0};
    different_water.water.scattering = 5;
    different_water.caustics = 10;
    renderer.draw(dry, view, different_water, 12.5f, 640, 400);
    if (!equal(dry_baseline, renderer.capture()))
        throw std::runtime_error("absent water still changed appearance");
    auto invalid = scene;
    invalid.water->surface.transform(2, 3) += .1f;
    bool rejected = false;
    try {
        renderer.draw(invalid, view, appearance, 12.5f, 640, 400);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    if (!rejected)
        throw std::runtime_error("inconsistent water plane was accepted");
    rejected = false;
    try {
        renderer.capture();
    } catch (const std::logic_error &) {
        rejected = true;
    }
    if (!rejected)
        throw std::runtime_error("capture accepted an unsuccessful draw");
    renderer.draw(scene, view, appearance, 12.5f, 640, 400);
    if (!equal(baseline, renderer.capture()))
        throw std::runtime_error("failed draw damaged later rendering");
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc != 4 && argc != 5) {
            std::cerr << "robotics-render-capture SHADER_DIRECTORY TALOS_PACK OUTPUT_DIRECTORY "
                         "[REFERENCE_INPUTS]\n";
            return 2;
        }
        Window context;
        std::cout << "Renderer: " << glGetString(GL_RENDERER) << '\n';
        const std::filesystem::path assets(argv[2]), output(argv[3]);
        std::filesystem::create_directories(output);
        r::PoolGeometry pool;
        // Same pool-to-map mapping as the native competition pool.
        const auto mapToPool = glm::translate(glm::mat4(1), {19.5136f, 0, 0}) *
                               glm::rotate(glm::mat4(1), glm::radians(90.f), {0, 0, 1});
        pool.local_to_world = eigen(glm::inverse(mapToPool));
        auto scene = r::makePoolScene(pool);
        const auto model = glm::translate(glm::mat4(1), {3, 0, -1}) *
                           glm::translate(glm::mat4(1), {.157f, -.040f, .048f});
        for (const auto *name : {"Talos3_body.glb", "rotors/VUS.glb", "rotors/VUP.glb",
                                 "rotors/HUS.glb", "rotors/HUP.glb", "rotors/HLS.glb",
                                 "rotors/HLP.glb", "rotors/VLS.glb", "rotors/VLP.glb"}) {
            r::Instance object;
            object.mesh = std::make_shared<const r::MeshAsset>(r::loadMesh(assets / name));
            object.transform = eigen(model);
            scene.instances.push_back(std::move(object));
        }
        auto indicator_scene = scene;
        const bool have_indicators =
            argc == 5 &&
            std::filesystem::exists(std::filesystem::path(argv[4]) / "indicators.lights");
        if (have_indicators) {
            // Shared fixed input captured from the unchanged original config loader.
            // Public scene loading and source-color binding are validated separately.
            std::ifstream lights(std::filesystem::path(argv[4]) / "indicators.lights",
                                 std::ios::binary);
            const auto geometry = r::makeBoxMesh();
            for (int i = 0; i < 3; ++i) {
                r::Instance instance;
                Eigen::Matrix4f mount;
                lights.read(reinterpret_cast<char *>(mount.data()), 16 * sizeof(float));
                lights.read(reinterpret_cast<char *>(&instance.radiance), sizeof(float));
                instance.mesh = geometry;
                instance.transform = eigen(model * glm::make_mat4(mount.data()));
                instance.material = r::SurfaceMaterial::Emissive;
                instance.casts_shadow = false;
                indicator_scene.instances.push_back(std::move(instance));
            }
            if (!lights || lights.peek() != std::char_traits<char>::eof())
                throw std::runtime_error("invalid indicator reference geometry");
        }
        // A caller may have a pixel upload buffer bound when the renderer is constructed.
        GLuint unpack = 0;
        glGenBuffers(1, &unpack);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, unpack);
        glBufferData(GL_PIXEL_UNPACK_BUFFER, 4, nullptr, GL_STATIC_DRAW);
        r::Renderer renderer(argv[1]);
        glDeleteBuffers(1, &unpack);
        const std::vector<glm::vec3> eyes = {
            {6, -4, 3}, {4.5f, -2, -.8f},   {3.8f, -1, .03f},   {3.5f, -.7f, -.9f}, {6, -4, 3},
            {6, -4, 3}, {3.1f, .65f, -.5f}, {3.1f, .65f, -.5f}, {3.1f, .65f, -.5f}};
        for (std::size_t i = 0; i < (have_indicators ? eyes.size() : 6U); ++i) {
            r::View view;
            view.view = eigen(glm::lookAt(eyes[i], glm::vec3(3, 0, -1), glm::vec3(0, 0, 1)));
            view.projection = eigen(glm::perspective(glm::radians(65.f), 640.f / 400, .05f, 150.f));
            view.eye = {eyes[i].x, eyes[i].y, eyes[i].z};
            if (argc == 5) {
                std::ifstream input(std::filesystem::path(argv[4]) / (std::to_string(i) + ".view"),
                                    std::ios::binary);
                input.read(reinterpret_cast<char *>(view.view.data()), 16 * sizeof(float));
                input.read(reinterpret_cast<char *>(view.projection.data()), 16 * sizeof(float));
                input.read(reinterpret_cast<char *>(view.eye.data()), 3 * sizeof(float));
                if (!input || input.peek() != std::char_traits<char>::eof())
                    throw std::runtime_error("invalid reference view");
            }
            r::Appearance appearance;
            appearance.outdoor = i < 6 && i != 1 && i != 5;
            appearance.shadows = i != 5;
            appearance.reflections = i != 3 && i != 4;
            if (i >= 6) {
                for (std::size_t light = 0; light < 3; ++light) {
                    auto &instance = indicator_scene.instances[scene.instances.size() + light];
                    instance.tint = {0, 0, 0, 1};
                    if (i == 7)
                        instance.tint[0] = 1;
                    else if (i == 8)
                        instance.tint[static_cast<Eigen::Index>(light)] = 1;
                }
            }
            renderer.draw(i < 6 ? scene : indicator_scene, view, appearance, 12.5f, 640, 400);
            const auto capture = renderer.capture();
            const auto prefix = output / std::to_string(i);
            binary(prefix.string() + ".rgba", capture.rgba);
            binary(prefix.string() + ".opaque", capture.opaque_rgba);
            binary(prefix.string() + ".depth", capture.opaque_depth);
            binary(prefix.string() + ".composite", capture.composite_rgba);
            binary(prefix.string() + ".composite-depth", capture.composite_depth);
            std::ofstream ppm(prefix.string() + ".ppm", std::ios::binary);
            ppm << "P6\n640 400\n255\n";
            for (int y = 399; y >= 0; --y)
                for (int x = 0; x < 640; ++x)
                    ppm.write(
                        reinterpret_cast<const char *>(capture.rgba.data() + 4 * (y * 640 + x)), 3);
            if (i == 0)
                checkContracts(renderer, scene, view, appearance);
            if (glGetError() != GL_NO_ERROR)
                throw std::runtime_error("OpenGL error during capture");
        }
        r::View view;
        view.view = eigen(glm::lookAt(eyes[0], glm::vec3(3, 0, -1), glm::vec3(0, 0, 1)));
        view.projection = eigen(glm::perspective(glm::radians(65.f), 1280.f / 800, .05f, 150.f));
        view.eye = {eyes[0].x, eyes[0].y, eyes[0].z};
        r::Appearance appearance;
        appearance.outdoor = true;
        benchmark(
            output / "benchmark.json",
            [&](float time) { renderer.draw(scene, view, appearance, time, 1280, 800); },
            [] { glFinish(); });
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
