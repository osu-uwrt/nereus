#include "benchmark.hpp"
// Offline original renderer harness: no platform, ROS node, panels or wall clock.
#include "pool_viewer/renderer.hpp"
#include <GLFW/glfw3.h>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

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
        window = glfwCreateWindow(640, 400, "Original fixed renderer", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            throw std::runtime_error("GL context creation failed");
        }
        glfwMakeContextCurrent(window);
        if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
            glfwDestroyWindow(window);
            glfwTerminate();
            throw std::runtime_error("GLAD initialization failed");
        }
    }
    ~Window() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
};
template <class T> void binary(const std::filesystem::path &path, const std::vector<T> &data) {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(data.data()),
              static_cast<std::streamsize>(data.size() * sizeof(T)));
    out.close();
    if (!out)
        throw std::runtime_error("cannot write " + path.string());
}
void capture(const pool::Frame &f, const std::filesystem::path &prefix) {
    constexpr std::size_t pixels = 640 * 400;
    std::vector<float> rgba(pixels * 4), depth(pixels);
    for (const auto &item :
         {std::make_pair(&f.opaque, ".opaque"), std::make_pair(&f.composite, ".composite")}) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, item.first->fbo);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        glReadPixels(0, 0, 640, 400, GL_RGBA, GL_FLOAT, rgba.data());
        binary(prefix.string() + item.second, rgba);
        glReadPixels(0, 0, 640, 400, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());
        binary(prefix.string() + (item.first == &f.opaque ? ".depth" : ".composite-depth"), depth);
    }
    std::vector<unsigned char> final(pixels * 4);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, f.final.fbo);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, 640, 400, GL_RGBA, GL_UNSIGNED_BYTE, final.data());
    binary(prefix.string() + ".rgba", final);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            return 2;
        Window context;
        std::cout << "Renderer: " << glGetString(GL_RENDERER) << '\n';
        const std::filesystem::path root(argv[1]), output(argv[2]);
        std::filesystem::create_directories(output);
        std::vector<pool::ThrusterRotor> rotors;
        for (const auto *id : {"VUS", "VUP", "HUS", "HUP", "HLS", "HLP", "VLS", "VLP"}) {
            pool::ThrusterRotor rotor;
            rotor.id = id;
            rotor.mesh =
                (root / "camera_faker/models/talos3/rotors" / (std::string(id) + ".glb")).string();
            rotors.push_back(rotor);
        }
        pool::Renderer renderer((root / "camera_faker/shaders/pool").string(), root.string(),
                                (root / "camera_faker/textures").string(),
                                (root / "mapping.yaml").string(), (root / "markers.yaml").string(),
                                (root / "scene.yaml").string(), "fixture",
                                (root / "camera_faker/models/talos3/Talos3_body.glb").string(), "",
                                "", "", "", {}, rotors);
        renderer.robotPose(glm::translate(glm::mat4(1), {3, 0, -1}) *
                           glm::translate(glm::mat4(1), {.157f, -.040f, .048f}));
        pool::Frame frame;
        frame.resize(640, 400);
        const std::vector<glm::vec3> eyes = {{6, -4, 3},         {4.5f, -2, -.8f}, {3.8f, -1, .03f},
                                             {3.5f, -.7f, -.9f}, {6, -4, 3},       {6, -4, 3}};
        for (std::size_t i = 0; i < eyes.size(); ++i) {
            pool::View view;
            view.view = glm::lookAt(eyes[i], glm::vec3(3, 0, -1), glm::vec3(0, 0, 1));
            view.projection = glm::perspective(glm::radians(65.f), 640.f / 400, .05f, 150.f);
            view.eye = eyes[i];
            pool::Look appearance;
            appearance.outdoor = i != 1 && i != 5;
            appearance.shadows = i != 5;
            appearance.surfaceReflections = i != 3 && i != 4;
            appearance.tag = false;
            std::vector<float> camera(glm::value_ptr(view.view), glm::value_ptr(view.view) + 16);
            camera.insert(camera.end(), glm::value_ptr(view.projection),
                          glm::value_ptr(view.projection) + 16);
            camera.insert(camera.end(), glm::value_ptr(view.eye), glm::value_ptr(view.eye) + 3);
            binary(output / (std::to_string(i) + ".view"), camera);
            renderer.shadows(appearance);
            renderer.render(frame, view, appearance, 12.5f, true, true, false);
            capture(frame, output / std::to_string(i));
            if (glGetError() != GL_NO_ERROR)
                throw std::runtime_error("OpenGL error during original capture");
        }
        pool::View view;
        view.view = glm::lookAt(eyes[0], glm::vec3(3, 0, -1), glm::vec3(0, 0, 1));
        view.projection = glm::perspective(glm::radians(65.f), 1280.f / 800, .05f, 150.f);
        view.eye = eyes[0];
        pool::Look appearance;
        appearance.outdoor = true;
        appearance.tag = false;
        frame.resize(1280, 800);
        benchmark(
            output / "benchmark.json",
            [&](float time) {
                renderer.shadows(appearance);
                renderer.render(frame, view, appearance, time, true, true, false);
            },
            [] { glFinish(); });
        const pool::StatusLights lights(
            (root / "c_simulator/robots/talos/config/status_lights.yaml").string());
        pool::Renderer illuminated(
            (root / "camera_faker/shaders/pool").string(), root.string(),
            (root / "camera_faker/textures").string(), (root / "mapping.yaml").string(),
            (root / "markers.yaml").string(), (root / "scene.yaml").string(), "fixture",
            (root / "camera_faker/models/talos3/Talos3_body.glb").string(), "", "", "", "",
            lights.lights, rotors);
        illuminated.robotPose(glm::translate(glm::mat4(1), {3, 0, -1}) *
                              glm::translate(glm::mat4(1), {.157f, -.040f, .048f}));
        std::vector<float> geometry;
        for (const auto &light : lights.lights) {
            const auto mount = glm::scale(light.mount, light.size);
            geometry.insert(geometry.end(), glm::value_ptr(mount), glm::value_ptr(mount) + 16);
            geometry.push_back(light.radiance);
        }
        if (lights.lights.size() != 3)
            throw std::runtime_error("unexpected pinned indicator count");
        binary(output / "indicators.lights", geometry);
        frame.resize(640, 400);
        view.eye = {3.1f, .65f, -.5f};
        view.view = glm::lookAt(view.eye, glm::vec3(3.014f, .1173f, -.858f), glm::vec3(0, 0, 1));
        view.projection = glm::perspective(glm::radians(65.f), 640.f / 400, .05f, 150.f);
        appearance = pool::Look{};
        appearance.tag = false;
        for (int variant = 0; variant < 3; ++variant) {
            for (std::size_t i = 0; i < lights.lights.size(); ++i) {
                glm::vec3 rgb(0);
                if (variant == 1)
                    rgb.r = 1;
                else if (variant == 2)
                    rgb[static_cast<int>(i)] = 1;
                illuminated.statusLight(lights.lights[i].id, rgb);
            }
            std::vector<float> camera(glm::value_ptr(view.view), glm::value_ptr(view.view) + 16);
            camera.insert(camera.end(), glm::value_ptr(view.projection),
                          glm::value_ptr(view.projection) + 16);
            camera.insert(camera.end(), glm::value_ptr(view.eye), glm::value_ptr(view.eye) + 3);
            binary(output / (std::to_string(6 + variant) + ".view"), camera);
            illuminated.shadows(appearance);
            illuminated.render(frame, view, appearance, 12.5f, true, true, false);
            capture(frame, output / std::to_string(6 + variant));
            if (glGetError() != GL_NO_ERROR)
                throw std::runtime_error("OpenGL error during original indicator capture");
        }
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
