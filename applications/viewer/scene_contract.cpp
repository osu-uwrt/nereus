#include "desktop.hpp"
#include <GL/glew.h>
#include <iostream>
#include <robotics/rendering/renderer.hpp>
#include <robotics/visualization/camera.hpp>
#include <stdexcept>

namespace {
std::vector<unsigned char> read(unsigned int texture, int width, int height) {
    std::vector<unsigned char> result(static_cast<std::size_t>(width) * height * 4);
    glBindTexture(GL_TEXTURE_2D, texture);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, result.data());
    if (glGetError() != GL_NO_ERROR)
        throw std::runtime_error("viewport readback failed");
    return result;
}
void require(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}
} // namespace
int main(int argc, char **argv) {
    try {
        if (argc != 2)
            throw std::invalid_argument("scene-contract SHADER_DIRECTORY");
        robotics::viewer::Desktop desktop(true);
        robotics::rendering::Renderer renderer(argv[1]);
        robotics::viewer::Viewport viewport;
        auto scene = robotics::rendering::makePoolScene();
        robotics::visualization::Camera camera;
        camera.target = {0, 0, -1};
        camera.pitch = 1;
        camera.distance = 5;
        const auto view = robotics::visualization::cameraMatrices(camera, 1.6);
        const auto matrix = (view.projection * view.view).eval();
        for (const auto &size : {std::pair<int, int>{256, 160}, {320, 200}, {256, 160}}) {
            const auto [width, height] = size;
            const auto frame =
                renderer.draw(scene, {view.view, view.projection, view.eye}, {}, 0, width, height);
            const auto before = renderer.capture();
            const robotics::viewer::ViewportBackground background{frame.color_texture,
                                                                  frame.depth_texture};
            const auto plain =
                read(viewport.render({}, matrix, width, height, background), width, height);
            // Viewport RGB8 has opaque alpha, whereas renderer alpha is shader-defined.
            for (std::size_t i = 0; i < plain.size(); ++i)
                if (i % 4 != 3)
                    require(plain[i] == before.rgba[i], "viewport changed background color");
            const std::vector<robotics::visualization::Line> behind{
                {{-1, 0, -4}, {1, 0, -4}, {255, 0, 255}}};
            require(read(viewport.render(behind, matrix, width, height, background), width,
                         height) == plain,
                    "overlay behind floor was not occluded");
            const std::vector<robotics::visualization::Line> front{
                {{-1, 0, 0}, {1, 0, 0}, {255, 0, 255}}};
            require(read(viewport.render(front, matrix, width, height, background), width,
                         height) != plain,
                    "visible overlay was lost");
            const auto after = renderer.capture();
            require(before.rgba == after.rgba && before.composite_depth == after.composite_depth,
                    "viewer overlay modified renderer capture");
        }
        std::cout
            << "Scene viewport color, depth occlusion, resize and capture isolation passed.\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
