#include <iostream>
#include <robotics/rendering/renderer.hpp>

int main() {
    const auto scene = robotics::rendering::makePoolScene();
    if (!scene.water || scene.instances.size() != 13 || !scene.instances.front().mesh)
        return 1;
    const auto box = robotics::rendering::makeBoxMesh();
    if (box->submeshes.size() != 1 || box->submeshes[0].vertices.size() != 24)
        return 1;
    std::cout << "Pool scene constructed through the installed API without a graphics context.\n";
}
