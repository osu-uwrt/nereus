#include <iostream>
#include <robotics/rendering/renderer.hpp>

int main() {
    const auto scene = robotics::rendering::makePoolScene();
    if (!scene.water || scene.instances.size() != 13 || !scene.instances.front().mesh)
        return 1;
    std::cout << "Pool scene constructed through the installed API without a graphics context.\n";
}
