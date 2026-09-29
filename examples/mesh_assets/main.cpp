#include <iostream>
#include <robotics/rendering/assets.hpp>

int main(int argc, char **argv) {
    if (argc != 2)
        return 2;
    const std::filesystem::path root(argv[1]);
    std::size_t triangles = 0;
    for (const auto *name :
         {"Talos3_body.glb", "rotors/VUS.glb", "rotors/VUP.glb", "rotors/HUS.glb", "rotors/HUP.glb",
          "rotors/HLS.glb", "rotors/HLP.glb", "rotors/VLS.glb", "rotors/VLP.glb"}) {
        const auto mesh = robotics::rendering::loadMesh(root / name);
        for (const auto &part : mesh.submeshes)
            triangles += part.indices.size() / 3;
    }
    std::cout << "Original body/rotor triangles: " << triangles << '\n';
    return triangles == 1087230 ? 0 : 1;
}
