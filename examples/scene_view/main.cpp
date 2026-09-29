#include <iostream>
#include <robotics/scene_view/scene.hpp>
#include <robotics/viewer/workspace.hpp>

int main(int argc, char **argv) {
    if (argc != 3)
        return 2;
    const auto workspace = robotics::viewer::loadWorkspace(argv[1]);
    const auto document = robotics::scene_view::load(workspace.scene);
    robotics::visualization::LocalSource source(workspace.selected_source,
                                                robotics::viewer::loadRecording(argv[2]));
    source.seek(source.duration()); // Exercise source-bound state at the recording endpoint.
    const auto snapshot = source.snapshot();
    const auto scene = robotics::scene_view::resolve(document, workspace.fixed_frame, &snapshot);
    if (!scene.issues.empty() || scene.scene.instances.empty())
        return 1;
    std::cout << scene.scene.instances.size()
              << " installed mesh instances resolved without a window or simulation.\n";
}
