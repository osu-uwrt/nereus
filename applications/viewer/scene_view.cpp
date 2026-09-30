#include "scene_view.hpp"
#include <robotics/rendering/renderer.hpp>
#include <robotics/scene_view/scene.hpp>

namespace robotics::viewer {
SceneDraw sceneDrawer(std::filesystem::path shaders) {
    if (shaders.empty())
        shaders = std::filesystem::canonical("/proc/self/exe").parent_path().parent_path() /
                  "share/nereus/shaders";
    struct State {
        std::unique_ptr<rendering::Renderer> renderer;
        scene_view::Document document;
        std::filesystem::path path;
        std::string failure;
    };
    auto state = std::make_shared<State>();
    return [state, shaders](const Workspace &workspace, const visualization::SourceSnapshot *source,
                            int width, int height, bool reload, std::string &message) {
        if (workspace.scene.empty()) {
            *state = State{};
            return ViewportBackground{};
        }
        if (reload || state->path != workspace.scene) {
            state->document = {};
            state->path = workspace.scene;
            state->failure.clear();
            try {
                state->document = scene_view::load(workspace.scene);
                if (!state->renderer)
                    state->renderer = std::make_unique<rendering::Renderer>(shaders);
            } catch (const std::exception &error) {
                state->failure = error.what();
            }
        }
        if (!state->failure.empty())
            throw std::runtime_error(state->failure);
        auto resolved = scene_view::resolve(state->document, workspace.fixed_frame, source);
        for (const auto &[id, issue] : resolved.issues)
            message += id + ": " + issue + "\n";
        const auto camera =
            visualization::cameraMatrices(workspace.camera, static_cast<double>(width) / height);
        // Static scenes freeze at t=0. Live/playback water follows the selected source clock.
        const float time = source && source->data
                               ? static_cast<float>(static_cast<double>(source->time_ns) / 1e9)
                               : 0.f;
        const auto frame = state->renderer->draw(
            resolved.scene, {camera.view, camera.projection, camera.eye}, {}, time, width, height);
        return ViewportBackground{frame.color_texture, frame.depth_texture};
    };
}
} // namespace robotics::viewer
