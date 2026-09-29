#pragma once
#include "desktop.hpp"
#include <robotics/viewer/session.hpp>

#include <array>

namespace robotics::viewer {
// Injected presentation only; it receives the same frozen source snapshot as overlays.
using SceneDraw = std::function<ViewportBackground(
    const Workspace &, const visualization::SourceSnapshot *, int, int, bool, std::string &)>;
class Interface {
  public:
    explicit Interface(const std::filesystem::path &workspace, SceneDraw scene_draw = {},
                       const std::filesystem::path &scene_override = {});
    Interface(Workspace workspace, Sources sources, visualization::Displays displays,
              SceneDraw scene_draw = {});
    void draw();
    void advance(visualization::Time elapsed_ns);
    void seek(visualization::Time time_ns);

  private:
    void controls();
    void playback();
    void view();
    Session session_;
    SceneDraw scene_draw_;
    std::string scene_message_;
    bool reload_scene_{true};
    Viewport viewport_;
    std::array<char, 4096> path_{};
    std::array<char, 257> frame_{};
    std::array<char, 32> time_{};
    bool editing_time_{false};
    std::string message_;
    bool playing_{false};
    double speed_{1};
};
} // namespace robotics::viewer
