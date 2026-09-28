#pragma once
#include "desktop.hpp"
#include <robotics/viewer/session.hpp>

#include <array>

namespace robotics::viewer {
class Interface {
  public:
    explicit Interface(const std::filesystem::path &workspace);
    void draw();
    void advance(visualization::Time elapsed_ns);
    void seek(visualization::Time time_ns);

  private:
    void controls();
    void playback();
    void view();
    Session session_;
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
