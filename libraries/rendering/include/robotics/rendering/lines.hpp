#pragma once
#include <robotics/visualization/display.hpp>

#include <memory>

namespace robotics::rendering {
// Current OpenGL 3.3 context and initialized GLEW required throughout lifetime,
// on the owning thread. Call draw between application passes: GL state is not restored.
// Draws to the caller's framebuffer; owns no window or simulation state.
class Lines {
  public:
    Lines();
    ~Lines();
    Lines(const Lines &) = delete;
    Lines &operator=(const Lines &) = delete;
    void draw(const std::vector<visualization::Line> &lines, const Eigen::Matrix4f &matrix);

  private:
    struct Resources;
    std::unique_ptr<Resources> resources_;
};
} // namespace robotics::rendering
