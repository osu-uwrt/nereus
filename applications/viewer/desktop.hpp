#pragma once
#include <robotics/rendering/lines.hpp>

#include <filesystem>

struct GLFWwindow;
namespace robotics::viewer {
class Desktop {
  public:
    explicit Desktop(bool hidden);
    ~Desktop();
    Desktop(const Desktop &) = delete;
    Desktop &operator=(const Desktop &) = delete;
    bool closing() const;
    void begin();
    void finish(const std::filesystem::path &screenshot = {});

  private:
    void shutdown();
    GLFWwindow *window_{nullptr};
    bool glfw_ready_{false}, imgui_ready_{false}, platform_ready_{false}, renderer_ready_{false};
};
struct ViewportBackground {
    unsigned int color = 0, depth = 0;
};
class Viewport {
  public:
    Viewport();
    ~Viewport();
    Viewport(const Viewport &) = delete;
    Viewport &operator=(const Viewport &) = delete;
    unsigned int render(const std::vector<visualization::Line> &lines,
                        const Eigen::Matrix4f &matrix, int width, int height,
                        ViewportBackground background = {});

  private:
    rendering::Lines lines_;
    unsigned int framebuffer_{0}, read_framebuffer_{0}, texture_{0}, depth_{0};
    int width_{0}, height_{0};
};
} // namespace robotics::viewer
