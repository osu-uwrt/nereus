#pragma once
#include <robotics/visualization/source.hpp>

#include <array>
#include <functional>

namespace robotics::visualization {
using Color = std::array<int, 3>;
struct Line {
    Eigen::Vector3d from;
    Eigen::Vector3d to;
    Color color{180, 200, 220};
};
enum class Level { ready, warning, error };
struct DisplayResult {
    std::vector<Line> lines;
    Level level{Level::ready};
    std::string status{"Ready"};
};
struct DisplaySettings {
    std::string id;
    std::string type;
    std::string source;
    std::string stream;
    std::string frame;
    bool enabled{true};
    std::size_t history_limit{1000};
    Time max_age_ns{1000000000};
    double scale{0.5};
    Color color{67, 210, 195};
};
struct DisplayContext {
    const SourceSnapshot *source;
    const std::string &fixed_frame;
    const DisplaySettings &settings;
};
// Explicitly constructed registry. Extensions register ordinary functions without inheritance.
using Display = std::function<DisplayResult(const DisplayContext &)>;
class Displays {
  public:
    void add(std::string name, Display display);
    DisplayResult draw(const DisplayContext &context) const;

  private:
    std::map<std::string, Display> displays_;
};
Displays standardDisplays();
void validate(const DisplaySettings &settings);
} // namespace robotics::visualization
