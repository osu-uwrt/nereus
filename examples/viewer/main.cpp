#include <robotics/viewer/session.hpp>

#include <iostream>
#include <stdexcept>
#include <utility>

namespace v = robotics::visualization;
namespace ui = robotics::viewer;
// An ordinary live-style source: no playback capability and no simulation dependency.
class ExampleSource final : public v::Source {
  public:
    explicit ExampleSource(std::string id) : id_(std::move(id)) {
        v::SourceData data;
        data.clock = "example-device";
        data.frames = std::make_shared<const v::FrameGraph>("world", std::vector<v::FrameEdge>{});
        data.streams["pose"] = {{42, "world", {{1, 2, 3}, Eigen::Quaterniond::Identity()}}};
        data_ = std::make_shared<const v::SourceData>(std::move(data));
    }
    v::SourceSnapshot snapshot() const override {
        return {id_, 0, 42, data_};
    }
    void disconnect() override {
        data_.reset();
    }

  private:
    std::string id_;
    std::shared_ptr<const v::SourceData> data_;
};
int main() {
    ui::Sources sources{{"example", [](const std::filesystem::path &, const std::string &id) {
                             return ui::Connection{std::make_shared<ExampleSource>(id), {}, 0, {}};
                         }}};
    auto displays = v::standardDisplays();
    displays.add("origin-mark", [](const v::DisplayContext &) {
        return v::DisplayResult{
            {{{0, 0, 0}, {0, 0, 2}, {255, 220, 80}}}, v::Level::ready, "Example display"};
    });
    ui::Session session(std::move(sources), std::move(displays));
    auto workspace = ui::emptyWorkspace();
    workspace.sources = {{"device", "example", {}}};
    workspace.selected_source = "device";
    v::DisplaySettings pose;
    pose.id = "Device pose";
    pose.type = "pose";
    pose.source = "device";
    pose.stream = "pose";
    workspace.displays.push_back(pose);
    v::DisplaySettings mark;
    mark.id = "Mark";
    mark.type = "origin-mark";
    workspace.displays.push_back(mark);
    session.open(workspace);
    const auto scene = session.scene();
    if (session.canSeek() || scene.status.at("Device pose").first != v::Level::ready ||
        scene.lines.size() != 61)
        throw std::runtime_error("extension contract failed");
    session.disconnect();
    if (session.scene().status.at("Device pose").first != v::Level::warning)
        throw std::runtime_error("source removal did not invalidate the display");
    std::cout << "External source and display composed without simulation or a graphics context.\n";
}
