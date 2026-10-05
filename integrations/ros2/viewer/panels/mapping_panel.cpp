#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <cstdio>
#include <imgui.h>
namespace nereus::ros_viewer::panels {
namespace {
class MappingPanel final : public Panel {
    std::shared_ptr<Mapping> mapping;
    char parent[256]{}, child[256]{}, target[256]{};
    int samples = 10;
    bool locked = false, initialized = false;

  public:
    explicit MappingPanel(const Binding &b) : mapping(std::dynamic_pointer_cast<Mapping>(b.provider)) {
        std::snprintf(parent, sizeof(parent), "%s", b.options["parent_frame"].as<std::string>("").c_str());
        std::snprintf(child, sizeof(child), "%s", b.options["tag_frame"].as<std::string>("").c_str());
        samples = b.options["samples"].as<int>(10);
    }
    void draw() override {
        auto s = mapping ? mapping->state() : MappingState{};
        sectionTitle("Tag calibration");
        ImGui::BeginDisabled(s.calibrating);
        ImGui::TextUnformatted("Parent frame");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##parent", parent, sizeof(parent));
        ImGui::TextUnformatted("Tag frame");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##child", child, sizeof(child));
        ImGui::TextUnformatted("Samples");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputInt("##samples", &samples);
        ImGui::EndDisabled();
        ImGui::BeginDisabled(!mapping || (!s.calibrating && (!s.calibrationReady || samples < 1 || samples > 65535 ||
                                                             !parent[0] || !child[0] || std::string(parent) == child)));
        if (pins::Button(s.calibrating ? "Cancel calibration###tag_cal" : "Calibrate tag###tag_cal", {-1, ui(36)})) {
            if (s.calibrating)
                mapping->cancelCalibration();
            else
                mapping->calibrate(parent, child, samples);
        }
        ImGui::EndDisabled();
        if (s.calibrating)
            ImGui::Text("Samples: %u / %d", s.samples, samples);
        if (mapping && !s.calibrationMessage.empty())
            ImGui::TextWrapped("%s", s.calibrationMessage.c_str());
        sectionTitle("Mapping target");
        if (s.fresh) {
            ImGui::TextWrapped("Current: %s", s.target.empty() ? "Automatic" : s.target.c_str());
            ImGui::TextUnformatted(s.locked ? "Map locked" : "Map unlocked");
            if (!initialized) {
                std::snprintf(target, sizeof(target), "%s", s.target.c_str());
                locked = s.locked;
                initialized = true;
            }
        } else
            ImGui::TextDisabled("Mapping status unavailable / stale");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##target", "Target object (empty = automatic)", target, sizeof(target));
        pins::Checkbox("Lock map", &locked);
        ImGui::BeginDisabled(!mapping || !s.targetReady || s.settingTarget);
        if (pins::Button(s.settingTarget ? "Setting target...###set_target" : "Set mapping target###set_target",
                         {-1, ui(36)}))
            mapping->setTarget(target, locked);
        ImGui::EndDisabled();
        if (!s.targetMessage.empty())
            ImGui::TextWrapped("%s", s.targetMessage.c_str());
        ImGui::BeginDisabled(!mapping || !s.resetReady || s.resetting);
        if (pins::Button(s.resetting ? "Resetting...###reset_mapping" : "Reset mapping###reset_mapping", {-1, ui(36)}))
            mapping->reset();
        ImGui::EndDisabled();
        if (!s.resetMessage.empty())
            ImGui::TextWrapped("%s", s.resetMessage.c_str());
    }
};
} // namespace
void registerMappingPanel(Registry &r) {
    r.panels.emplace("mapping", ViewFactory<Panel>{Kind::Mapping,
                                                   [](const YAML::Node &n) {
                                                       keys(n, {"parent_frame", "tag_frame", "samples"},
                                                            "mapping panel");
                                                       positive(n, "samples", 10, 65535);
                                                       (void)n["samples"].as<int>(10);
                                                   },
                                                   [](const Binding &b) { return std::make_unique<MappingPanel>(b); }});
}
} // namespace nereus::ros_viewer::panels
