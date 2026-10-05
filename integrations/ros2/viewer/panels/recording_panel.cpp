#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "status_chip.hpp"
#include <cstdio>
#include <ctime>
#include <imgui.h>
namespace nereus::ros_viewer::panels {
std::string recordingFile(const std::string &base, const std::string &camera, const std::string &stamp,
                          const std::string &home) {
    std::string stem = base, extension = ".svo2";
    if (!stem.empty() && stem[0] == '~' && (stem.size() == 1 || stem[1] == '/'))
        stem = home + stem.substr(1);
    for (const char *known : {".svo2", ".svo"}) {
        const std::string suffix = known;
        if (stem.size() > suffix.size() && stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0) {
            extension = suffix;
            stem.resize(stem.size() - suffix.size());
            break;
        }
    }
    return stem + (stamp.empty() ? "" : "_" + stamp) + "_" + camera + extension;
}
namespace {
std::string clock(double seconds) {
    char text[32];
    const int total = int(seconds);
    std::snprintf(text, sizeof(text), "%02d:%02d", total / 60, total % 60);
    return text;
}
std::string localStamp() {
    char text[32];
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    std::strftime(text, sizeof(text), "%Y%m%d_%H%M%S", &local);
    return text;
}
// SVO recording per camera and still capture (the RViz mapping panel's recording and picture taker tools).
// The path is on the robot; header form: a red REC chip per camera while it records.
class RecordingPanel final : public Panel {
    std::shared_ptr<Recording> recording;
    char path[512]{};
    std::string home;
    bool timestamp = true;

  public:
    explicit RecordingPanel(const Binding &b)
        : recording(std::dynamic_pointer_cast<Recording>(b.provider)),
          home(b.options["home"].as<std::string>("/home/ros")), timestamp(b.options["timestamp"].as<bool>(true)) {
        std::snprintf(path, sizeof(path), "%s", b.options["path"].as<std::string>("~/svos/run").c_str());
    }
    void header() override {
        if (!recording)
            return;
        bool first = true;
        for (const auto &camera : recording->state().cameras)
            if (camera.recording) {
                if (!first)
                    ImGui::SameLine(0, 4);
                first = false;
                ImGui::PushID(camera.id.c_str());
                const auto text = "REC " + camera.label + " " + clock(camera.elapsed);
                if (statusChip(text.c_str(), levelColor(Level::Error)))
                    ImGui::SetTooltip("Recording to %s", camera.file.c_str());
                ImGui::PopID();
            }
    }
    void draw() override {
        auto s = recording ? recording->state() : RecordingState{};
        if (!recording)
            ImGui::TextDisabled("Preview / recording disconnected");
        ImGui::SeparatorText("SVO recording");
        ImGui::TextUnformatted("Path on the robot");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##path", "~/svos/run", path, sizeof(path));
        pins::Checkbox("Add date and time", &timestamp);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Inserts the start time so a new recording never overwrites an older one.");
        if (recording && !s.svoSupported)
            ImGui::TextWrapped("Built without zed_msgs: recordings can be stopped but not started.");
        const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) / 2;
        for (const auto &camera : s.cameras) {
            pins::Scope cameraScope(camera.id); // each camera's Start / Stop pins on its own
            ImGui::Spacing();
            ImGui::TextUnformatted(camera.label.c_str());
            ImGui::SameLine();
            if (camera.recording)
                ImGui::TextColored(levelColor(Level::Error), "REC %s", clock(camera.elapsed).c_str());
            else
                ImGui::TextDisabled("%s", camera.stopReady || camera.startReady ? "idle" : "service unavailable");
            const auto file = recordingFile(path, camera.id, timestamp ? localStamp() : "", home);
            ImGui::BeginDisabled(!s.svoSupported || !camera.startReady || camera.pending || camera.recording ||
                                 !path[0]);
            if (pins::Button(("Start " + camera.label + "###start").c_str(), {half, ui(30)}))
                recording->start(camera.id, file);
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!camera.stopReady || camera.pending);
            if (pins::Button(("Stop " + camera.label + "###stop").c_str(), {half, ui(30)}))
                recording->stop(camera.id);
            ImGui::EndDisabled();
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::TextWrapped("%s", camera.recording ? camera.file.c_str() : file.c_str());
            ImGui::PopStyleColor();
            if (!camera.message.empty())
                ImGui::TextWrapped("%s", camera.message.c_str());
        }
        ImGui::SeparatorText("Still images");
        ImGui::BeginDisabled(!recording || !s.captureReady || s.capturing);
        if (pins::Button(s.capturing ? "Capturing...###capture" : "Capture image###capture", {-1, ui(36)}))
            recording->capture();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("The picture taker saves the newest camera frame on the robot.");
        if (recording && !s.captureReady && !s.capturing)
            ImGui::TextDisabled("Capture service unavailable");
        if (!s.captureMessage.empty())
            ImGui::TextWrapped("%s", s.captureMessage.c_str());
    }
};
} // namespace
void registerRecordingPanel(Registry &r) {
    r.panels.emplace("recording",
                     ViewFactory<Panel>{Kind::Recording,
                                        [](const YAML::Node &n) {
                                            keys(n, {"path", "home", "timestamp"}, "recording panel");
                                            (void)n["path"].as<std::string>("");
                                            (void)n["home"].as<std::string>("");
                                            (void)n["timestamp"].as<bool>(true);
                                        },
                                        [](const Binding &b) { return std::make_unique<RecordingPanel>(b); }});
}
} // namespace nereus::ros_viewer::panels
