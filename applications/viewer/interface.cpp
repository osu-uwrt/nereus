#include "interface.hpp"

#include <imgui.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace robotics::viewer {
namespace v = visualization;
Interface::Interface(const std::filesystem::path &workspace)
    : session_(localSources(), v::standardDisplays()) {
    if (!workspace.empty()) {
        session_.open(loadWorkspace(workspace));
        std::snprintf(path_.data(), path_.size(), "%s", workspace.c_str());
    }
    std::snprintf(frame_.data(), frame_.size(), "%s", session_.workspace().fixed_frame.c_str());
}
void Interface::seek(v::Time time_ns) {
    session_.seek(time_ns);
}
void Interface::advance(v::Time elapsed_ns) {
    if (!playing_)
        return;
    session_.advance(static_cast<v::Time>(
        static_cast<double>(std::min<v::Time>(elapsed_ns, 250000000)) * speed_));
    const auto source = session_.snapshot();
    if (!session_.canSeek() || (source && source->time_ns == session_.duration()))
        playing_ = false;
}
void Interface::controls() {
    auto &workspace = session_.workspace();
    ImGui::TextUnformatted("SOURCES");
    if (ImGui::BeginCombo("##source", workspace.selected_source.empty()
                                          ? "No source"
                                          : workspace.selected_source.c_str())) {
        for (const auto &source : workspace.sources)
            if (ImGui::Selectable(source.id.c_str(), source.id == workspace.selected_source)) {
                workspace.selected_source = source.id;
                playing_ = false;
            }
        ImGui::EndCombo();
    }
    const auto snapshot = session_.snapshot();
    if (snapshot && snapshot->data) {
        ImGui::TextWrapped("Clock: %s", snapshot->data->clock.c_str());
        ImGui::Text("Generation: %llu", static_cast<unsigned long long>(snapshot->generation));
        const auto &delivery = snapshot->delivery;
        if (delivery.dropped_queue || delivery.trimmed_history || delivery.rejected_stale)
            ImGui::TextWrapped(
                "Display delivery: %llu dropped, %llu history trimmed, %llu stale rejected",
                static_cast<unsigned long long>(delivery.dropped_queue),
                static_cast<unsigned long long>(delivery.trimmed_history),
                static_cast<unsigned long long>(delivery.rejected_stale));
        if (ImGui::Button("Disconnect")) {
            session_.disconnect();
            playing_ = false;
        }
    } else if (snapshot) {
        ImGui::TextDisabled("No active source data");
        if (ImGui::Button("Reconnect"))
            session_.reconnect();
    } else {
        ImGui::TextWrapped("Open a workspace to inspect local data.");
    }
    for (const auto &[id, error] : session_.errors())
        ImGui::TextWrapped("%s: %s", id.c_str(), error.c_str());
    ImGui::Separator();
    ImGui::TextUnformatted("FIXED FRAME");
    if (ImGui::InputText("##frame", frame_.data(), frame_.size(),
                         ImGuiInputTextFlags_EnterReturnsTrue))
        if (frame_[0] != '\0')
            workspace.fixed_frame = frame_.data();
    if (snapshot && snapshot->data) {
        if (ImGui::BeginCombo("Choose frame", workspace.fixed_frame.c_str())) {
            for (const auto &frame : snapshot->data->frames->frames())
                if (ImGui::Selectable(frame.c_str(), frame == workspace.fixed_frame)) {
                    workspace.fixed_frame = frame;
                    std::snprintf(frame_.data(), frame_.size(), "%s", frame.c_str());
                }
            ImGui::EndCombo();
        }
    }
    ImGui::TextDisabled("Metres / right-handed / Z up");
    ImGui::Separator();
    ImGui::TextUnformatted("DISPLAYS");
    const auto scene = session_.scene();
    for (auto &display : workspace.displays) {
        if (!display.source.empty() && display.source != workspace.selected_source)
            continue;
        ImGui::PushID(display.id.c_str());
        ImGui::Checkbox(display.id.c_str(), &display.enabled);
        const auto status = scene.status.find(display.id);
        if (status != scene.status.end()) {
            const auto color = status->second.first == v::Level::ready
                                   ? ImVec4(0.5F, 0.7F, 0.75F, 1)
                                   : ImVec4(1, 0.7F, 0.35F, 1);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", status->second.second.c_str());
            ImGui::PopStyleColor();
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    if (ImGui::Button("Reset camera"))
        workspace.camera = Camera{};
    ImGui::TextWrapped("Drag to orbit. Right-drag to pan. Scroll to zoom. Arrow keys orbit the "
                       "focused view; +/- zoom.");
    ImGui::TextDisabled("Axes: X red / Y green / Z blue");
}
void Interface::playback() {
    const auto source = session_.snapshot();
    ImGui::BeginDisabled(!session_.canSeek());
    if (ImGui::Button(playing_ ? "Pause" : "Play"))
        playing_ = !playing_;
    ImGui::SameLine();
    if (ImGui::Button("Rewind")) {
        session_.seek(0);
        playing_ = false;
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    const char *speeds[] = {"0.25x", "1x", "2x", "4x"};
    const double values[] = {0.25, 1, 2, 4};
    int selected = 1;
    for (int i = 0; i < 4; ++i)
        if (speed_ == values[i])
            selected = i;
    if (ImGui::Combo("Speed", &selected, speeds, 4))
        speed_ = values[selected];
    const v::Time current = source ? source->time_ns : 0;
    const auto duration = session_.duration();
    double fraction =
        duration > 0 ? static_cast<double>(current) / static_cast<double>(duration) : 0;
    const double minimum = 0, maximum = 1;
    ImGui::SetNextItemWidth(-1);
    if (ImGui::SliderScalar("##timeline", ImGuiDataType_Double, &fraction, &minimum, &maximum,
                            "")) {
        const long double scaled =
            static_cast<long double>(fraction) * static_cast<long double>(duration);
        session_.seek(scaled >= duration ? duration : static_cast<v::Time>(scaled));
        playing_ = false;
    }
    if (!editing_time_)
        std::snprintf(time_.data(), time_.size(), "%lld", static_cast<long long>(current));
    ImGui::SetNextItemWidth(200);
    if (ImGui::InputText("Time (ns)", time_.data(), time_.size(),
                         ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsDecimal)) {
        v::Time exact = 0;
        const char *end = time_.data() + std::strlen(time_.data());
        const auto parsed = std::from_chars(time_.data(), end, exact);
        if (parsed.ec == std::errc{} && parsed.ptr == end && exact >= 0 && exact <= duration) {
            session_.seek(exact);
            playing_ = false;
            message_.clear();
        } else {
            message_ = "Time must be integer nanoseconds within the selected recording.";
        }
    }
    editing_time_ = ImGui::IsItemActive();
    ImGui::SameLine();
    ImGui::Text("/ %.3f s", static_cast<double>(duration) / 1e9);
    ImGui::EndDisabled();
}
void Interface::view() {
    auto &camera = session_.workspace().camera;
    const auto available = ImGui::GetContentRegionAvail();
    const ImVec2 size(std::max(1.0F, available.x), std::max(1.0F, available.y - 135));
    const auto scene = session_.scene();
    const auto texture = viewport_.render(scene.lines, v::viewProjection(camera, size.x / size.y),
                                          static_cast<int>(size.x), static_cast<int>(size.y));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
    ImGui::ImageButton("Viewport", static_cast<ImTextureID>(texture), size, {0, 1}, {1, 0});
    ImGui::PopStyleVar();
    const bool viewFocused = ImGui::IsItemFocused();
    if (ImGui::IsItemHovered()) {
        auto &io = ImGui::GetIO();
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            camera.yaw -= static_cast<double>(io.MouseDelta.x) * 0.008;
            camera.pitch =
                std::clamp(camera.pitch + static_cast<double>(io.MouseDelta.y) * 0.008, -1.5, 1.5);
        }
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            const Eigen::Vector3d right(-std::sin(camera.yaw), std::cos(camera.yaw), 0);
            const Eigen::Vector3d up(-std::sin(camera.pitch) * std::cos(camera.yaw),
                                     -std::sin(camera.pitch) * std::sin(camera.yaw),
                                     std::cos(camera.pitch));
            camera.target += camera.distance * 0.002 *
                             (-static_cast<double>(io.MouseDelta.x) * right +
                              static_cast<double>(io.MouseDelta.y) * up);
        }
        camera.distance = std::clamp(
            camera.distance * std::exp(-static_cast<double>(io.MouseWheel) * 0.12), 0.1, 10000.0);
    }
    if (viewFocused && !ImGui::GetIO().WantTextInput) {
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))
            camera.yaw += 0.08;
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow))
            camera.yaw -= 0.08;
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow))
            camera.pitch = std::min(1.5, camera.pitch + 0.08);
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow))
            camera.pitch = std::max(-1.5, camera.pitch - 0.08);
        if (ImGui::IsKeyPressed(ImGuiKey_Equal))
            camera.distance = std::max(0.1, camera.distance / 1.1);
        if (ImGui::IsKeyPressed(ImGuiKey_Minus))
            camera.distance = std::min(10000.0, camera.distance * 1.1);
    }
    playback();
}
void Interface::draw() {
    const auto *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Robotics Viewer", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::TextUnformatted("ROBOTICS VIEWER");
    ImGui::SameLine();
    ImGui::TextDisabled("Local data workspace");
    ImGui::SetNextItemWidth(std::max(200.0F, ImGui::GetContentRegionAvail().x - 235));
    ImGui::InputText("##workspace", path_.data(), path_.size());
    ImGui::SameLine();
    try {
        if (ImGui::Button("Open")) {
            auto loaded = loadWorkspace(path_.data());
            session_.open(std::move(loaded));
            playing_ = false;
            std::snprintf(frame_.data(), frame_.size(), "%s",
                          session_.workspace().fixed_frame.c_str());
            message_ = "Workspace opened";
        }
        ImGui::SameLine();
        if (ImGui::Button("Save as")) {
            saveWorkspace(session_.workspace(), path_.data());
            message_ = "Workspace saved";
        }
        ImGui::SameLine();
        if (ImGui::Button("Empty")) {
            session_.open(emptyWorkspace());
            playing_ = false;
            message_.clear();
            std::snprintf(frame_.data(), frame_.size(), "%s",
                          session_.workspace().fixed_frame.c_str());
        }
    } catch (const std::exception &error) {
        message_ = error.what();
    }
    if (!message_.empty())
        ImGui::TextWrapped("%s", message_.c_str());
    ImGui::Separator();
    ImGui::BeginChild("Properties", {300, 0}, ImGuiChildFlags_Borders);
    controls();
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("Scene", {0, 0}, ImGuiChildFlags_Borders);
    view();
    ImGui::EndChild();
    ImGui::End();
}
} // namespace robotics::viewer
