// Bagging panel: start/stop `ros2 bag record` on this computer or the robot, choose the bag folder, name and
// topics, and show a BAG chip per recording machine in the header row.
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "status_chip.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <imgui.h>
#include <imgui_internal.h>
#include <map>
#include <set>

namespace nereus::ros_viewer::panels {
namespace {

// Elapsed time as MM:SS, or H:MM:SS from an hour on.
std::string clock(double seconds) {
    char text[32];
    const int total = std::max(0, int(seconds));
    if (total >= 3600)
        std::snprintf(text, sizeof(text), "%d:%02d:%02d", total / 3600, total / 60 % 60, total % 60);
    else
        std::snprintf(text, sizeof(text), "%02d:%02d", total / 60, total % 60);
    return text;
}

// Where a bag is, for scp: host:path on another machine.
std::string location(const BagTargetState &t, const std::string &path) {
    return t.host.empty() ? path : t.host + ":" + path;
}

// Wrapped text in the theme's muted colour.
void muted(const std::string &text) {
    ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
    ImGui::TextWrapped("%s", text.c_str());
    ImGui::PopStyleColor();
}

// ros2 bag record on this computer or the robot (the RViz rosbag panel's recorder, with a choice of machine).
// Header form: a red BAG chip per machine while it records.
class BaggingPanel final : public Panel {
    std::shared_ptr<Bagging> bagging;
    int target = 0, mode = 0;         // mode 0: all topics, 1: the selected ones
    char name[128]{}, exclude[256]{}; // bag name prefix and the -x regex (all-topics mode only)
    bool timestamp = true;
    std::map<std::string, std::array<char, 512>> directories; // per target, from its configured directory
    std::map<std::string, std::array<char, 256>> hosts;       // per remote target: the ssh destination being edited
    std::set<std::string> selected;                           // topics recorded in "Selected" mode
    ImGuiTextFilter filter;                                   // topic list filter

  public:
    explicit BaggingPanel(const Binding &b)
        : bagging(std::dynamic_pointer_cast<Bagging>(b.provider)), mode(b.options["all"].as<bool>(true) ? 0 : 1),
          timestamp(b.options["timestamp"].as<bool>(true)) {
        std::snprintf(name, sizeof(name), "%s", b.options["name"].as<std::string>("bag").c_str());
        std::snprintf(exclude, sizeof(exclude), "%s", b.options["exclude"].as<std::string>("").c_str());

        // Preselect the configured `target` by its ID.
        if (bagging && b.options["target"]) {
            const auto s = bagging->state();
            for (size_t i = 0; i < s.targets.size(); ++i)
                if (s.targets[i].id == b.options["target"].as<std::string>())
                    target = int(i);
        }
    }

    // One chip per machine that is recording (red, with elapsed time) or closing its bag (amber).
    void header() override {
        if (!bagging)
            return;
        bool first = true;
        for (const auto &t : bagging->state().targets)
            if (t.recording || t.stopping) {
                if (!first)
                    ImGui::SameLine(0, ui(6));
                first = false;
                ImGui::PushID(t.id.c_str());
                const auto text = "BAG " + t.label + " " + (t.recording ? clock(t.elapsed) : std::string("closing"));
                if (statusChip(text.c_str(), levelColor(t.recording ? Level::Error : Level::Warn)))
                    ImGui::SetTooltip("%s\n%s", location(t, t.bag).c_str(), byteSize(t.bytes).c_str());
                ImGui::PopID();
            }
    }

    void draw() override {
        const auto s = bagging ? bagging->state() : BaggingState{};
        if (s.targets.empty()) {
            emptyState("Connected, this records ROS bags on this computer or on the robot.");
            return;
        }

        // Machine choice, its ssh destination and the free disk space (amber below 5 GB).
        target = std::clamp(target, 0, int(s.targets.size()) - 1);
        chooseTarget(s);
        const auto &t = s.targets[size_t(target)];
        auto &directory = directoryOf(t);
        const bool busy = t.recording || t.stopping || t.pending;
        if (!t.host.empty())
            hostField(t, busy);
        std::string where = t.host.empty() ? "This computer" : "";
        if (t.freeBytes >= 0)
            where += (where.empty() ? "" : "  ·  ") + byteSize(t.freeBytes) + " free";
        if (t.freeBytes >= 0 && t.freeBytes < 5e9)
            ImGui::TextColored(palette().warn, "%s", where.c_str());
        else if (!where.empty())
            muted(where);

        // Record / Stop first, then the bag settings (locked while this machine records).
        const std::string bagNameNow = bagName(name, timestamp);
        const bool nameOk = validBagName(bagNameNow);
        std::vector<std::string> topics(selected.begin(), selected.end());
        const bool ready = t.known && t.reachable && !busy && nameOk && directory[0] && (mode == 0 || !topics.empty());
        actions(t, ready, bagNameNow, directory.data(), topics);

        ImGui::BeginDisabled(busy);
        sectionTitle("Bag");
        ImGui::TextUnformatted(t.host.empty() ? "Folder" : "Folder on the robot");
        ImGui::SetNextItemWidth(-1);
        ImGui::PushID(t.id.c_str());
        ImGui::InputTextWithHint("##folder", t.directory.c_str(), directory.data(), directory.size());
        ImGui::PopID();
        ImGui::TextUnformatted("Name");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##name", "bag", name, sizeof(name));
        pins::Checkbox("Add date and time", &timestamp);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Appends the start time so a new bag never collides with an older one.");
        if (nameOk)
            muted(std::string(directory.data()) + "/" + bagNameNow);
        else
            ImGui::TextColored(levelColor(Level::Error), "Names use letters, digits, '.', '_' and '-' only.");

        sectionTitle("Topics");
        pins::Switch("Topics##mode", &mode, {"All", "Selected"}, ImGui::GetContentRegionAvail().x);
        if (mode == 0) {
            ImGui::TextUnformatted("Leave out (regex)");
            ImGui::SetNextItemWidth(-1);
            ImGui::InputTextWithHint("##exclude", "e.g. image|point_cloud", exclude, sizeof(exclude));
            muted("Every topic, including ones that appear while recording.");
        } else
            topicList(s);
        ImGui::EndDisabled();
    }

  private:
    // Record / Stop (each machine's pins on its own), the recording's progress, the outcome and the last bag.
    void actions(const BagTargetState &t, bool ready, const std::string &bagNameNow, const char *directory,
                 const std::vector<std::string> &topics) {
        pins::Scope targetScope(t.id);

        // Record while idle, Stop while recording (disabled once a stop is under way).
        if (!t.recording && !t.stopping) {
            ImGui::BeginDisabled(!ready);
            if (pins::Button(("Record on " + t.label + "###record").c_str(), {-1, ui(36)})) {
                BagRequest request;
                request.name = bagNameNow;
                request.directory = directory;
                request.all = mode == 0;
                request.topics = topics;
                request.exclude = mode == 0 ? exclude : "";
                bagging->start(t.id, request);
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("%s", t.host.empty() ? "Records on this computer; it stops when the viewer closes."
                                                       : "Records on the robot; it keeps going if the viewer closes "
                                                         "or the link drops, and any viewer can stop it.");
        } else {
            ImGui::BeginDisabled(t.pending || t.stopping);
            if (pins::Button(("Stop on " + t.label + "###stop").c_str(), {-1, ui(36)}))
                bagging->stop(t.id);
            ImGui::EndDisabled();
        }

        // Progress of the running recording; a recorder stuck closing can be killed.
        if (t.recording || t.stopping) {
            if (t.recording)
                ImGui::TextColored(levelColor(Level::Error), "REC %s", clock(t.elapsed).c_str());
            else
                ImGui::TextColored(palette().warn, "Closing the bag...");
            ImGui::SameLine();
            ImGui::TextUnformatted(byteSize(t.bytes).c_str());
            muted(location(t, t.bag));
            if (t.stopping && !t.pending) {
                if (pins::Button("Kill recorder###kill"))
                    bagging->kill(t.id);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("For a recorder that never finishes: the bag keeps what it wrote but loses its "
                                      "metadata.yaml (ros2 bag reindex rebuilds it).");
            }
        }

        // Provider message (red on failure) and the last finished bag.
        if (!t.message.empty()) {
            if (t.failed)
                ImGui::PushStyleColor(ImGuiCol_Text, levelColor(Level::Error));
            ImGui::TextWrapped("%s", t.message.c_str());
            if (t.failed)
                ImGui::PopStyleColor();
        }
        if (!t.recording && !t.stopping && !t.lastBag.empty()) {
            ImGui::Spacing();
            muted("Last bag: " + location(t, t.lastBag) + " (" + byteSize(t.lastBytes) + ")");
            if (ImGui::SmallButton("Copy path"))
                ImGui::SetClipboardText(location(t, t.lastBag).c_str());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", t.host.empty() ? "The bag's folder on this computer."
                                                       : "host:path, for scp -r or rsync.");
        }
    }

    // Machine picker: a switch for two or three targets, a combo for more (nothing for one).
    void chooseTarget(const BaggingState &s) {
        std::vector<const char *> labels;
        for (const auto &t : s.targets)
            labels.push_back(t.label.c_str());
        const float width = ImGui::GetContentRegionAvail().x;
        if (labels.size() == 2)
            pins::Switch("Record on##target", &target, {labels[0], labels[1]}, width);
        else if (labels.size() == 3)
            pins::Switch("Record on##target", &target, {labels[0], labels[1], labels[2]}, width);
        else if (labels.size() > 3) {
            std::string items;
            for (const auto *label : labels)
                items += std::string(label) + '\0';
            ImGui::SetNextItemWidth(-1);
            pins::Combo("Record on##target", &target, items.c_str());
        }
    }

    // The robot's ssh destination (user@host), prefilled from the configuration; Enter or leaving the field connects
    // to it. Locked while a bag records there.
    void hostField(const BagTargetState &t, bool busy) {
        auto [found, inserted] = hosts.try_emplace(t.id);
        auto &buffer = found->second;
        ImGui::PushID(t.id.c_str());

        // Follow the provider's host unless the operator is typing in the field.
        const bool editing = ImGui::GetActiveID() == ImGui::GetID("##host");
        if (inserted || (!editing && t.host != buffer.data()))
            std::snprintf(buffer.data(), buffer.size(), "%s", t.host.c_str());
        ImGui::BeginDisabled(busy);
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("ssh");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-1);
        const bool entered =
            ImGui::InputTextWithHint("##host", "user@host", buffer.data(), buffer.size(),
                                     ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsNoBlank);
        if ((entered || ImGui::IsItemDeactivatedAfterEdit()) && buffer[0])
            bagging->setHost(t.id, buffer.data());
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("%s",
                              busy ? "Stop the bag to change machines."
                                   : "Where to record: as you would type `ssh <this>` (a key login, no password).");
        ImGui::EndDisabled();
        ImGui::PopID();
    }

    // The editable folder buffer for a target, seeded once from its configured directory.
    std::array<char, 512> &directoryOf(const BagTargetState &t) {
        auto [found, inserted] = directories.try_emplace(t.id);
        if (inserted)
            std::snprintf(found->second.data(), found->second.size(), "%s", t.directory.c_str());
        return found->second;
    }

    // Topic picker for "Selected" mode: presets, a filter, bulk select/clear and a clipped checkbox list.
    void topicList(const BaggingState &s) {
        // A preset replaces the selection.
        if (!s.presets.empty()) {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##preset", "Preset...")) {
                for (const auto &preset : s.presets)
                    if (ImGui::Selectable(preset.name.c_str())) {
                        selected = {preset.topics.begin(), preset.topics.end()};
                        filter.Clear();
                    }
                ImGui::EndCombo();
            }
        }

        // Live topics plus selected ones not published now (they record once they appear).
        std::vector<std::pair<std::string, std::string>> rows = s.topics;
        for (const auto &topic : selected)
            if (!std::binary_search(rows.begin(), rows.end(), std::make_pair(topic, std::string()),
                                    [](const auto &a, const auto &b) { return a.first < b.first; }))
                rows.emplace_back(topic, "");
        std::sort(rows.begin(), rows.end());
        std::vector<const std::pair<std::string, std::string> *> shown;
        for (const auto &row : rows)
            if (filter.PassFilter(row.first.c_str()))
                shown.push_back(&row);

        const float buttons = buttonWidth("Select shown") + buttonWidth("Clear") + 2 * ImGui::GetStyle().ItemSpacing.x;
        ImGui::SetNextItemWidth(std::max(ui(80), ImGui::GetContentRegionAvail().x - buttons));
        if (ImGui::InputTextWithHint("##filter", "Filter topics", filter.InputBuf, IM_ARRAYSIZE(filter.InputBuf)))
            filter.Build();
        ImGui::SameLine();
        if (ImGui::Button("Select shown"))
            for (const auto *row : shown)
                selected.insert(row->first);
        ImGui::SameLine();
        if (ImGui::Button("Clear"))
            selected.clear();
        char count[64];
        std::snprintf(count, sizeof(count), "%zu selected of %zu", selected.size(), rows.size());
        muted(count);

        // Only the visible rows are drawn (ImGuiListClipper).
        ImGui::BeginChild("topics", {0, std::max(ui(120), ImGui::GetContentRegionAvail().y)}, ImGuiChildFlags_Borders);
        ImGuiListClipper clipper;
        clipper.Begin(int(shown.size()));
        while (clipper.Step())
            for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; ++i) {
                const auto &[topic, type] = *shown[size_t(i)];
                bool on = selected.count(topic) != 0;
                ImGui::PushID(topic.c_str());
                if (ImGui::Checkbox(topic.c_str(), &on)) {
                    if (on)
                        selected.insert(topic);
                    else
                        selected.erase(topic);
                }
                ImGui::SameLine();
                ImGui::TextDisabled("%s", type.empty() ? "not published" : type.c_str());
                ImGui::PopID();
            }
        if (shown.empty())
            ImGui::TextDisabled(rows.empty() ? "No topics seen yet." : "No topic matches the filter.");
        ImGui::EndChild();
    }
};

} // namespace

// Registers the "bagging" panel type; options: name, timestamp, all, exclude, target.
void registerBaggingPanel(Registry &r) {
    r.panels.emplace("bagging",
                     ViewFactory<Panel>{Kind::Bagging,
                                        [](const YAML::Node &n) {
                                            keys(n, {"name", "timestamp", "all", "exclude", "target"}, "bagging panel");
                                            const auto name = n["name"].as<std::string>("bag");
                                            if (!name.empty() && !validBagName(name))
                                                throw std::invalid_argument("bag name: letters, digits, . _ - only");
                                            (void)n["timestamp"].as<bool>(true);
                                            (void)n["all"].as<bool>(true);
                                            (void)n["exclude"].as<std::string>("");
                                            (void)n["target"].as<std::string>("");
                                        },
                                        [](const Binding &b) { return std::make_unique<BaggingPanel>(b); }});
}

} // namespace nereus::ros_viewer::panels
