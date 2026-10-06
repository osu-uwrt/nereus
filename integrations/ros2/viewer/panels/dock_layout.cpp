// Operator window layout: the built-in dock presets and their DockBuilder construction, side-width queries on the
// live dock tree, the [Nereus][Windows] ini handler for open/closed states, and saved-layout file helpers.
#include "nereus/ros_viewer/dock_layout.hpp"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <imgui_internal.h>

namespace nereus::ros_viewer {

// The three built-in presets (Ctrl+Shift+1..3); unset fractions keep LayoutPreset's defaults.
const std::vector<LayoutPreset> &layoutPresets() {
    static const std::vector<LayoutPreset> presets = [] {
        LayoutPreset standard;
        standard.id = "standard";
        standard.label = "Standard";
        standard.shortcut = "Ctrl+Shift+1";
        standard.description = "Panels on the left, camera feeds and the course map on the right.";

        LayoutPreset wide;
        wide.id = "wide";
        wide.label = "Wide view";
        wide.shortcut = "Ctrl+Shift+2";
        wide.description = "A wide pool view, with the camera feeds and course map in a strip below it.";
        wide.left = .22f;
        wide.bottom = .3f;
        wide.remap = {{Dock::Right, Dock::Bottom}, {Dock::RightBottom, Dock::Bottom}};

        LayoutPreset cameras;
        cameras.id = "cameras";
        cameras.label = "Camera wall";
        cameras.shortcut = "Ctrl+Shift+3";
        cameras.description = "Large camera feeds on the right, the course map and its tabs under them.";
        cameras.left = .22f;
        cameras.right = .42f;
        cameras.rightBottom = .26f;

        return std::vector<LayoutPreset>{standard, wide, cameras};
    }();
    return presets;
}

const LayoutPreset *findPreset(const std::string &id) {
    for (const auto &preset : layoutPresets())
        if (preset.id == id)
            return &preset;
    return nullptr;
}

void buildLayout(ImGuiID dockspace, ImVec2 size, const LayoutPreset &preset, const std::string &center,
                 const std::vector<LayoutWindow> &windows) {
    // Group the windows by the area this preset puts them in; only used areas get a node.
    std::map<Dock, std::vector<const LayoutWindow *>> areas;
    for (const auto &window : windows)
        areas[preset.area(window.area)].push_back(&window);
    const auto used = [&](Dock area) { return areas.count(area) > 0; };

    // Start from an empty dock space.
    ImGui::DockBuilderRemoveNode(dockspace); // undocks every window (their saved dock IDs too)
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, size);

    // Split off the left column, right column and bottom strip; `rest` ends up as the central node.
    ImGuiID rest = dockspace, left = 0, leftTop = 0, right = 0, rightBottom = 0, bottom = 0;
    float leftShare = 0;
    if (used(Dock::Left) || used(Dock::LeftTop)) {
        left = ImGui::DockBuilderSplitNode(rest, ImGuiDir_Left, preset.left, nullptr, &rest);
        leftShare = preset.left;
    }
    // Split ratios are of the node being split; keep `right` a fraction of the whole dock space.
    if (used(Dock::Right) || used(Dock::RightBottom))
        right = ImGui::DockBuilderSplitNode(rest, ImGuiDir_Right, std::min(.9f, preset.right / (1 - leftShare)),
                                            nullptr, &rest);
    if (used(Dock::Bottom))
        bottom = ImGui::DockBuilderSplitNode(rest, ImGuiDir_Down, preset.bottom, nullptr, &rest);

    // The sub-areas take the whole column when the column's main area is unused.
    if (used(Dock::LeftTop))
        leftTop =
            used(Dock::Left) ? ImGui::DockBuilderSplitNode(left, ImGuiDir_Up, preset.leftTop, nullptr, &left) : left;
    if (used(Dock::RightBottom))
        rightBottom = used(Dock::Right)
                          ? ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, preset.rightBottom, nullptr, &right)
                          : right;

    ImGui::DockBuilderDockWindow(center.c_str(), rest);
    std::vector<std::pair<ImGuiID, const LayoutWindow *>> shownTabs;
    // Stacked windows each take an even share of the area, in order; tabbed ones share the last share.
    const auto place = [&](Dock area, ImGuiID node, ImGuiDir direction) {
        if (!node)
            return;

        // Split the node into one part per stacked window plus one for all the tabbed ones.
        std::vector<const LayoutWindow *> stacked, tabbed;
        for (const auto *window : areas[area])
            (window->stacked ? stacked : tabbed).push_back(window);
        const int parts = int(stacked.size()) + (tabbed.empty() ? 0 : 1);
        std::vector<ImGuiID> nodes;
        for (int i = 0; i + 1 < parts; ++i)
            nodes.push_back(ImGui::DockBuilderSplitNode(node, direction, 1.f / float(parts - i), nullptr, &node));
        nodes.push_back(node);

        // Dock the windows; remember which tab each multi-tab node should open on.
        std::size_t next = 0;
        for (const auto *window : stacked)
            ImGui::DockBuilderDockWindow(window->name.c_str(), nodes[next++]);
        const LayoutWindow *shown = nullptr;
        for (const auto *window : tabbed) {
            ImGui::DockBuilderDockWindow(window->name.c_str(), nodes[next]);
            if (!shown && window->selected)
                shown = window;
        }
        if (tabbed.size() > 1)
            shownTabs.emplace_back(nodes[next], shown ? shown : tabbed.front());
    };

    // Floating windows have no area node: DockBuilderRemoveNode above already undocked them.
    place(Dock::LeftTop, leftTop, ImGuiDir_Up);
    place(Dock::Left, left, ImGuiDir_Up);
    place(Dock::Right, right, ImGuiDir_Up);
    place(Dock::RightBottom, rightBottom, ImGuiDir_Up);
    place(Dock::Bottom, bottom, ImGuiDir_Left);
    ImGui::DockBuilderFinish(dockspace);

    // A new node's tab bar opens on its SelectedTabId: the window's tab ID ("#TAB" in the window's ID scope).
    for (const auto &[id, window] : shownTabs)
        if (auto *node = ImGui::DockBuilderGetNode(id))
            node->SelectedTabId = ImHashStr("#TAB", 0, ImHashStr(window->name.c_str()));
}

namespace {

// The dock space's central node (the pool view), or null before the dock space exists.
ImGuiDockNode *centralNode(ImGuiID dockspace) {
    auto *root = ImGui::DockBuilderGetNode(dockspace);
    return root ? root->CentralNode : nullptr;
}

} // namespace

float sideWidth(ImGuiID dockspace, Side side) {
    const auto *root = ImGui::DockBuilderGetNode(dockspace);
    const auto *central = centralNode(dockspace);
    if (!root || !central || root->Size.x <= 0)
        return 0;
    return std::max(0.f, side == Side::Left ? central->Pos.x - root->Pos.x
                                            : root->Pos.x + root->Size.x - central->Pos.x - central->Size.x);
}

std::vector<std::string> sideWindows(ImGuiID dockspace, Side side) {
    std::vector<std::string> names;
    const auto *central = centralNode(dockspace);
    if (!central)
        return names;
    // Active windows docked in this dock space, classified by where their node sits relative to the central node
    // (1 px slack for rounding).
    for (const auto *window : ImGui::GetCurrentContext()->Windows) {
        const auto *node = window->DockNode;
        if (!window->WasActive || !node || node == central ||
            ImGui::DockNodeGetRootNode(window->DockNode)->ID != dockspace)
            continue;
        const bool left = node->Pos.x + node->Size.x <= central->Pos.x + 1;
        const bool right = node->Pos.x >= central->Pos.x + central->Size.x - 1;
        if (side == Side::Left ? left : right)
            names.push_back(window->Name);
    }
    return names;
}

bool setSideWidth(ImGuiID dockspace, Side side, float width) {
    const float current = sideWidth(dockspace, side);
    auto *node = centralNode(dockspace);
    if (current <= 0 || !node)
        return false;
    // The split nearest the central node with the central side toward the side's opposite: its other child is
    // the part of the side bordering the pool view; widen or narrow that by the difference.
    for (; node->ParentNode; node = node->ParentNode) {
        auto *parent = node->ParentNode;
        if (parent->SplitAxis != ImGuiAxis_X)
            continue;
        auto *beside = side == Side::Left ? (parent->ChildNodes[1] == node ? parent->ChildNodes[0] : nullptr)
                                          : (parent->ChildNodes[0] == node ? parent->ChildNodes[1] : nullptr);
        if (!beside)
            continue;
        const float target = std::max(40.f, beside->Size.x + width - current); // keep at least 40 px
        ImGui::DockBuilderSetNodeSize(beside->ID, {target, std::max(1.f, beside->Size.y)});
        return true;
    }
    return false;
}

bool windowInFront(const char *name) {
    const auto *window = ImGui::FindWindowByName(name);
    return window && window->WasActive && !window->Collapsed && (!window->DockIsActive || window->DockTabIsVisible);
}

// The handler reads "key=0|1" lines of the [Nereus][Windows] section into saved_, applies them once the ini is
// loaded, and writes saved_ overlaid with the current flags (so absent windows keep their saved state).
void WindowStates::install() {
    ImGuiSettingsHandler handler;
    handler.TypeName = "Nereus";
    handler.TypeHash = ImHashStr("Nereus");
    handler.UserData = this;

    handler.ClearAllFn = [](ImGuiContext *, ImGuiSettingsHandler *h) {
        static_cast<WindowStates *>(h->UserData)->saved_.clear();
    };
    handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *h, const char *name) -> void * {
        return std::strcmp(name, "Windows") == 0 ? h->UserData : nullptr;
    };
    handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler *, void *entry, const char *line) {
        const char *equals = std::strrchr(line, '=');
        if (equals && equals != line)
            static_cast<WindowStates *>(entry)->saved_[std::string(line, equals)] = std::atoi(equals + 1) != 0;
    };
    handler.ApplyAllFn = [](ImGuiContext *, ImGuiSettingsHandler *h) {
        static_cast<WindowStates *>(h->UserData)->apply();
    };
    handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler *h, ImGuiTextBuffer *out) {
        auto &self = *static_cast<WindowStates *>(h->UserData);
        std::map<std::string, bool> states = self.saved_;
        for (const auto &[key, flag] : self.flags_())
            states[key] = *flag;
        out->append("[Nereus][Windows]\n");
        for (const auto &[key, open] : states)
            out->appendf("%s=%d\n", key.c_str(), open ? 1 : 0);
        out->append("\n");
    };
    ImGui::AddSettingsHandler(&handler);
}

namespace {

// "key=1;key=0;..." of the current flags, to detect operator changes cheaply.
std::string signatureOf(const WindowStates::Flags &flags) {
    std::string signature;
    for (const auto &[key, flag] : flags)
        signature += key + (*flag ? "=1;" : "=0;");
    return signature;
}

} // namespace

void WindowStates::apply() {
    const auto flags = flags_();
    for (const auto &[key, flag] : flags) {
        const auto found = saved_.find(key);
        if (found != saved_.end())
            *flag = found->second;
    }
    signature_ = signatureOf(flags); // applying saved states is not an operator change
}

void WindowStates::update() {
    auto signature = signatureOf(flags_());
    if (signature == signature_)
        return;
    // The first call only records the baseline.
    if (!signature_.empty() && ImGui::GetCurrentContext())
        ImGui::MarkIniSettingsDirty();
    signature_ = std::move(signature);
}

std::filesystem::path configDirectory() {
    if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg)
        return std::filesystem::path(xdg) / "nereus";
    if (const char *home = std::getenv("HOME"); home && *home)
        return std::filesystem::path(home) / ".config" / "nereus";
    return {};
}

std::string layoutFileStem(const std::string &name) {
    std::string stem;
    for (const char c : name)
        if (std::isalnum(static_cast<unsigned char>(c)) || c == ' ' || c == '-' || c == '_')
            stem += c;

    // Trim surrounding spaces.
    const auto first = stem.find_first_not_of(' '), last = stem.find_last_not_of(' ');
    return first == std::string::npos ? std::string() : stem.substr(first, last - first + 1);
}

std::vector<std::string> savedLayouts(const std::filesystem::path &dir) {
    std::vector<std::string> names;
    std::error_code error;
    if (dir.empty() || !std::filesystem::is_directory(dir, error))
        return names;
    for (const auto &entry : std::filesystem::directory_iterator(dir, error))
        if (entry.is_regular_file(error) && entry.path().extension() == ".ini")
            names.push_back(entry.path().stem().string());
    std::sort(names.begin(), names.end());
    return names;
}

} // namespace nereus::ros_viewer
