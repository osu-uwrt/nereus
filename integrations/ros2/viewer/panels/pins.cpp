#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <imgui_internal.h>
#include <map>
#include <set>

namespace nereus::ros_viewer::pins {
namespace {
enum class Kind { Button, Checkbox, Combo };

struct Record {
    Kind kind = Kind::Button;
    std::string label, scopeTitle;
    bool disabled = false, value = false;
    int current = 0;
    std::vector<std::string> items;
    ImVec4 colors[4]{}; // button, hovered, active, text as the original was drawn
    int frame = -100;   // last frame the original was drawn
};

struct Board {
    std::map<std::string, Record> records;
    std::vector<std::string> pinned;    // in pin order
    std::map<std::string, int> pending; // key -> click (1) / toggle (1) / selection + 1
    std::string scope, title;
    std::vector<std::string> parts;
    std::set<std::string> seen; // scopes whose widgets ran this frame
    std::string menuKey;        // the control the right-click menu is for
};
Board &board() {
    static Board instance;
    return instance;
}
const ImGuiID kMenuId = ImHashStr("##nereus_pin_menu");

// "Pause###pause" -> id "pause", label "Pause"; "Fire" -> both "Fire"; "##tree" -> id "tree", label "".
std::string idOf(const char *label) {
    const char *triple = std::strstr(label, "###");
    if (triple)
        return triple + 3;
    const char *hidden = std::strstr(label, "##");
    return hidden ? std::string(hidden + 2) : std::string(label);
}
std::string textOf(const char *label) {
    const char *hidden = std::strstr(label, "##");
    return hidden ? std::string(label, hidden) : std::string(label);
}
bool scoped() {
    return !board().scope.empty();
}
std::string keyOf(const char *label) {
    auto &b = board();
    std::string key = b.scope;
    for (const auto &part : b.parts)
        key += "/" + part;
    return key + "/" + idOf(label);
}
bool isPinned(const std::string &key) {
    const auto &p = board().pinned;
    return std::find(p.begin(), p.end(), key) != p.end();
}
Record &record(const std::string &key, Kind kind, const char *label) {
    auto &b = board();
    auto &r = b.records[key];
    r.kind = kind;
    r.label = textOf(label);
    r.scopeTitle = b.title;
    r.disabled = (GImGui->CurrentItemFlags & ImGuiItemFlags_Disabled) != 0;
    const auto &colors = ImGui::GetStyle().Colors;
    r.colors[0] = colors[ImGuiCol_Button];
    r.colors[1] = colors[ImGuiCol_ButtonHovered];
    r.colors[2] = colors[ImGuiCol_ButtonActive];
    r.colors[3] = colors[ImGuiCol_Text];
    r.frame = ImGui::GetFrameCount();
    b.seen.insert(b.scope);
    return r;
}
// Right-click on the widget just drawn (even disabled) opens the pin menu; not from inside another popup.
void offerMenu(const std::string &key) {
    if (GImGui->BeginPopupStack.Size > 0)
        return;
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled) && ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
        board().menuKey = key;
        ImGui::OpenPopup(kMenuId);
    }
}
bool take(const std::string &key, int &value) {
    auto &pending = board().pending;
    const auto found = pending.find(key);
    if (found == pending.end())
        return false;
    value = found->second;
    pending.erase(found);
    return true;
}
} // namespace

void beginScope(const std::string &scope, const std::string &title) {
    board().scope = scope;
    board().title = title;
    board().parts.clear();
}
void endScope() {
    board().scope.clear();
    board().parts.clear();
}
Scope::Scope(const std::string &part) {
    board().parts.push_back(part);
    ImGui::PushID(part.c_str());
}
Scope::~Scope() {
    ImGui::PopID();
    board().parts.pop_back();
}

bool Button(const char *label, const ImVec2 &size) {
    if (!scoped())
        return ImGui::Button(label, size);
    const auto key = keyOf(label);
    record(key, Kind::Button, label);
    bool clicked = ImGui::Button(label, size);
    offerMenu(key);
    int value = 0;
    if (take(key, value) && !board().records[key].disabled)
        clicked = true;
    return clicked;
}

bool Checkbox(const char *label, bool *v) {
    if (!scoped())
        return ImGui::Checkbox(label, v);
    const auto key = keyOf(label);
    auto *r = &record(key, Kind::Checkbox, label);
    bool changed = ImGui::Checkbox(label, v);
    offerMenu(key);
    int value = 0;
    if (take(key, value) && !r->disabled) {
        *v = !*v;
        changed = true;
    }
    board().records[key].value = *v;
    return changed;
}

bool Combo(const char *label, int *current, const char *items) {
    if (!scoped())
        return ImGui::Combo(label, current, items);
    const auto key = keyOf(label);
    auto &r = record(key, Kind::Combo, label);
    r.items.clear();
    for (const char *item = items; *item; item += std::strlen(item) + 1)
        r.items.emplace_back(item);
    bool changed = ImGui::Combo(label, current, items);
    offerMenu(key);
    int value = 0;
    auto &now = board().records[key];
    if (take(key, value) && !now.disabled && value - 1 >= 0 && value - 1 < int(now.items.size()) &&
        value - 1 != *current) {
        *current = value - 1;
        changed = true;
    }
    now.current = *current;
    return changed;
}

void newFrame() {
    board().seen.clear();
}

void drawMenu() {
    auto &b = board();
    if (!ImGui::BeginPopupEx(kMenuId, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar |
                                          ImGuiWindowFlags_NoSavedSettings))
        return;
    const bool pinned = isPinned(b.menuKey);
    const auto found = b.records.find(b.menuKey);
    if (found != b.records.end() && !found->second.label.empty())
        ImGui::TextDisabled("%s", found->second.label.c_str());
    if (ImGui::MenuItem("Pin to toolbar", nullptr, pinned))
        setPinned(b.menuKey, !pinned);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("A copy of this control in the pool view's toolbar, working even with its window closed");
    ImGui::EndPopup();
}

bool needsDrawing(const std::string &scope) {
    const auto &b = board();
    if (b.seen.count(scope))
        return false;
    const std::string prefix = scope + "/";
    return std::any_of(b.pinned.begin(), b.pinned.end(),
                       [&](const std::string &key) { return key.compare(0, prefix.size(), prefix) == 0; });
}

void drawOffscreen(const std::string &scope, const std::string &title, const std::function<void()> &body) {
    // Far outside the display, never focused or clicked: its widgets run (and record) but cannot be seen.
    ImGui::SetNextWindowPos({-100000, -100000});
    ImGui::SetNextWindowSize({ui(400), ui(800)});
    const auto name = "##pins_offscreen." + scope;
    ImGui::Begin(name.c_str(), nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoNav |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoDocking |
                     ImGuiWindowFlags_NoBackground);
    beginScope(scope, title);
    body();
    endScope();
    ImGui::End();
}

void drawPinned() {
    auto &b = board();
    const int frame = ImGui::GetFrameCount();
    for (const auto &key : std::vector<std::string>(b.pinned)) {
        const auto &r = b.records[key];
        const bool live = frame - r.frame <= 2;
        const std::string text = r.label.empty() ? key.substr(key.rfind('/') + 1) : r.label;
        ImGui::PushID(key.c_str());
        if (r.kind == Kind::Combo) {
            sameLineIfFits(ui(160));
            ImGui::SetNextItemWidth(ui(160));
        } else
            sameLineIfFits(buttonWidth(text.c_str()));
        ImGui::BeginDisabled(!live || r.disabled);
        if (r.kind == Kind::Button) {
            // As the original was drawn (KILL red, ...); one never drawn yet (restored from a layout) as plain.
            const ImGuiCol slots[4] = {ImGuiCol_Button, ImGuiCol_ButtonHovered, ImGuiCol_ButtonActive, ImGuiCol_Text};
            for (int i = 0; i < 4; ++i)
                ImGui::PushStyleColor(slots[i], r.frame >= 0 ? r.colors[i] : ImGui::GetStyle().Colors[slots[i]]);
            if (ImGui::Button((text + "##pinned").c_str()))
                b.pending[key] = 1;
            ImGui::PopStyleColor(4);
        } else if (r.kind == Kind::Checkbox) {
            pushActiveColors(r.value);
            if (ImGui::Button((text + "##pinned").c_str()))
                b.pending[key] = 1;
            popActiveColors();
        } else {
            const char *preview = r.current >= 0 && r.current < int(r.items.size()) ? r.items[r.current].c_str() : "";
            if (ImGui::BeginCombo("##pinned", preview)) {
                for (int i = 0; i < int(r.items.size()); ++i)
                    if (ImGui::Selectable(r.items[i].c_str(), i == r.current))
                        b.pending[key] = i + 1;
                ImGui::EndCombo();
            }
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            ImGui::SetTooltip("%s%s%s (pinned; right-click to unpin)", text.c_str(), r.scopeTitle.empty() ? "" : " - ",
                              r.scopeTitle.c_str());
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
                b.menuKey = key;
                ImGui::OpenPopup(kMenuId);
            }
        }
        ImGui::PopID();
    }
}

void drawCustomization() {
    auto &b = board();
    if (b.pinned.empty()) {
        ImGui::TextDisabled("None: right-click a button, checkbox or dropdown in a panel");
        return;
    }
    for (const auto &key : std::vector<std::string>(b.pinned)) {
        const auto &r = b.records[key];
        bool keep = true;
        const std::string text =
            (r.label.empty() ? key : r.label) + (r.scopeTitle.empty() ? "" : "  (" + r.scopeTitle + ")");
        ImGui::PushID(key.c_str());
        if (ImGui::Checkbox(text.c_str(), &keep) && !keep)
            setPinned(key, false);
        ImGui::PopID();
    }
}

bool any() {
    return !board().pinned.empty();
}
void clear() {
    board().pinned.clear();
    board().pending.clear();
    if (ImGui::GetCurrentContext())
        ImGui::MarkIniSettingsDirty();
}

std::vector<std::string> pinnedKeys() {
    return board().pinned;
}
void setPinned(const std::string &key, bool pinned) {
    auto &p = board().pinned;
    const auto found = std::find(p.begin(), p.end(), key);
    if (pinned && found == p.end())
        p.push_back(key);
    else if (!pinned && found != p.end())
        p.erase(found);
    if (ImGui::GetCurrentContext())
        ImGui::MarkIniSettingsDirty();
}

// One line per pinned control: kind, key, label and window title (tab separated), so the toolbar can show it
// before its window first draws.
void install() {
    ImGuiSettingsHandler handler;
    handler.TypeName = "NereusPins";
    handler.TypeHash = ImHashStr("NereusPins");
    handler.ClearAllFn = [](ImGuiContext *, ImGuiSettingsHandler *) { board().pinned.clear(); };
    handler.ReadOpenFn = [](ImGuiContext *, ImGuiSettingsHandler *, const char *) -> void * { return &board(); };
    handler.ReadLineFn = [](ImGuiContext *, ImGuiSettingsHandler *, void *, const char *line) {
        std::vector<std::string> fields;
        for (const char *at = line;; ++at) {
            const char *end = std::strchr(at, '\t');
            fields.emplace_back(at, end ? end : at + std::strlen(at));
            if (!end)
                break;
            at = end;
        }
        if (fields.size() < 2 || fields[1].empty())
            return;
        auto &r = board().records[fields[1]];
        r.kind = fields[0] == "checkbox" ? Kind::Checkbox : fields[0] == "combo" ? Kind::Combo : Kind::Button;
        if (fields.size() > 2 && r.label.empty())
            r.label = fields[2];
        if (fields.size() > 3 && r.scopeTitle.empty())
            r.scopeTitle = fields[3];
        setPinned(fields[1], true);
    };
    handler.WriteAllFn = [](ImGuiContext *, ImGuiSettingsHandler *, ImGuiTextBuffer *out) {
        out->append("[NereusPins][Toolbar]\n");
        for (const auto &key : board().pinned) {
            const auto &r = board().records[key];
            out->appendf("%s\t%s\t%s\t%s\n",
                         r.kind == Kind::Checkbox ? "checkbox"
                         : r.kind == Kind::Combo  ? "combo"
                                                  : "button",
                         key.c_str(), r.label.c_str(), r.scopeTitle.c_str());
        }
        out->append("\n");
    };
    ImGui::AddSettingsHandler(&handler);
}
} // namespace nereus::ros_viewer::pins
