#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <imgui.h>
#include <set>
#include <stdexcept>

namespace nereus::ros_viewer::panels {
void keys(const YAML::Node &node, std::initializer_list<const char *> allowed, const std::string &where) {
    if (!node || !node.IsMap())
        throw std::invalid_argument(where + ": expected a mapping");
    std::set<std::string> known(allowed.begin(), allowed.end()), seen;
    for (const auto &item : node) {
        auto key = item.first.as<std::string>();
        if (!known.count(key) || !seen.insert(key).second)
            throw std::invalid_argument(where + ": unknown or duplicate key '" + key + "'");
    }
}
void required(const YAML::Node &node, std::initializer_list<const char *> names) {
    for (const auto *name : names)
        if (!node[name] || !node[name].IsScalar() || node[name].as<std::string>().empty())
            throw std::invalid_argument(std::string("missing/non-scalar option '") + name + "'");
}
void positive(const YAML::Node &node, const char *key, double fallback, double maximum) {
    const auto value = node[key].as<double>(fallback);
    if (!std::isfinite(value) || value <= 0 || value > maximum)
        throw std::invalid_argument(std::string("invalid range for '") + key + "'");
}
std::string expand(std::string value, const Context &ctx) {
    for (const auto &entry :
         {std::make_pair("{namespace}", ctx.robotNamespace), std::make_pair("{fixed_frame}", ctx.fixedFrame)}) {
        size_t at = 0;
        while ((at = value.find(entry.first, at)) != std::string::npos) {
            value.replace(at, std::strlen(entry.first), entry.second);
            at += entry.second.size();
        }
    }
    if (value.find('{') != std::string::npos || value.find('}') != std::string::npos)
        throw std::invalid_argument("unresolved substitution: " + value);
    return value;
}
namespace {
// Items laid out left to right from the cursor, each placed explicitly: a SameLine before an item that then draws
// nothing would leave the line open and shift whatever the caller draws next onto it.
template <typename Draw> void row(Draw &&draw, std::size_t count) {
    const ImVec2 start = ImGui::GetCursorPos();
    float x = start.x;
    bool open = false; // the cursor was placed with no item after it
    for (std::size_t i = 0; i < count; ++i) {
        ImGui::SetCursorPos({x, start.y});
        open = true;
        const auto before = ImGui::GetCursorPos();
        draw(i);
        const auto after = ImGui::GetCursorPos();
        if (after.x != before.x || after.y != before.y) { // drew something
            x = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x + ImGui::GetScrollX() +
                ImGui::GetStyle().ItemSpacing.x;
            open = false;
        }
    }
    if (open)
        ImGui::Dummy({0, 0}); // close the placement (ImGui checks a SetCursorPos is followed by an item)
}
void validateSubstitutions(const YAML::Node &node, const Context &ctx) {
    if (node.IsScalar())
        expand(node.as<std::string>(), ctx);
    else if (node.IsSequence())
        for (const auto &item : node)
            validateSubstitutions(item, ctx);
    else if (node.IsMap())
        for (const auto &item : node)
            validateSubstitutions(item.second, ctx);
}
} // namespace
Dock parseDock(const std::string &name) {
    static const std::map<std::string, Dock> areas{{"left_top", Dock::LeftTop}, {"left", Dock::Left},
                                                   {"right", Dock::Right},      {"right_bottom", Dock::RightBottom},
                                                   {"bottom", Dock::Bottom},    {"floating", Dock::Floating}};
    const auto found = areas.find(name);
    if (found == areas.end())
        throw std::invalid_argument("unknown dock area '" + name +
                                    "' (left_top, left, right, right_bottom, bottom, floating)");
    return found->second;
}
std::string panelWindowName(const std::string &id, const std::string &title) {
    return title + "###panel." + id;
}
Composition::Composition(const YAML::Node &config, const Context &ctx, const Registry &registry) {
    keys(config,
         {"sidebar_width", "sidebar_width_fraction", "sidebar_visible", "providers", "panels", "toolbar", "header",
          "overlays", "ownership"},
         "composition");
    const bool panelsShown = config["sidebar_visible"].as<bool>(true);
    if (config["sidebar_width"] && config["sidebar_width_fraction"])
        throw std::invalid_argument("choose sidebar_width or sidebar_width_fraction, not both");
    sidebarFraction = config["sidebar_width"] ? 0.f : config["sidebar_width_fraction"].as<float>(.29f);
    if (!std::isfinite(sidebarFraction) || sidebarFraction < 0 || sidebarFraction >= 1 ||
        (config["sidebar_width_fraction"] && sidebarFraction == 0))
        throw std::invalid_argument("composition.sidebar_width_fraction must be greater than 0 and less than 1");
    sidebarWidth = config["sidebar_width"].as<float>(350);
    if (!std::isfinite(sidebarWidth) || sidebarWidth < 300 || sidebarWidth > 600)
        throw std::invalid_argument("composition.sidebar_width must be 300..600 pixels");
    const auto definitions = config["providers"];
    if (!definitions || !definitions.IsMap())
        throw std::invalid_argument("composition.providers must be a mapping ({} is valid)");
    std::map<std::string, Kind> kinds;
    for (const auto &entry : definitions) {
        const auto id = entry.first.as<std::string>();
        try {
            keys(entry.second, {"type", "options"}, "provider");
            required(entry.second, {"type"});
            const auto &factory = registry.providers.at(entry.second["type"].as<std::string>());
            if (id.empty() || !kinds.emplace(id, factory.kind).second)
                throw std::invalid_argument("empty or duplicate provider ID");
            factory.validate(entry.second["options"]);
            validateSubstitutions(entry.second["options"], ctx);
        } catch (const std::exception &e) {
            throw std::invalid_argument("providers." + id + ": " + e.what());
        }
    }
    // A missing key yields an invalid node that must not be assigned to; build the default separately.
    YAML::Node toolbarConfig(YAML::NodeType::Sequence);
    if (config["toolbar"])
        toolbarConfig = YAML::Clone(config["toolbar"]);
    else
        for (const auto &item : defaultToolbar())
            if (registry.panels.count(item["type"].as<std::string>()))
                toolbarConfig.push_back(item);
    const auto validateViews = [&](const char *name, const YAML::Node &items, const auto &factories) {
        // Header items are toolbar items drawn in the header row: same schema, ids default to the type.
        const bool isToolbar = std::string(name) == "toolbar" || std::string(name) == "header";
        if (items && !items.IsSequence())
            throw std::invalid_argument(std::string(name) + ": expected a sequence");
        std::set<std::string> ids;
        for (const auto &item : items) {
            std::string id = item.IsMap() ? item["id"].template as<std::string>("") : "";
            if (id.empty() && isToolbar && item.IsMap())
                id = item["type"].template as<std::string>("");
            try {
                keys(item, {"id", "type", "provider", "title", "visible", "open", "dock", "options"}, name);
                required(item, {"type"});
                if (!isToolbar)
                    required(item, {"id"});
                if (item["dock"] && std::string(name) != "panels")
                    throw std::invalid_argument("'dock' places panel windows; " + std::string(name) +
                                                " items have no window");
                if (item["dock"])
                    parseDock(item["dock"].template as<std::string>());
                const auto type = item["type"].template as<std::string>();
                const auto found = factories.find(type);
                if (found == factories.end()) {
                    std::string known;
                    for (const auto &entry : factories)
                        known += (known.empty() ? "" : ", ") + entry.first;
                    throw std::invalid_argument("unknown " + std::string(isToolbar ? "toolbar item" : "panel") +
                                                " type '" + type + "' (known: " + known + ")");
                }
                const auto &factory = found->second;
                if (!ids.insert(id).second)
                    throw std::invalid_argument("duplicate instance ID (give repeated items distinct ids)");
                if (factory.hosted) {
                    if (item["provider"])
                        throw std::invalid_argument("'" + type + "' is a host item and takes no provider");
                } else {
                    required(item, {"provider"});
                    const auto provider = item["provider"].template as<std::string>();
                    if (!kinds.count(provider))
                        throw std::invalid_argument("unknown provider '" + provider + "'");
                    if (factory.kind != kinds.at(provider))
                        throw std::invalid_argument("provider capability mismatch");
                }
                if (!isToolbar && factory.toolbarOnly)
                    throw std::invalid_argument("'" + type + "' is toolbar-only and cannot be a panel window");
                (void)item["visible"].template as<bool>(true);
                (void)item["open"].template as<bool>(true);
                (void)item["title"].template as<std::string>(id);
                factory.validate(item["options"] ? item["options"] : YAML::Node(YAML::NodeType::Map));
            } catch (const std::exception &e) {
                throw std::invalid_argument(std::string(name) + "." + id + ": " + e.what());
            }
        }
    };
    validateViews("panels", config["panels"], registry.panels);
    validateViews("toolbar", toolbarConfig, registry.panels);
    validateViews("header", config["header"], registry.panels);
    validateViews("overlays", config["overlays"], registry.overlays);
    if (config["ownership"] && !config["ownership"].IsSequence())
        throw std::invalid_argument("ownership must be a sequence");
    std::set<std::string> owned;
    for (const auto &link : config["ownership"]) {
        keys(link, {"motion", "autonomy"}, "ownership");
        required(link, {"motion", "autonomy"});
        if (kinds.at(link["motion"].as<std::string>()) != Kind::Motion ||
            kinds.at(link["autonomy"].as<std::string>()) != Kind::Autonomy ||
            !owned.insert(link["motion"].as<std::string>()).second)
            throw std::invalid_argument("ownership requires one autonomy provider per motion provider");
    }
    // No provider is instantiated until the entire composition has validated.
    if (!ctx.preview)
        for (const auto &entry : definitions)
            sources.emplace(
                entry.first.as<std::string>(),
                registry.providers.at(entry.second["type"].as<std::string>()).create(entry.second["options"], ctx));
    for (const auto &link : config["ownership"])
        if (!ctx.preview)
            ownership.push_back({std::dynamic_pointer_cast<Motion>(sources.at(link["motion"].as<std::string>())),
                                 std::dynamic_pointer_cast<Autonomy>(sources.at(link["autonomy"].as<std::string>()))});
    const auto binding = [&](const YAML::Node &item) {
        Binding b;
        b.documents = ctx.documents;
        const auto providerId = item["provider"].as<std::string>("");
        b.drawOverlayControls = [this, provider = providerId] { drawOverlayControls(provider); };
        b.drawPanelMenu = [this] { drawPanelMenu(); };
        b.focus = ctx.focus;
        const auto id = item["id"].as<std::string>(item["type"].as<std::string>("")); // toolbar ids default to the type
        b.showWindow = std::find(ctx.initialWindows.begin(), ctx.initialWindows.end(), id) != ctx.initialWindows.end();
        if (!ctx.preview && !providerId.empty())
            b.provider = sources.at(providerId);
        b.options = item["options"] ? item["options"] : YAML::Node(YAML::NodeType::Map);
        b.mayStart = [this, provider = b.provider] {
            for (const auto &link : ownership)
                if (link.mission == provider) {
                    const auto state = link.motion->state();
                    if (!state.enabled || !state.fresh || state.pending || state.competing)
                        return false;
                }
            return true;
        };
        b.kill = [this, provider = b.provider] {
            if (auto motion = std::dynamic_pointer_cast<Motion>(provider)) {
                motion->kill();
                for (const auto &link : ownership)
                    if (link.motion == motion && link.mission->state().busy)
                        link.mission->stop();
            }
        };
        return b;
    };
    for (const auto &item : config["panels"]) {
        const auto id = item["id"].as<std::string>();
        panelInstances.push_back({id, item["title"].as<std::string>(id), panelsShown && item["visible"].as<bool>(true),
                                  item["open"].as<bool>(true),
                                  registry.panels.at(item["type"].as<std::string>()).create(binding(item)),
                                  parseDock(item["dock"].as<std::string>("left"))});
    }
    for (const auto &item : toolbarConfig) {
        const auto type = item["type"].as<std::string>();
        const auto id = item["id"].as<std::string>(type);
        toolbarInstances.push_back({id, item["title"].as<std::string>(id), item["visible"].as<bool>(true), true,
                                    registry.panels.at(type).create(binding(item)), Dock::Left, false, type,
                                    item["title"].as<std::string>("")});
    }
    for (const auto &item : config["header"]) {
        const auto type = item["type"].as<std::string>();
        const auto id = item["id"].as<std::string>(type);
        headerInstances.push_back({id, item["title"].as<std::string>(id), item["visible"].as<bool>(true), true,
                                   registry.panels.at(type).create(binding(item))});
    }
    for (const auto &item : config["overlays"])
        overlays.push_back({item["id"].as<std::string>(), item["title"].as<std::string>(item["id"].as<std::string>()),
                            item["visible"].as<bool>(true),
                            registry.overlays.at(item["type"].as<std::string>()).create(binding(item)),
                            item["provider"].as<std::string>()});
}
void Composition::syncOwnership() {
    for (const auto &link : ownership)
        link.motion->block(link.mission->state().busy);
}
void Composition::touch() {
    for (const auto &source : sources)
        source.second->touch();
    syncOwnership();
}
void Composition::drawPanelMenu() {
    nereus::ros_viewer::sameLineIfFits(nereus::ros_viewer::buttonWidth("Windows"));
    if (ImGui::Button("Windows"))
        ImGui::OpenPopup("panel_menu");
    if (ImGui::BeginPopup("panel_menu")) {
        if (windowMenu)
            windowMenu();
        else
            drawPanelMenuItems();
        ImGui::EndPopup();
    }
}
void Composition::drawPanelMenuItems() {
    for (auto &item : panelInstances)
        if (ImGui::MenuItem(item.title.c_str(), nullptr, item.visible)) {
            item.visible = !item.visible;
            item.focus = item.visible; // shown on top (its tab selected) when reopened
        }
}
void Composition::drawToolMenuItems() {
    for (auto *group : {&toolbarInstances, &panelInstances})
        for (auto &item : *group) {
            ImGui::PushID(item.id.c_str());
            item.panel->windowMenu();
            ImGui::PopID();
        }
}
std::vector<Composition::ToolbarItem> Composition::toolbarItems() {
    std::vector<ToolbarItem> items;
    for (auto &item : toolbarInstances)
        items.push_back({item.id, item.type, item.configuredTitle, &item.visible});
    return items;
}
std::vector<Composition::PanelWindow> Composition::panelWindows() const {
    std::vector<PanelWindow> windows;
    for (const auto &item : panelInstances)
        windows.push_back({item.id, panelWindowName(item.id, item.title), item.dock, item.open});
    return windows;
}
std::vector<std::pair<std::string, bool *>> Composition::visibility() {
    std::vector<std::pair<std::string, bool *>> flags;
    for (auto &item : panelInstances)
        flags.emplace_back("panel." + item.id, &item.visible);
    return flags;
}
void Composition::focusPanel(const std::string &id) {
    for (auto &item : panelInstances)
        if (item.id == id)
            item.focus = item.visible = true;
}
void Composition::drawToolbar() {
    for (auto &item : toolbarInstances)
        if (item.visible) {
            ImGui::PushID(item.id.c_str());
            item.panel->toolbar();
            ImGui::PopID();
        }
}
void Composition::drawHeader(float right) {
    if (headerInstances.empty())
        return;
    const float after = ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x + ImGui::GetStyle().ItemSpacing.x;
    // The caller sets the line's y (not SameLine: the items' own SameLine would then return to the line of the
    // item before, e.g. text sitting lower than a button); this sets x only.
    ImGui::SetCursorPosX(std::max(after, right - headerWidth));
    ImGui::BeginGroup();
    row( // items may draw nothing (e.g. no recording running); only drawn ones take space
        [&](std::size_t i) {
            auto &item = headerInstances[i];
            if (!item.visible)
                return;
            ImGui::PushID(item.id.c_str());
            item.panel->header();
            ImGui::PopID();
        },
        headerInstances.size());
    ImGui::EndGroup();
    headerWidth = ImGui::GetItemRectSize().x;
}
std::vector<std::string> Composition::toolbarIds() const {
    std::vector<std::string> ids;
    for (const auto &item : toolbarInstances)
        ids.push_back(item.id);
    return ids;
}
std::vector<std::string> Composition::panelIds() const {
    std::vector<std::string> ids;
    for (const auto &item : panelInstances)
        ids.push_back(item.id);
    return ids;
}
const std::vector<HostItemType> &hostItemTypes() {
    static const std::vector<HostItemType> types{
        {"scene_settings", false}, {"pool_viewer", false},  {"view", false},     {"focus", false},
        {"follow", false},         {"labels", false},       {"tf", false},       {"mpc_path", false},
        {"thrust", false},         {"preview_task", false}, {"detections", true}};
    return types;
}

void registerHostItem(Registry &registry, const std::string &type, std::function<void()> toolbar,
                      std::function<void()> panel) {
    struct HostPanel final : Panel {
        std::function<void()> bar, body;
        void toolbar() override {
            if (bar)
                bar();
        }
        void draw() override {
            if (body)
                body();
        }
    };
    const bool sidebar = static_cast<bool>(panel);
    registry.panels.insert_or_assign(type, ViewFactory<Panel>{Kind::Motion,
                                                              [type](const YAML::Node &n) { keys(n, {}, type); },
                                                              [toolbar, panel](const Binding &) {
                                                                  auto item = std::make_unique<HostPanel>();
                                                                  item->bar = toolbar;
                                                                  item->body = panel;
                                                                  return std::unique_ptr<Panel>(std::move(item));
                                                              },
                                                              true, !sidebar});
}

void registerHostPlaceholders(Registry &registry) {
    for (const auto &item : hostItemTypes())
        registerHostItem(registry, item.type, [] {}, item.sidebar ? std::function<void()>([] {}) : nullptr);
}

YAML::Node defaultToolbar() {
    return YAML::Load("[{type: scene_settings}, {type: pool_viewer}, {type: panels_menu}, {type: view}, "
                      "{type: focus}, {type: follow}, {type: labels}, {type: tf}, "
                      "{type: mpc_path}, {type: thrust}, {type: preview_task}]");
}

void Composition::drawPinned() {
    row(
        [&](std::size_t i) {
            auto &item = panelInstances[i];
            ImGui::PushID(item.id.c_str());
            pins::beginScope("panel." + item.id, item.title);
            item.panel->pinned();
            pins::endScope();
            ImGui::PopID();
        },
        panelInstances.size());
}
void Composition::drawPanels() {
    for (auto &item : panelInstances) {
        if (!item.visible)
            continue;
        if (item.focus) {
            ImGui::SetNextWindowFocus();
            item.focus = false;
        }
        ImGui::SetNextWindowSize(ui(ImVec2(380, 520)), ImGuiCond_FirstUseEver);
        if (ImGui::Begin(panelWindowName(item.id, item.title).c_str(), &item.visible)) {
            if (windowContextMenu)
                windowContextMenu(item.id);
            ImGui::PushID(item.id.c_str());
            pins::beginScope("panel." + item.id, item.title);
            item.panel->draw();
            pins::endScope();
            ImGui::PopID();
        }
        ImGui::End();
    }
    // Panels with controls pinned to the toolbar keep running while closed or behind another tab.
    for (auto &item : panelInstances)
        if (pins::needsDrawing("panel." + item.id))
            pins::drawOffscreen("panel." + item.id, item.title, [&] {
                ImGui::PushID(item.id.c_str());
                item.panel->draw();
                ImGui::PopID();
            });
    syncOwnership();
}
void Composition::drawWindows() {
    for (auto &item : toolbarInstances) {
        ImGui::PushID(item.id.c_str());
        pins::beginScope("tool." + item.id, toolTitle(item));
        item.panel->drawWindows();
        pins::endScope();
        ImGui::PopID();
    }
    for (auto &item : panelInstances) {
        ImGui::PushID(item.id.c_str());
        pins::beginScope("panel." + item.id, item.title);
        item.panel->drawWindows();
        pins::endScope();
        ImGui::PopID();
    }
    // A tool window (Simulation, the run scorecard) with pinned controls keeps running while closed.
    for (auto &item : toolbarInstances)
        if (pins::needsDrawing("tool." + item.id))
            pins::drawOffscreen("tool." + item.id, toolTitle(item), [&] {
                ImGui::PushID(item.id.c_str());
                item.panel->draw();
                ImGui::PopID();
            });
}
std::string Composition::toolTitle(const PanelInstance &item) {
    if (!item.configuredTitle.empty())
        return item.configuredTitle;
    std::string title = item.type;
    if (!title.empty())
        title[0] = char(std::toupper(static_cast<unsigned char>(title[0])));
    return title == "Run" ? "Run tracking" : title;
}
void Composition::drawOverlayControls(const std::string &provider) {
    for (auto &item : overlays) {
        if (item.provider != provider)
            continue;
        ImGui::PushID(item.id.c_str());
        if (pins::Checkbox(item.title.c_str(), &item.visible) && !item.visible)
            item.overlay->cancelInteraction();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show %s in the pool view", item.title.c_str());
        ImGui::PopID();
    }
}
bool Composition::input(const Viewport &view) {
    bool consumed = false;
    for (auto &overlay : overlays) {
        auto input = view;
        input.interactive = view.interactive && !consumed;
        if (overlay.visible)
            consumed = overlay.overlay->input(input) || consumed;
    }
    return consumed;
}
void Composition::drawOverlays(const Viewport &view) {
    for (auto &overlay : overlays)
        if (overlay.visible)
            overlay.overlay->draw(view);
}
} // namespace nereus::ros_viewer::panels
