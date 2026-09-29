// Ported from riptide_simulator camera_faker pool_viewer composition.cpp; see docs/PROVENANCE.md.
#include "robotics/ros_viewer/panels/composition.hpp"
#include "robotics/ros_viewer/panel_layout.hpp"
#include <imgui.h>
#include <set>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <stdexcept>

namespace robotics::ros_viewer::panels {
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
Composition::Composition(const YAML::Node &config, const Context &ctx, const Registry &registry) {
    keys(config,
         {"schema_version", "sidebar_width", "sidebar_width_fraction", "sidebar_visible", "providers", "panels",
          "toolbar", "overlays", "ownership"},
         "composition");
    if (config["schema_version"].as<int>(0) != 1)
        throw std::invalid_argument("composition.schema_version must be 1");
    sidebarShown = config["sidebar_visible"].as<bool>(true);
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
    YAML::Node toolbarConfig = config["toolbar"];
    if (!toolbarConfig) {
        toolbarConfig = YAML::Node(YAML::NodeType::Sequence);
        for (const auto &item : defaultToolbar())
            if (registry.panels.count(item["type"].as<std::string>()))
                toolbarConfig.push_back(item);
    }
    const auto validateViews = [&](const char *name, const YAML::Node &items, const auto &factories) {
        const bool isToolbar = std::string(name) == "toolbar";
        if (items && !items.IsSequence())
            throw std::invalid_argument(std::string(name) + ": expected a sequence");
        std::set<std::string> ids;
        for (const auto &item : items) {
            std::string id = item.IsMap() ? item["id"].template as<std::string>("") : "";
            if (id.empty() && isToolbar && item.IsMap())
                id = item["type"].template as<std::string>("");
            try {
                keys(item, {"id", "type", "provider", "title", "visible", "open", "options"}, name);
                required(item, {"type"});
                if (!isToolbar)
                    required(item, {"id"});
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
                    throw std::invalid_argument("'" + type + "' is toolbar-only and cannot be a sidebar panel");
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
        b.showWindow = std::find(ctx.initialWindows.begin(), ctx.initialWindows.end(), item["id"].as<std::string>()) !=
                       ctx.initialWindows.end();
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
        panelInstances.push_back({id, item["title"].as<std::string>(id), item["visible"].as<bool>(true),
                                  item["open"].as<bool>(true),
                                  registry.panels.at(item["type"].as<std::string>()).create(binding(item))});
    }
    for (const auto &item : toolbarConfig) {
        const auto type = item["type"].as<std::string>();
        const auto id = item["id"].as<std::string>(type);
        toolbarInstances.push_back({id, item["title"].as<std::string>(id), item["visible"].as<bool>(true), true,
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
    robotics::ros_viewer::sameLineIfFits(88);
    if (ImGui::Button("Panels", {88, 30}))
        ImGui::OpenPopup("panel_menu");
    if (ImGui::BeginPopup("panel_menu")) {
        for (auto &item : panelInstances)
            ImGui::Checkbox(item.title.c_str(), &item.visible);
        ImGui::EndPopup();
    }
    if (sidebarShown)
        return;
    for (auto &item : panelInstances) {
        ImGui::PushID(item.id.c_str());
        item.panel->toolbar();
        ImGui::PopID();
    }
}
void Composition::drawToolbar() {
    for (auto &item : toolbarInstances)
        if (item.visible) {
            ImGui::PushID(item.id.c_str());
            item.panel->toolbar();
            ImGui::PopID();
        }
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
YAML::Node defaultToolbar() {
    return YAML::Load("[{type: scene_settings}, {type: pool_viewer}, {type: panels_menu}, {type: view}, "
                      "{type: focus}, {type: follow}, {type: labels}, {type: tf}, {type: detections}, "
                      "{type: mpc_path}, {type: preview_task}]");
}
void Composition::drawSidebar(float height) {
    ImGui::BeginChild("operator_sidebar", {sidebarWidth, height}, ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    for (auto &item : panelInstances) {
        ImGui::PushID(item.id.c_str());
        item.panel->pinned();
        ImGui::PopID();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8, ImGui::GetStyle().FramePadding.y));
    ImGui::BeginChild("panel_contents", {0, 0});
    bool first = true;
    for (auto &item : panelInstances) {
        ImGui::PushID(item.id.c_str());
        if (item.visible) {
            if (!first)
                ImGui::Spacing();
            first = false;
            item.open = robotics::ros_viewer::disclosureHeader(item.title.c_str(), item.open);
            if (item.open)
                item.panel->draw();
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
    ImGui::EndChild();
    syncOwnership();
}
void Composition::drawWindows() {
    for (auto &item : toolbarInstances) {
        ImGui::PushID(item.id.c_str());
        item.panel->drawWindows();
        ImGui::PopID();
    }
    for (auto &item : panelInstances) {
        ImGui::PushID(item.id.c_str());
        item.panel->drawWindows();
        ImGui::PopID();
    }
}
void Composition::drawOverlayControls(const std::string &provider) {
    for (auto &item : overlays) {
        if (item.provider != provider)
            continue;
        ImGui::PushID(item.id.c_str());
        if (ImGui::Checkbox(item.title.c_str(), &item.visible) && !item.visible)
            item.overlay->cancelInteraction();
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
} // namespace robotics::ros_viewer::panels
