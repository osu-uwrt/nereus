#pragma once
#include "nereus/ros_viewer/panels/capabilities.hpp"
#include <functional>
#include <map>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::panels {
struct Context {
    std::string robotNamespace, fixedFrame;
    bool preview = false, useSimTime = false;
    // Immutable host documents and optional navigation; no robot/task types.
    std::map<std::string, YAML::Node> documents{};
    std::function<void(const std::string &)> focus{};
    std::vector<std::string> initialWindows{};
};
struct Viewport {
    glm::mat4 projection{1}, view{1};
    glm::vec3 eye{0};
    glm::vec2 origin{0}, size{1};
    bool interactive = false, focused = true;
    // Where motion commands are drawn: display = displayFromCommand * command (identity: the command
    // frame itself). The host re-roots them, e.g. at the simulator truth robot instead of the estimate.
    // Overlays hold the offset per command (captured when the command changes), so a fixed command
    // stays still while the offset carries estimation noise; large jumps are taken at once.
    glm::mat4 displayFromCommand{1};
};
struct Panel {
    virtual ~Panel() = default;
    virtual void toolbar() {} // compact form in the pool view's toolbar (composition `toolbar:`)
    virtual void pinned() {}  // critical controls in the always-visible command bar, whatever the window layout
    virtual void header() {   // compact form in the always-visible header row (composition `header:`)
        toolbar();
    }
    virtual void draw() = 0;      // the panel window's contents (composition `panels:`)
    virtual void drawWindows() {} // tool windows the panel opens itself (e.g. a scorecard)
    virtual void windowMenu() {}  // ImGui::MenuItem per tool window of drawWindows(), for the Windows menu
};
struct Overlay {
    virtual ~Overlay() = default;
    virtual void cancelInteraction() {}
    virtual bool input(const Viewport &) = 0;
    virtual void draw(const Viewport &) = 0;
};
using Providers = std::map<std::string, std::shared_ptr<Provider>>;
struct Binding {
    std::shared_ptr<Provider> provider;
    YAML::Node options;
    std::function<bool()> mayStart;
    std::function<void()> kill;
    std::map<std::string, YAML::Node> documents{};
    std::function<void(const std::string &)> focus{};
    bool showWindow = false;
    std::function<void()> drawOverlayControls{};
    std::function<void()> drawPanelMenu{}; // the Windows popup (toolbar item "panels_menu")
};
struct ProviderFactory {
    Kind kind;
    std::function<void(const YAML::Node &)> validate;
    std::function<std::shared_ptr<Provider>(const YAML::Node &, const Context &)> create;
};
template <class T> struct ViewFactory {
    Kind kind;
    std::function<void(const YAML::Node &)> validate;
    std::function<std::unique_ptr<T>(const Binding &)> create;
    // hosted: provided by the host application (no provider; `provider` is rejected). toolbarOnly: never a
    // panel window (its draw() has no window form).
    bool hosted = false, toolbarOnly = false;
};
struct Registry {
    std::map<std::string, ProviderFactory> providers;
    std::map<std::string, ViewFactory<Panel>> panels;
    std::map<std::string, ViewFactory<Overlay>> overlays;
};
void keys(const YAML::Node &, std::initializer_list<const char *> allowed, const std::string &where);
void required(const YAML::Node &, std::initializer_list<const char *> names);
void positive(const YAML::Node &, const char *key, double fallback, double maximum = 60.);
std::string expand(std::string value, const Context &context);

// Where a panel window goes in the built-in layouts (`dock:`); the operator can move it anywhere afterwards.
enum class Dock { LeftTop, Left, Right, RightBottom, Bottom, Floating };
Dock parseDock(const std::string &);
class Composition {
  public:
    Composition(const YAML::Node &, const Context &, const Registry &);
    void touch();
    // Every panel's pinned() on the current line (the command bar), visible or not.
    void drawPinned();
    // Each visible panel as a dockable window named panelWindowName().
    void drawPanels();
    // Draws the configured `toolbar:` items in order (the default list when the key is absent).
    void drawToolbar();
    // Draws the configured `header:` items on the current line, right-aligned to end at window x `right`
    // (never left of the previous item).
    void drawHeader(float right);
    // Instance IDs of the toolbar / panel windows in display order.
    std::vector<std::string> toolbarIds() const;
    std::vector<std::string> panelIds() const;
    void drawWindows();
    // Windows menu items: one per panel window, and the panels' own tool windows.
    void drawPanelMenuItems();
    void drawToolMenuItems();
    // The host's complete Windows menu, shown by the `panels_menu` toolbar item (panel items alone if unset).
    void setWindowMenu(std::function<void()> menu) {
        windowMenu = std::move(menu);
    }
    // Called right after each visible panel window's Begin() with the panel's ID: the host's right-click menu
    // on the window's tab or title bar (ImGui::BeginPopupContextItem).
    void setWindowContextMenu(std::function<void(const std::string &)> menu) {
        windowContextMenu = std::move(menu);
    }
    // Toolbar items for the operator's toolbar customization: `title` is the configured title (empty if none),
    // `visible` whether the item is shown.
    struct ToolbarItem {
        std::string id, type, title;
        bool *visible;
    };
    std::vector<ToolbarItem> toolbarItems();
    struct PanelWindow {
        std::string id, name; // instance ID, ImGui window name
        Dock dock;
        bool selected; // `open`: the shown tab of its dock area in a built-in layout
    };
    std::vector<PanelWindow> panelWindows() const;
    // Open state of each panel window by stable key ("panel.<id>"), for saved layouts.
    std::vector<std::pair<std::string, bool *>> visibility();
    void focusPanel(const std::string &id);
    bool input(const Viewport &);
    void drawOverlays(const Viewport &);
    // Left dock column width in the built-in layout (`sidebar_width` or `sidebar_width_fraction` of the window).
    float width(float windowWidth = 0) const {
        return windowWidth > 0 && sidebarFraction > 0 ? windowWidth * sidebarFraction : sidebarWidth;
    }
    bool empty() const {
        return panelInstances.empty();
    }
    const Providers &providers() const {
        return sources;
    }

  private:
    struct PanelInstance {
        std::string id, title;
        bool visible, open;
        std::unique_ptr<Panel> panel;
        Dock dock = Dock::Left;
        bool focus = false;
        std::string type, configuredTitle;
    };
    struct Ownership {
        std::shared_ptr<Motion> motion;
        std::shared_ptr<Autonomy> mission;
    };
    void syncOwnership();
    void drawOverlayControls(const std::string &provider);
    static std::string toolTitle(const PanelInstance &);
    void drawPanelMenu();
    float sidebarWidth = 350, sidebarFraction = .29f;
    Providers sources;
    std::vector<PanelInstance> panelInstances, toolbarInstances, headerInstances;
    float headerWidth = 0; // last frame's, for right alignment
    struct OverlayInstance {
        std::string id, title;
        bool visible;
        std::unique_ptr<Overlay> overlay;
        std::string provider;
    };
    std::vector<OverlayInstance> overlays;
    std::vector<Ownership> ownership;
    std::function<void()> windowMenu;
    std::function<void(const std::string &)> windowContextMenu;
};
// ImGui window name of a panel instance: the title shown, the instance ID as the stable identity.
std::string panelWindowName(const std::string &id, const std::string &title);
// The built-in toolbar item used when a composition has no `toolbar:` list: the host registers the items it
// provides (types not in the registry are skipped).
YAML::Node defaultToolbar();
// Host-application items (drawn by the viewer, no provider). hostItemTypes() is the canonical list with
// whether each can also be a panel window; registerHostItem binds one to drawing functions (toolbar form,
// optional window form). registerHostPlaceholders registers the whole list with no drawing, so tools and
// tests validate compositions exactly like the viewer.
struct HostItemType {
    const char *type;
    bool sidebar;
};
const std::vector<HostItemType> &hostItemTypes();
void registerHostItem(Registry &, const std::string &type, std::function<void()> toolbar,
                      std::function<void()> panel = {});
void registerHostPlaceholders(Registry &);
void registerPanels(Registry &);
} // namespace nereus::ros_viewer::panels
