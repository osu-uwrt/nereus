// The panel composition built from the viewer YAML: provider bindings, validation (a bad config creates nothing),
// preview passivity, autonomy owning motion, panel windows and dock placement, the toolbar and header, themes.
#include "nereus/ros_viewer/panels/composition.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <cassert>
#include <functional>
#include <imgui.h>
#include <imgui_internal.h>
#include <iostream>
using namespace nereus::ros_viewer::panels;

// In-memory Motion provider: enable/kill/activate/drag/block just update its state.
struct FakeMotion : Motion {
    MotionState s;
    MotionState state() override {
        return s;
    }
    void enable() override {
        s.enabled = true;
    }
    void kill() override {
        s.enabled = false;
    }
    void activate(Mode m, const Pose &p) override {
        s.mode = m;
        s.commanded = p;
    }
    void drag(const Pose &p) override {
        s.commanded = p;
    }
    void block(bool b) override {
        s.blocked = b;
    }
};

// In-memory Autonomy provider: busy between start() and stop().
struct FakeMission : Autonomy {
    MissionState s;
    MissionState state() override {
        return s;
    }
    void refresh() override {}
    void start(const std::string &) override {
        s.busy = true;
    }
    void stop() override {
        s.busy = false;
    }
};

// Host items: no provider, recorded when drawn so the toolbar order can be observed.
struct HostItem final : Panel {
    std::string name;
    std::vector<std::string> *log;
    HostItem(std::string n, std::vector<std::string> *l) : name(std::move(n)), log(l) {}
    void toolbar() override {
        log->push_back("toolbar:" + name);
    }
    void draw() override {
        log->push_back("draw:" + name);
    }
};

int main() {
    Registry r;
    registerPanels(r);
    std::vector<std::string> drawLog; // this test registers its own recording host items
    for (const char *type : {"view", "detections", "scene_settings"})
        r.panels.emplace(type,
                         ViewFactory<Panel>{Kind::Motion, [type](const YAML::Node &o) { keys(o, {"label"}, type); },
                                            [&, type](const Binding &) {
                                                return std::unique_ptr<Panel>(new HostItem(type, &drawLog));
                                            },
                                            true, std::string(type) != "detections"});

    // Fake provider types; `created` counts providers built, `telemetryReads` the telemetry state() calls.
    int created = 0, telemetryReads = 0;
    struct FakeTelemetry : Telemetry {
        int *reads;
        explicit FakeTelemetry(int *r) : reads(r) {}
        TelemetryState state() override {
            ++*reads;
            return {{{"fog", "FOG", "40.0\u00B0C", "detail", Level::Ok}}};
        }
    };
    r.providers.emplace("fake.telemetry",
                        ProviderFactory{Kind::Telemetry, [](auto) {},
                                        [&](auto, auto) { return std::make_shared<FakeTelemetry>(&telemetryReads); }});
    r.providers.emplace("fake.motion", ProviderFactory{Kind::Motion, [](auto) {},
                                                       [&](auto, auto) {
                                                           ++created;
                                                           return std::make_shared<FakeMotion>();
                                                       }});
    r.providers.emplace("fake.mission", ProviderFactory{Kind::Autonomy, [](auto) {},
                                                        [&](auto, auto) {
                                                            ++created;
                                                            return std::make_shared<FakeMission>();
                                                        }});

    // The base config: a motion panel, an autonomy panel, a gizmo overlay, and autonomy owning motion.
    const auto text = R"(providers:
  motion: {type: fake.motion, options: {}}
  mission: {type: fake.mission, options: {}}
panels:
  - {id: control, type: motion, provider: motion}
  - {id: autonomy, type: autonomy, provider: mission}
overlays:
  - {id: target, type: pose_gizmo, provider: motion}
ownership:
  - {motion: motion, autonomy: mission}
)";

    Context ctx{"some_robot", "some_frame", false, false};
    Composition good(YAML::Load(text), ctx, r);
    assert(created == 2);

    // Left dock column of the built-in layout: a fraction of the window (default .29) or sidebar_width pixels.
    assert(std::abs(good.width(1000) - 290) < .01f);
    assert(std::abs(good.width(1500) - 435) < .01f);

    // Panels are dockable windows: the title shown, the instance ID as the window's identity in saved layouts.
    {
        const Context ctx{"some_robot", "some_frame", true, false}; // preview: these create no providers
        auto fixedWidth = YAML::Load(text);
        fixedWidth["sidebar_width"] = 400;
        assert(Composition(fixedWidth, ctx, r).width(1500) == 400);
        const auto windows = good.panelWindows();
        assert(windows.size() == 2 && windows[0].id == "control" && windows[0].name == "control###panel.control");
        assert(windows[0].dock == Dock::Left && windows[0].selected);
        auto docked = YAML::Load(text);
        docked["panels"][0]["dock"] = "left_top";
        docked["panels"][1]["open"] = false;
        docked["panels"][1]["title"] = "Mission";
        Composition c(docked, ctx, r);
        const auto placed = c.panelWindows();
        assert(placed[0].dock == Dock::LeftTop && placed[1].name == "Mission###panel.autonomy" && !placed[1].selected);
        auto flags = c.visibility();
        assert(flags.size() == 2 && flags[0].first == "panel.control" && *flags[0].second);
        auto hidden = YAML::Load(text);
        hidden["sidebar_visible"] = false; // panel windows start closed
        Composition closed(hidden, ctx, r);
        for (const auto &flag : closed.visibility())
            assert(!*flag.second);
        assert(parseDock("right_bottom") == Dock::RightBottom && parseDock("floating") == Dock::Floating);
    }

    // Ownership: motion is blocked while the mission runs.
    auto motion = std::dynamic_pointer_cast<Motion>(good.providers().at("motion"));
    auto mission = std::dynamic_pointer_cast<Autonomy>(good.providers().at("mission"));
    mission->start("test");
    good.touch();
    assert(motion->state().blocked);
    mission->stop();
    good.touch();
    assert(!motion->state().blocked);

    // Preview compositions create no providers; a config without panels is empty.
    ctx.preview = true;
    Composition preview(YAML::Load(text), ctx, r);
    assert(created == 2 && preview.providers().empty());
    Composition empty(YAML::Load("providers: {}"), ctx, r);
    assert(empty.empty());
    ctx.preview = false;

    // Asserts that `cfg` is rejected without creating any provider.
    auto fails = [&](const YAML::Node &cfg) {
        const int before = created; // a rejected composition creates no provider
        bool threw = false;
        try {
            Composition bad(cfg, ctx, r);
        } catch (const std::exception &) {
            threw = true;
        }
        if (!threw || created != before)
            std::cerr << "expected rejection (threw " << threw << ", providers created " << created - before << "):\n"
                      << YAML::Dump(cfg) << "\n";
        assert(threw && created == before);
    };

    // Rejected: unknown or mismatched providers, duplicate IDs, out-of-range or conflicting sidebar widths,
    // unknown provider types / dock areas / keys, invalid overlay options.
    auto cfg = YAML::Load(text);
    cfg["panels"][0]["provider"] = "missing";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][0]["provider"] = "mission";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][1]["id"] = "control";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["sidebar_width"] = 10;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["sidebar_width_fraction"] = 1;
    fails(cfg);
    cfg["sidebar_width_fraction"] = .29;
    cfg["sidebar_width"] = 350;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["providers"]["motion"]["type"] = "missing";
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["panels"][0]["dock"] = "middle"; // unknown dock area
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["overlays"][0]["options"]["size_meters"] = -5;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["typo"] = true;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["tools"] = YAML::Load("[{id: tool, type: motion, provider: motion}]");
    fails(cfg); // the old `tools:` list is now `toolbar:`

    // --- toolbar: parsing, ordering, validation
    const auto toolbarText = std::string(text) + R"(toolbar:
  - {type: view}
  - {id: pm, type: panels_menu}
  - {id: mine, type: motion, provider: motion}
  - {type: scene_settings, options: {label: x}}
)";
    {
        Composition c(YAML::Load(toolbarText), ctx, r);
        const std::vector<std::string> order{"view", "pm", "mine", "scene_settings"};
        assert(c.toolbarIds() == order);
        ImGui::CreateContext();
        auto &io = ImGui::GetIO();
        io.IniFilename = nullptr;
        io.DisplaySize = {800, 600};
        io.DeltaTime = 1.f / 30;
        unsigned char *pixels;
        int w, h;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
        ImGui::NewFrame();
        ImGui::Begin("t");
        drawLog.clear();
        c.drawToolbar();
        ImGui::End();
        ImGui::Render();
        const std::vector<std::string> drawn{"toolbar:view", "toolbar:scene_settings"};
        assert(drawLog == drawn); // panels_menu / motion draw their own widgets, not through the log

        // The operator's toolbar customization: every item with its type, configured title and shown flag.
        {
            auto titled = YAML::Load(toolbarText);
            titled["toolbar"][2]["title"] = "Drive";
            Composition c(titled, ctx, r);
            auto items = c.toolbarItems();
            assert(items.size() == 4 && items[0].id == "view" && items[0].type == "view" && items[0].title.empty());
            assert(items[2].id == "mine" && items[2].type == "motion" && items[2].title == "Drive");
            *items[0].visible = false; // hidden from the toolbar like `visible: false`
            ImGui::NewFrame();
            ImGui::Begin("t");
            drawLog.clear();
            c.drawToolbar();
            ImGui::End();
            ImGui::Render();
            assert(drawLog == std::vector<std::string>{"toolbar:scene_settings"});
        }

        // Themes restyle everything; unknown names change nothing. Every shipped theme file loads.
        assert(nereus::ros_viewer::loadThemes(NEREUS_VIEWER_THEMES).empty());
        assert(nereus::ros_viewer::themes().size() >= 12);
        for (const auto &theme : nereus::ros_viewer::themes()) {
            assert(nereus::ros_viewer::applyTheme(theme.id) && nereus::ros_viewer::currentTheme() == theme.id);
            assert(ImGui::GetStyle().WindowMenuButtonPosition == ImGuiDir_None);
            assert(ImGui::GetStyle().TabCloseButtonMinWidthSelected == 0); // tab close box only on hover
            assert(ImGui::GetStyle().Colors[ImGuiCol_Text].x == nereus::ros_viewer::palette().text.x);
        }
        assert(nereus::ros_viewer::applyTheme("daylight"));
        const float daylightText = nereus::ros_viewer::palette().text.x;
        assert(!nereus::ros_viewer::applyTheme("nope") && nereus::ros_viewer::currentTheme() == "daylight");
        assert(nereus::ros_viewer::applyTheme("abyss") && nereus::ros_viewer::palette().text.x > daylightText);

        // Panel windows: each visible panel is its own window; closing it (Windows menu, its x) hides it.
        Composition windows(YAML::Load(toolbarText), ctx, r);
        const auto frame = [&] {
            ImGui::NewFrame();
            ImGui::Begin("t");
            windows.drawPinned();
            ImGui::End();
            windows.drawPanels();
            ImGui::Render();
        };
        frame();
        const auto active = [](const char *name) {
            const auto *window = ImGui::FindWindowByName(name);
            return window && window->Active;
        };
        assert(active("control###panel.control") && active("autonomy###panel.autonomy"));
        *windows.visibility()[1].second = false;
        frame();
        frame();
        assert(active("control###panel.control") && !active("autonomy###panel.autonomy"));

        // The host's right-click menu runs right after each visible panel window's Begin, with the panel's ID.
        std::vector<std::string> menus;
        windows.setWindowContextMenu([&](const std::string &id) { menus.push_back(id); });
        frame();
        assert(menus == std::vector<std::string>{"control"});
        windows.focusPanel("autonomy"); // reopens it on top
        frame();
        frame();
        assert(active("autonomy###panel.autonomy") && *windows.visibility()[1].second);

        // visible: false hides an item without dropping it
        auto hidden = YAML::Load(toolbarText);
        hidden["toolbar"][0]["visible"] = false;
        Composition h2(hidden, ctx, r);
        ImGui::NewFrame();
        ImGui::Begin("t");
        drawLog.clear();
        h2.drawToolbar();
        ImGui::End();
        ImGui::Render();
        const std::vector<std::string> drawnHidden{"toolbar:scene_settings"};
        assert(drawLog == drawnHidden);
        ImGui::DestroyContext();
    }
    {
        // No `toolbar:` key: the default list, restricted to the item types this host registered.
        Composition c(YAML::Load(text), ctx, r);
        const std::vector<std::string> order{"scene_settings", "panels_menu", "view"};
        assert(c.toolbarIds() == order);
        Composition none(YAML::Load("providers: {}\ntoolbar: []"), ctx, r);
        assert(none.toolbarIds().empty());
    }
    {
        // A host item can also be a panel window (draw()), but not a toolbar-only one.
        auto c2 = YAML::Load(text);
        c2["panels"].push_back(YAML::Load("{id: det, type: detections, title: Detections}"));
        c2["toolbar"] = YAML::Load("[{type: detections}, {id: det2, type: detections}]");
        Composition c(c2, ctx, r);
        const std::vector<std::string> order{"detections", "det2"};
        assert(c.toolbarIds() == order);
        assert(c.panelIds().back() == "det");
        auto bad = YAML::Load(text);
        bad["panels"].push_back(YAML::Load("{id: v, type: view}"));
        fails(bad);
    }
    {
        // header: toolbar items drawn in the header row, in order (Panel::header() defaults to toolbar()).
        auto c3 = YAML::Load(text);
        c3["header"] = YAML::Load("[{type: view}, {id: s, type: scene_settings}, {id: t, type: telemetry, "
                                  "provider: telemetry}]");
        c3["providers"]["telemetry"] = YAML::Load("{type: fake.telemetry, options: {}}");
        Composition c(c3, ctx, r);
        ImGui::CreateContext();
        auto &io = ImGui::GetIO();
        io.IniFilename = nullptr;
        io.DisplaySize = {800, 600};
        io.DeltaTime = 1.f / 30;
        unsigned char *pixels;
        int w, h;
        io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
        for (int frame = 0; frame < 2; ++frame) {
            ImGui::NewFrame();
            ImGui::Begin("t");
            drawLog.clear();
            ImGui::TextUnformatted("title");
            c.drawHeader(700);
            ImGui::End();
            ImGui::Render();
        }
        const std::vector<std::string> drawn{"toolbar:view", "toolbar:scene_settings"};
        assert(drawLog == drawn && telemetryReads == 2);
        ImGui::DestroyContext();
        auto bad = YAML::Load(text);
        bad["header"] = YAML::Load("[{type: motion, provider: mission}]");
        fails(bad);
        bad["header"] = YAML::Load("[{type: nope}]");
        fails(bad);
        bad["header"] = YAML::Load("{type: view}");
        fails(bad);
    }

    // SVO names: ~ is the robot's home, the camera id keeps cameras apart, the extension is kept or .svo2.
    assert(recordingFile("~/svos/run", "ffc", "", "/home/ros") == "/home/ros/svos/run_ffc.svo2");
    assert(recordingFile("/data/gate.svo", "dfc", "20261002_101500", "/home/ros") ==
           "/data/gate_20261002_101500_dfc.svo");
    assert(recordingFile("~other/x.svo2", "ffc", "", "/h") == "~other/x_ffc.svo2");

    // Toolbar validation: the base config with this `toolbar:` list.
    const auto withToolbar = [&](const char *yaml) {
        auto c = YAML::Load(text);
        c["toolbar"] = YAML::Load(yaml);
        return c;
    };
    fails(withToolbar("[{type: nope}]"));                      // unknown type
    fails(withToolbar("[{type: view, color: red}]"));          // unknown key
    fails(withToolbar("[{type: view, options: {typo: 1}}]"));  // unknown option
    fails(withToolbar("[{type: view, provider: motion}]"));    // host item takes no provider
    fails(withToolbar("[{type: motion}]"));                    // provider-backed needs a provider
    fails(withToolbar("[{type: motion, provider: mission}]")); // capability mismatch
    fails(withToolbar("[{type: motion, provider: missing}]")); // unknown provider
    fails(withToolbar("[{type: view}, {type: view}]"));        // duplicate (default) ID
    fails(withToolbar("[{id: a, type: view}, {id: a, type: scene_settings}]"));
    fails(withToolbar("{type: view}"));                   // not a sequence
    fails(withToolbar("[{id: x}]"));                      // missing type
    fails(withToolbar("[{type: view, slot: settings}]")); // slot is gone

    // The error names the offending item and the problem.
    try {
        Composition bad(withToolbar("[{type: nope}]"), ctx, r);
        assert(false);
    } catch (const std::exception &e) {
        const std::string message = e.what();
        assert(message.find("toolbar.nope") != std::string::npos &&
               message.find("unknown toolbar item type") != std::string::npos);
    }
    cfg = YAML::Load(text);
    cfg["panels"][0]["slot"] = "settings";
    fails(cfg);
    fails(withToolbar("[{type: view, dock: left}]")); // dock places panel windows only

    // Topic templates expand context placeholders; unknown placeholders are errors.
    assert(expand("/{namespace}/{fixed_frame}", ctx) == "/some_robot/some_frame");
    bool threw = false;
    try {
        expand("{typo}", ctx);
    } catch (const std::exception &) {
        threw = true;
    }
    assert(threw);
    std::cout << "PASS: composition bindings, validation, preview passivity, ownership, and empty layout\n";
}
