#include "robotics/ros_viewer/panels/composition.hpp"
#include <imgui.h>
#include <cassert>
#include <functional>
#include <iostream>
using namespace robotics::ros_viewer::panels;
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
        r.panels.emplace(type, ViewFactory<Panel>{Kind::Motion,
                                                  [type](const YAML::Node &o) {
                                                      keys(o, {"label"}, type);
                                                  },
                                                  [&, type](const Binding &) {
                                                      return std::unique_ptr<Panel>(new HostItem(type, &drawLog));
                                                  },
                                                  true, std::string(type) != "detections"});
    int created = 0;
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
    const auto text = R"(schema_version: 1
providers:
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
    assert(good.sidebarVisible());
    assert(std::abs(good.width(1000) - 290) < .01f);
    assert(std::abs(good.width(1500) - 435) < .01f);
    good.setWidth(400, true);
    assert(good.width(1500) == 400);
    good.toggleSidebar();
    assert(!good.sidebarVisible());
    good.toggleSidebar();
    auto motion = std::dynamic_pointer_cast<Motion>(good.providers().at("motion"));
    auto mission = std::dynamic_pointer_cast<Autonomy>(good.providers().at("mission"));
    mission->start("test");
    good.touch();
    assert(motion->state().blocked);
    mission->stop();
    good.touch();
    assert(!motion->state().blocked);
    ctx.preview = true;
    Composition preview(YAML::Load(text), ctx, r);
    assert(created == 2 && preview.providers().empty());
    Composition empty(YAML::Load("schema_version: 1\nproviders: {}"), ctx, r);
    assert(empty.empty());
    ctx.preview = false;
    auto fails = [&](const YAML::Node &cfg) {
        const int before = created; // a rejected composition creates no provider
        bool threw = false;
        try {
            Composition bad(cfg, ctx, r);
        } catch (const std::exception &) {
            threw = true;
        }
        if (!threw || created != before)
            std::cerr << "expected rejection (threw " << threw << ", providers created " << created - before
                      << "):\n" << YAML::Dump(cfg) << "\n";
        assert(threw && created == before);
    };
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
    cfg["overlays"][0]["options"]["size_metres"] = -5;
    fails(cfg);
    cfg = YAML::Load(text);
    cfg["schema_version"] = 2;
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
        Composition none(YAML::Load("schema_version: 1\nproviders: {}\ntoolbar: []"), ctx, r);
        assert(none.toolbarIds().empty());
    }
    {
        // A host item can also be a sidebar panel (draw()), but not a toolbar-only one.
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
    const auto withToolbar = [&](const char *yaml) {
        auto c = YAML::Load(text);
        c["toolbar"] = YAML::Load(yaml);
        return c;
    };
    fails(withToolbar("[{type: nope}]"));                                   // unknown type
    fails(withToolbar("[{type: view, colour: red}]"));                      // unknown key
    fails(withToolbar("[{type: view, options: {typo: 1}}]"));               // unknown option
    fails(withToolbar("[{type: view, provider: motion}]"));                 // host item takes no provider
    fails(withToolbar("[{type: motion}]"));                                 // provider-backed needs a provider
    fails(withToolbar("[{type: motion, provider: mission}]"));              // capability mismatch
    fails(withToolbar("[{type: motion, provider: missing}]"));              // unknown provider
    fails(withToolbar("[{type: view}, {type: view}]"));                     // duplicate (default) ID
    fails(withToolbar("[{id: a, type: view}, {id: a, type: scene_settings}]"));
    fails(withToolbar("{type: view}"));                                     // not a sequence
    fails(withToolbar("[{id: x}]"));                                        // missing type
    fails(withToolbar("[{type: view, slot: settings}]"));                   // slot is gone
    try {
        Composition bad(withToolbar("[{type: nope}]"), ctx, r);
        assert(false);
    } catch (const std::exception &e) {
        const std::string message = e.what();
        assert(message.find("toolbar.nope") != std::string::npos && message.find("unknown toolbar item type") != std::string::npos);
    }
    cfg = YAML::Load(text);
    cfg["panels"][0]["slot"] = "settings";
    fails(cfg);
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
