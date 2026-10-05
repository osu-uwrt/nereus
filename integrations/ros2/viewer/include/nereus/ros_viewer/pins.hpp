// Controls pinned to the pool view toolbar: right-click a button, checkbox or dropdown in a panel and "Pin to
// toolbar". ImGui widgets cannot move, so each pinnable widget records itself as it is drawn (label, enabled,
// value, colours) and the toolbar draws a copy from that record; clicking the copy queues the action, which the
// original widget performs the next time its panel draws. A panel with pins that is closed or hidden behind a
// tab is drawn off screen so its pinned controls keep working.
#pragma once
#include <functional>
#include <imgui.h>
#include <string>
#include <vector>

namespace nereus::ros_viewer::pins {
// The window whose widgets are drawn next ("panel.<id>", "display", ...) and its title; outside a scope the
// widgets below are plain ImGui widgets.
void beginScope(const std::string &scope, const std::string &title);
void endScope();
// A nested part of the key for repeated widgets (per camera, per command); also pushes the ImGui ID.
struct Scope {
    explicit Scope(const std::string &part);
    ~Scope();
    Scope(const Scope &) = delete;
    Scope &operator=(const Scope &) = delete;
};

// Pinnable widgets: as ImGui's, plus right-click to pin. A label whose text changes ("Pause" / "Resume") keeps its
// pin with a fixed ID after "###".
bool Button(const char *label, const ImVec2 &size = ImVec2(0, 0));
bool Checkbox(const char *label, bool *value);
bool Combo(const char *label, int *current, const char *itemsSeparatedByZeros);

// Called once per frame by the host.
void newFrame();
void drawMenu(); // the right-click menu, at top level after all windows
// Whether a scope has pinned controls but did not draw this frame; drawOffscreen draws it unseen.
bool needsDrawing(const std::string &scope);
void drawOffscreen(const std::string &scope, const std::string &title, const std::function<void()> &body);
// The toolbar's copies of the pinned controls, in pin order.
void drawPinned();
// For the toolbar's customization popup: a checkbox per pinned control (untick to unpin).
void drawCustomization();
bool any();
void clear();

// Pins by key (scope/part/id), for saved layouts and tests.
std::vector<std::string> pinnedKeys();
void setPinned(const std::string &key, bool pinned);
// [Nereus][Pins] in the ImGui ini, so saved layouts carry the pinned controls. Once per ImGui context.
void install();
} // namespace nereus::ros_viewer::pins
