// Operator window layout: built-in dock arrangements, the open/closed state of every window kept in the ImGui
// ini next to the dock tree, and named layouts saved by the operator. ImGui only; no ROS or GL.
#pragma once
#include "nereus/ros_viewer/panels/composition.hpp"
#include <filesystem>
#include <functional>
#include <imgui.h>
#include <map>
#include <string>
#include <vector>

namespace nereus::ros_viewer {
using panels::Dock;

// A window a built-in layout places: its ImGui window name and dock area. Stacked windows that share an area
// split it evenly (camera feeds one above the other); the others share it as tabs, `selected` the one shown.
struct LayoutWindow {
    std::string name;
    Dock area = Dock::Left;
    bool stacked = false, selected = false;
};

// A built-in arrangement: column and strip sizes as fractions of the dock space, and areas it moves elsewhere.
struct LayoutPreset {
    std::string id, label, shortcut, description;
    float left = .24f, leftTop = .65f, right = .27f, rightBottom = .3f, bottom = .3f;
    std::map<Dock, Dock> remap;

    // The area a window assigned to `d` lands in under this preset.
    Dock area(Dock d) const {
        const auto found = remap.find(d);
        return found == remap.end() ? d : found->second;
    }
};

// The built-in presets.
const std::vector<LayoutPreset> &layoutPresets();
// The preset with that id, or null.
const LayoutPreset *findPreset(const std::string &id);

// Rebuilds the dock space `dockspace` of `size`: `center` takes the central node, every other window its preset
// area (each tabbed area opening on its selected window), floating ones are undocked. Call before
// DockSpace()/DockSpaceOverViewport() in a frame.
void buildLayout(ImGuiID dockspace, ImVec2 size, const LayoutPreset &, const std::string &center,
                 const std::vector<LayoutWindow> &);

// The dock space's sides: everything docked left / right of its central node (the pool view).
enum class Side { Left, Right };

// Width of the dock space beside the central node on that side (0 when nothing is shown there).
float sideWidth(ImGuiID dockspace, Side);
// Names of the windows shown docked on that side last frame (front or behind a tab).
std::vector<std::string> sideWindows(ImGuiID dockspace, Side);
// Resizes the side to `width` by moving its border with the central node; false if the side is not shown.
bool setSideWidth(ImGuiID dockspace, Side, float width);

// Whether a window was shown last frame and not covered by another tab of its dock node (a toolbar button for
// it then hides it rather than bringing it forward). Call before the window's Begin() in a frame.
bool windowInFront(const char *name);

// Open/closed state of the viewer's windows by stable key, kept in the ImGui ini as [Nereus][Windows] so a saved
// layout restores which windows are shown as well as where they are. States of windows that do not exist right
// now (a camera of another scenario) are kept and written back.
class WindowStates {
  public:
    // Each window's stable key and its open flag (owned by the caller), gathered fresh on every use.
    using Flags = std::vector<std::pair<std::string, bool *>>;

    explicit WindowStates(std::function<Flags()> flags) : flags_(std::move(flags)) {}
    WindowStates(const WindowStates &) = delete;
    WindowStates &operator=(const WindowStates &) = delete;

    // Registers the ini handler in the current ImGui context; this object must outlive the context's use of it.
    void install();
    // Saved states onto the current flags (call again when windows appear, e.g. after the scenario arrives).
    void apply();
    // Marks the ini settings dirty when a flag changed since the last call.
    void update();
    // Whether the ini held a state for this key.
    bool saved(const std::string &key) const {
        return saved_.count(key) > 0;
    }

  private:
    std::function<Flags()> flags_;
    std::map<std::string, bool> saved_; // states read from (and written back to) the ini
    std::string signature_;             // the flags as of the last apply()/update(), to detect changes
};

// $XDG_CONFIG_HOME/nereus, else ~/.config/nereus (empty when neither is set).
std::filesystem::path configDirectory();

// A file name for a layout name: letters, digits, space, '-' and '_' kept, the rest dropped; empty if none left.
std::string layoutFileStem(const std::string &name);
// Names of the saved layouts (<dir>/*.ini), sorted.
std::vector<std::string> savedLayouts(const std::filesystem::path &dir);
} // namespace nereus::ros_viewer
