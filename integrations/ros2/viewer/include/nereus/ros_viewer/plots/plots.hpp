// Live plots of ROS topics and panel figures: plot windows (lanes sharing one timeline, a results table per lane),
// the Topics browser, saved plots, the panels' "Plot this" menus and hover trends. Process-wide, like the pins:
// the host app installs and configures it; panels call the hooks, which do nothing until it is configured.
#pragma once
#include "nereus/ros_viewer/panels/composition.hpp"
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace nereus::ros_viewer::plots {

// --- Panel hooks ---

// The tooltip of a figure a panel shows: `title`, the figure's last 30 s with its min and max, `detail`, and
// "Right-click to plot". Call while the figure is hovered. Without plots it is a plain tooltip of title and detail.
void figureTooltip(const std::string &figure, const std::string &title, const std::string &detail = {});
// The right-click menu items for a group of figures ("motion.z", "telemetry.fog"): Plot, Add to ▸ (and, for a
// Motion axis, Plot all six axes). Call inside an open popup; nothing for an unknown group.
void figureMenuItems(const std::string &group);

// --- The host application's side ---

struct Options {
    std::string robotNamespace;
    bool useSimTime = false;
    std::filesystem::path savedDir;  // saved plots (one YAML file each)
    std::filesystem::path exportDir; // CSV exports of a plot's visible span
};

// [NereusPlots] in the ImGui ini: the plot windows (definitions, not data) and the shared timeline's span. Once per
// ImGui context, before the ini is read.
void install();
// Starts the plots' ROS node once the robot namespace is known. The providers supply the panels' figures (Motion
// axes, telemetry readings). Calling it again (another scenario) restarts the node.
void configure(const Options &, const panels::Providers &);
// Stops the node; call before rclcpp shuts down.
void shutdown();
// Once per frame before any window: samples the figures and keeps the ROS side tidy.
void frame();
// The plot windows and the Topics browser (dock windows).
void drawWindows();

// The windows plots adds: each plot window ("plot.<id>") and the Topics browser ("topics").
struct Window {
    std::string key, name, label;
    bool *open;
    bool plot = true; // false: the Topics browser
};
std::vector<Window> windows();
// Windows › Plots: the plots (ticked when shown), the saved plots, New plot, Manage saved plots…
void drawMenu();

// Ctrl+P entries: Plot <topic field>, Plot <panel figure>, Open plot <saved>, New plot. Running one with Shift held
// adds to the focused plot instead of opening a new one.
struct Command {
    std::string group, label;
    std::function<void()> run;
};
std::vector<Command> commands();

} // namespace nereus::ros_viewer::plots
