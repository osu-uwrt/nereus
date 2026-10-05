// Compact status chips for header and toolbar forms (telemetry readings, recording indicators).
#pragma once
#include "nereus/ros_viewer/panels/capabilities.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <imgui.h>

namespace nereus::ros_viewer::panels {
inline ImVec4 levelColor(Level level) {
    const auto &p = palette();
    switch (level) {
    case Level::Ok:
        return p.accent;
    case Level::Warn:
        return p.warn;
    case Level::Error:
        return p.error;
    default:
        return p.muted;
    }
}
inline const char *levelName(Level level) {
    switch (level) {
    case Level::Ok:
        return "OK";
    case Level::Warn:
        return "Warning";
    case Level::Error:
        return "Error";
    default:
        return "No data";
    }
}
// statusChip() is theme.hpp's outlined status capsule (the same as the viewer's status pill).
inline float chipWidth(const char *text) {
    return statusChipWidth(text);
}
} // namespace nereus::ros_viewer::panels
