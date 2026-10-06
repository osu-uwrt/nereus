// Screen-space overlays (detections, MPC and planned paths, thrust arrows, TF axes) and the TF tree table.
#include "overlay_draw.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <imgui_internal.h>

namespace nereus::ros_viewer::host {
// Clip space -> NDC -> pixels (ImGui y grows downwards).
bool projectToScreen(const glm::mat4 &vp, const ScreenRect &rect, const glm::vec4 &world, ImVec2 &pixel) {
    const auto clip = vp * world;
    if (clip.w <= 0 || clip.z < -clip.w || clip.z > clip.w)
        return false;
    pixel = {rect.position.x + (clip.x / clip.w * .5f + .5f) * rect.width,
             rect.position.y + (.5f - clip.y / clip.w * .5f) * rect.height};
    return true;
}

ImU32 rgba(float r, float g, float b, float a) {
    return ImGui::ColorConvertFloat4ToU32({r, g, b, a});
}

namespace {
// A 6 px dash / 4 px gap line from a to b.
void dashedLine(ImDrawList *draw, ImVec2 a, ImVec2 b, ImU32 color, float thickness) {
    const float dx = b.x - a.x, dy = b.y - a.y, length = std::sqrt(dx * dx + dy * dy);
    constexpr float dash = 6.f, gap = 4.f;
    for (float t = 0; t < length; t += dash + gap) {
        const float t1 = std::min(t + dash, length);
        draw->AddLine({a.x + dx * t / length, a.y + dy * t / length}, {a.x + dx * t1 / length, a.y + dy * t1 / length},
                      color, thickness);
    }
}
} // namespace

void drawDetections(const std::vector<PlacedDetection> &detections, const glm::mat4 &vp, const ScreenRect &rect,
                    bool both) {
    using visualization_msgs::msg::Marker;
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(rect.position, {rect.position.x + rect.width, rect.position.y + rect.height}, true);
    bool haveApprox = false;

    // Each detection: pick its tint from the placement kind, then draw by marker type.
    for (const auto &placed : detections) {
        const auto &m = placed.marker;
        using Kind = PlacedDetection::Kind;
        const bool approx = placed.kind == Kind::EstimateApprox;
        const bool outlined = both && placed.kind != Kind::Truth; // estimate next to truth: cyan, unfilled
        haveApprox |= approx;
        const float alpha = m.color.a * (approx ? .45f : 1.f);
        const ImU32 tint = outlined ? rgba(.25f, .9f, 1.f, std::max(alpha, approx ? .45f : .9f))
                                    : rgba(m.color.r, m.color.g, m.color.b, alpha);

        if (m.type == Marker::CUBE) {
            // Flat rectangle in the marker's XY plane (scale.x by scale.y), skipped unless all corners project.
            const float hx = float(m.scale.x) / 2, hy = float(m.scale.y) / 2;
            const glm::vec4 corners[] = {{-hx, -hy, 0, 1}, {hx, -hy, 0, 1}, {hx, hy, 0, 1}, {-hx, hy, 0, 1}};
            ImVec2 pixels[4];
            bool visible = true;
            for (int i = 0; i < 4 && visible; ++i)
                visible = projectToScreen(vp, rect, placed.pose * corners[i], pixels[i]);
            if (!visible)
                continue;
            if (!outlined)
                draw->AddConvexPolyFilled(pixels, 4, tint);
            if (approx)
                for (int i = 0; i < 4; ++i)
                    dashedLine(draw, pixels[i], pixels[(i + 1) % 4], tint, 1.5f);
            else
                draw->AddPolyline(pixels, 4, tint, ImDrawFlags_Closed, outlined ? 2.f : 1.5f);
        } else if (m.type == Marker::ARROW) {
            // Pose+scale arrow along the marker's +X, drawn like rviz: a shaft ending at 77% of the length and
            // a head over the final 23%. The head base is projected in 3D so it foreshortens with the view;
            // its wings are spread in screen space so the head stays readable at any depth.
            ImVec2 tail, neck, tip;
            if (!projectToScreen(vp, rect, placed.pose[3], tail) ||
                !projectToScreen(vp, rect, placed.pose * glm::vec4(float(m.scale.x) * .77f, 0, 0, 1), neck) ||
                !projectToScreen(vp, rect, placed.pose * glm::vec4(float(m.scale.x), 0, 0, 1), tip))
                continue;

            const float thickness = 1.5f;
            if (approx)
                dashedLine(draw, tail, neck, tint, thickness);
            else
                draw->AddLine(tail, neck, tint, thickness);

            // Head: a dot when it projects to nothing, else a triangle 3..12 px half-wide.
            const ImVec2 axis{tip.x - neck.x, tip.y - neck.y};
            const float headLength = std::sqrt(axis.x * axis.x + axis.y * axis.y);
            if (headLength < 1e-3f) {
                draw->AddCircleFilled(tip, thickness, tint);
                continue;
            }
            const float halfWidth = std::clamp(headLength * .45f, 3.f, 12.f);
            const ImVec2 normal{-axis.y / headLength * halfWidth, axis.x / headLength * halfWidth};
            const ImVec2 head[] = {tip, {neck.x + normal.x, neck.y + normal.y}, {neck.x - normal.x, neck.y - normal.y}};
            if (!outlined)
                draw->AddTriangleFilled(head[0], head[1], head[2], tint);
            draw->AddTriangle(head[0], head[1], head[2], tint, 1.f);
        }
    }

    // Legend plate for the styles in use.
    if (both || haveApprox) {
        float y = rect.position.y + ui(52); // below the status pills
        const float x = rect.position.x + ui(10);
        const int lines = (both ? 2 : 0) + (haveApprox ? 1 : 0);
        float wide = 0; // the plate fits its longest line
        if (both)
            wide = std::max(ImGui::CalcTextSize("Detections: solid = simulator truth").x,
                            ImGui::CalcTextSize("Detections: cyan outline = estimate (TF)").x);
        if (haveApprox)
            wide = std::max(wide, ImGui::CalcTextSize("Detections: dim dashed = approximate (TF lagged)").x);
        draw->AddRectFilled({x - ui(6), y - ui(3)}, {x + wide + ui(6), y + ImGui::GetTextLineHeight() * lines + ui(3)},
                            rgba(.03f, .07f, .1f, .72f), 3.f);
        if (both) {
            draw->AddText({x, y}, rgba(1.f, 1.f, 1.f, .9f), "Detections: solid = simulator truth");
            y += ImGui::GetTextLineHeight();
            draw->AddText({x, y}, rgba(.25f, .9f, 1.f, .95f), "Detections: cyan outline = estimate (TF)");
            y += ImGui::GetTextLineHeight();
        }
        if (haveApprox)
            draw->AddText({x, y}, rgba(1.f, 1.f, 1.f, .7f), "Detections: dim dashed = approximate (TF lagged)");
    }
    draw->PopClipRect();
}

void drawMpcPath(const std::vector<glm::mat4> &path, const glm::mat4 &vp, const ScreenRect &rect) {
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(rect.position, {rect.position.x + rect.width, rect.position.y + rect.height}, true);
    const ImU32 line = IM_COL32(255, 170, 60, 230), heading = IM_COL32(255, 225, 150, 230);
    ImVec2 previous;
    bool previousVisible = false;
    // Connect consecutive stage positions that both project; dot every stage.
    for (std::size_t i = 0; i < path.size(); ++i) {
        ImVec2 pixel;
        const bool visible = projectToScreen(vp, rect, path[i][3], pixel);
        if (visible && previousVisible)
            draw->AddLine(previous, pixel, line, 2.f);
        if (visible) {
            draw->AddCircleFilled(pixel, 2.5f, line);
            // Body +X every few stages and at the end shows the planned heading.
            ImVec2 nose;
            if ((i % 5 == 0 || i + 1 == path.size()) &&
                projectToScreen(vp, rect, path[i] * glm::vec4(.15f, 0, 0, 1), nose))
                draw->AddLine(pixel, nose, heading, 1.5f);
        }
        previous = pixel;
        previousVisible = visible;
    }
    draw->PopClipRect();
}

void drawPlannedPath(const std::vector<glm::mat4> &path, const glm::mat4 &vp, const ScreenRect &rect) {
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(rect.position, {rect.position.x + rect.width, rect.position.y + rect.height}, true);
    const ImU32 line = IM_COL32(90, 200, 255, 200), heading = IM_COL32(190, 235, 255, 220);
    ImVec2 previous;
    bool previousVisible = false;
    for (std::size_t i = 0; i < path.size(); ++i) {
        ImVec2 pixel;
        const bool visible = projectToScreen(vp, rect, path[i][3], pixel);
        if (visible && previousVisible)
            draw->AddLine(previous, pixel, line, 1.5f);
        // Body +X every 0.5 m of plan (poses are 10 cm apart) and at the end: the planned heading.
        ImVec2 nose;
        if (visible && (i % 5 == 0 || i + 1 == path.size()) &&
            projectToScreen(vp, rect, path[i] * glm::vec4(.2f, 0, 0, 1), nose))
            draw->AddLine(pixel, nose, heading, 1.5f);
        previous = pixel;
        previousVisible = visible;
    }

    // Ring at the final pose.
    if (!path.empty()) {
        ImVec2 end;
        if (projectToScreen(vp, rect, path.back()[3], end))
            draw->AddCircle(end, 5.f, line, 0, 1.5f);
    }
    draw->PopClipRect();
}

void drawThrust(const std::vector<ThrusterMount> &mounts, const std::vector<float> &forces, const glm::mat4 &body,
                float metresPerNewton, const glm::mat4 &vp, const ScreenRect &rect) {
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(rect.position, {rect.position.x + rect.width, rect.position.y + rect.height}, true);
    const ImU32 tint = IM_COL32(230, 70, 60, 235); // RViz's default force colour, brightened for the water
    for (const auto &mount : mounts) {
        if (mount.index >= forces.size())
            continue;

        // Arrow in the world: from the mount along its axis, signed length from the force.
        const float length = forces[mount.index] * mount.inputScale * metresPerNewton;
        if (std::abs(length) < 1e-3f)
            continue;
        const glm::vec4 at = body * glm::vec4(mount.position, 1), axis = body * glm::vec4(mount.axis, 0);
        ImVec2 tail, tip;
        if (!projectToScreen(vp, rect, at, tail) || !projectToScreen(vp, rect, at + axis * length, tip))
            continue;

        // Head of a fixed screen size (shortened arrows keep a readable head), at most half the arrow.
        const ImVec2 screen{tip.x - tail.x, tip.y - tail.y};
        const float pixels = std::sqrt(screen.x * screen.x + screen.y * screen.y);
        if (pixels < 1) {
            draw->AddCircleFilled(tail, 2.5f, tint);
            continue;
        }
        const float head = std::min(10.f, pixels * .5f);
        const ImVec2 unit{screen.x / pixels, screen.y / pixels};
        const ImVec2 neck{tip.x - unit.x * head, tip.y - unit.y * head};
        const ImVec2 wing{-unit.y * head * .5f, unit.x * head * .5f};
        draw->AddCircleFilled(tail, 2.5f, tint);
        draw->AddLine(tail, neck, tint, 2.5f);
        draw->AddTriangleFilled(tip, {neck.x + wing.x, neck.y + wing.y}, {neck.x - wing.x, neck.y - wing.y}, tint);
    }
    draw->PopClipRect();
}

void drawTfAxes(const TfOverlay &tf, const glm::mat4 &vp, const ScreenRect &rect) {
    auto *draw = ImGui::GetWindowDrawList();
    draw->PushClipRect(rect.position, {rect.position.x + rect.width, rect.position.y + rect.height}, true);
    const ImU32 colors[] = {IM_COL32(255, 70, 70, 255), IM_COL32(75, 235, 100, 255), IM_COL32(80, 150, 255, 255)};
    const ImU32 white = ImGui::ColorConvertFloat4ToU32({.87f, .92f, .95f, 1});

    // Each frame: x / y / z axes in red / green / blue, a white origin dot and a shadowed name.
    if (tf.snapshot)
        for (const auto &[name, frame] : tf.snapshot->frames) {
            ImVec2 origin;
            if (!projectToScreen(vp, rect, frame[3], origin))
                continue;
            for (int axis = 0; axis < 3; ++axis) {
                ImVec2 end;
                if (!projectToScreen(vp, rect, frame[3] + frame[axis] * tf.axisLength, end))
                    continue;
                draw->AddLine(origin, end, colors[axis], 2.5f);
                draw->AddCircleFilled(end, 3, colors[axis]);
            }
            draw->AddCircleFilled(origin, 3, IM_COL32(255, 255, 255, 255));
            if (tf.names) {
                const ImVec2 at(origin.x + 5, origin.y + 5);
                draw->AddText(tf.font, tf.font->FontSize, {at.x + 1, at.y + 1}, IM_COL32(0, 0, 0, 255), name.c_str());
                draw->AddText(tf.font, tf.font->FontSize, at, white, name.c_str());
            }
        }

    // Caption and the truth-vs-estimate difference line under the status pills.
    draw->AddText(tf.font, tf.font->FontSize, {rect.position.x + ui(14), rect.position.y + ui(50)}, white,
                  tf.caption.c_str());
    if (tf.snapshot && !tf.snapshot->difference.empty())
        draw->AddText(tf.font, tf.font->FontSize, {rect.position.x + ui(14), rect.position.y + ui(67)}, white,
                      tf.snapshot->difference.c_str());
    draw->PopClipRect();
}

void drawTfTree(TfTree &tree, const std::string &root) {
    // Bulk toggles above the table.
    if (ImGui::Button("Show all"))
        tree.selectAll(true);
    ImGui::SameLine();
    if (ImGui::Button("Hide all"))
        tree.selectAll(false);
    ImGui::TextDisabled("Only this: one frame. With children: the frame and all descendants.");
    ImGui::BeginChild("TF frame tree", {620, 320}, ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
    if (ImGui::BeginTable("TF visibility", 3, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("Frame", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Only this", ImGuiTableColumnFlags_WidthFixed, 76);
        ImGui::TableSetupColumn("With children", ImGuiTableColumnFlags_WidthFixed, 112);
        ImGui::TableHeadersRow();

        // One row per frame, recursing into children while the node is open; `visited` guards against cycles.
        std::set<std::string> visited;
        const auto row = [&](const auto &self, const std::string &name) -> void {
            if (!visited.insert(name).second)
                return;
            auto &frame = tree.frames.at(name);
            const auto children = tree.children.find(name);
            const bool leaf = children == tree.children.end() || children->second.empty();
            ImGui::PushID(name.c_str());
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGuiTreeNodeFlags flags =
                ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_FramePadding | ImGuiTreeNodeFlags_SpanAvailWidth;
            if (leaf)
                flags |= ImGuiTreeNodeFlags_Leaf;
            if (name == root)
                flags |= ImGuiTreeNodeFlags_DefaultOpen;

            // Column 0: tree node (right-click for branch show / hide).
            const bool open = ImGui::TreeNodeEx("branch", flags, "%s", name.c_str());
            if (ImGui::BeginPopupContextItem("branch selection")) {
                if (ImGui::MenuItem("Show branch"))
                    tree.selectBranch(name, true);
                if (ImGui::MenuItem("Hide branch"))
                    tree.selectBranch(name, false);
                ImGui::EndPopup();
            }
            if (!frame.available) {
                ImGui::SameLine();
                ImGui::TextDisabled("(unavailable)");
            }

            // Column 1: this frame only. Column 2: tri-state for the whole branch (MixedValue when partial).
            ImGui::TableSetColumnIndex(1);
            ImGui::Checkbox("##enabled", &frame.enabled);
            ImGui::TableSetColumnIndex(2);
            if (!leaf) {
                const auto selection = tree.branchSelection(name);
                bool enabled = selection == TfTree::Selection::Shown;
                ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, selection == TfTree::Selection::Mixed);
                if (ImGui::Checkbox("##with_children", &enabled))
                    tree.selectBranch(name, enabled);
                ImGui::PopItemFlag();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Show or hide this frame and all descendants.\nA filled square means some frames are shown.");
            }

            ImGui::TableSetColumnIndex(0);
            if (open) {
                if (!leaf)
                    for (const auto &child : children->second)
                        self(self, child);
                ImGui::TreePop();
            }
            ImGui::PopID();
        };

        for (const auto &r : tree.roots)
            row(row, r);
        ImGui::EndTable();
    }
    ImGui::EndChild();
}
} // namespace nereus::ros_viewer::host
