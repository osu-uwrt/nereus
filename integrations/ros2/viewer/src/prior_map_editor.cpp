#include "prior_map_editor.hpp"
#include "nereus/ros_viewer/panel_layout.hpp"
#include "nereus/ros_viewer/pins.hpp"
#include "nereus/ros_viewer/panels/pose_math.hpp"
#include "nereus/ros_viewer/theme.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui_internal.h>
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace nereus::ros_viewer::host {
namespace fs = std::filesystem;
namespace pm = prior_map;
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kUndoDepth = 200;
const glm::vec3 kBox(.45f, .45f, .12f); // a stand-alone prop with no mesh

fs::path expandHome(const std::string &path) {
    if (!path.empty() && path[0] == '~')
        if (const char *home = std::getenv("HOME"))
            return fs::path(home) / path.substr(path.size() > 1 && path[1] == '/' ? 2 : 1);
    return path;
}
glm::mat4 poseMatrix(double x, double y, double z, double yawDegrees) {
    return glm::rotate(glm::translate(glm::mat4(1), glm::vec3(x, y, z)), float(yawDegrees * kPi / 180),
                       glm::vec3(0, 0, 1));
}
double yawOf(const glm::mat4 &m) {
    return std::atan2(m[0].y, m[0].x) * 180 / kPi;
}
// A stable colour per name (the fallback box of a prop without a mesh).
glm::vec4 colorFor(const std::string &name) {
    unsigned h = 0;
    for (const char c : name)
        h = h * 31 + static_cast<unsigned char>(c);
    const float hue = float(h % 360) / 60.f, s = .6f, v = .85f;
    const float c = v * s, x = c * (1 - std::abs(std::fmod(hue, 2.f) - 1)), m = v - c;
    const glm::vec3 rgb = hue < 1 ? glm::vec3(c, x, 0)
                          : hue < 2 ? glm::vec3(x, c, 0)
                          : hue < 3 ? glm::vec3(0, c, x)
                          : hue < 4 ? glm::vec3(0, x, c)
                          : hue < 5 ? glm::vec3(x, 0, c)
                                    : glm::vec3(c, 0, x);
    return {rgb + m, 1};
}
bool project(const glm::mat4 &vp, const glm::vec2 &origin, const glm::vec2 &size, const glm::vec3 &point,
             ImVec2 &pixel) {
    const auto clip = vp * glm::vec4(point, 1);
    if (clip.w <= 1e-4f)
        return false;
    pixel = {origin.x + (clip.x / clip.w * .5f + .5f) * size.x, origin.y + (.5f - clip.y / clip.w * .5f) * size.y};
    return std::abs(clip.x) <= clip.w * 1.2f && std::abs(clip.y) <= clip.w * 1.2f;
}
// Screen pixels per metre at `center`: the longest of the projected unit axes (from straight above the height
// axis projects to nothing).
float pixelsPerMetre(const glm::mat4 &vp, const glm::vec2 &origin, const glm::vec2 &size, const glm::vec3 &center) {
    ImVec2 c;
    if (!project(vp, origin, size, center, c))
        return 1;
    float best = 1;
    for (const glm::vec3 axis : {glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)}) {
        ImVec2 tip;
        if (project(vp, origin, size, center + axis, tip))
            best = std::max(best, std::hypot(tip.x - c.x, tip.y - c.y));
    }
    return best;
}
std::string sanitized(const fs::path &path) {
    std::string out;
    for (const char c : path.string())
        out += std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '-' ? c : '_';
    return out;
}
// Where a prop's label box starts: centred under it in 2D (as Dead Reckoning), up and to the right in 3D.
// A label chip: the prop's colour swatch, then its name.
float labelWidth(ImVec2 text) {
    return text.x + ui(22);
}
ImVec2 labelCorner(ImVec2 at, ImVec2 text, bool plan) {
    return plan ? ImVec2(at.x - labelWidth(text) * .5f, at.y + ui(11)) : ImVec2(at.x + ui(8), at.y - ui(22));
}
// An arrow from `from` to `tip` with a filled head.
void arrow(ImDrawList *d, ImVec2 from, ImVec2 tip, ImU32 color, float width) {
    glm::vec2 along(tip.x - from.x, tip.y - from.y);
    const float length = glm::length(along);
    if (length < ui(12))
        return;
    along /= length;
    const glm::vec2 side(-along.y, along.x), base = glm::vec2(tip.x, tip.y) - along * ui(11);
    d->AddLine(from, {base.x, base.y}, color, width);
    d->AddTriangleFilled(tip, {base.x + side.x * ui(6), base.y + side.y * ui(6)},
                         {base.x - side.x * ui(6), base.y - side.y * ui(6)}, color);
}
ImU32 colorU32(const glm::vec4 &c, float alpha = 1) {
    return ImGui::GetColorU32(ImVec4(c.x, c.y, c.z, c.w * alpha));
}
// A lock or eye toggle drawn as an icon (Dead Reckoning's object list): bright when on, faint when off.
enum class Icon { Lock, Eye };
bool iconToggle(const char *id, bool &on, Icon icon, ImVec4 onColor) {
    const float s = ImGui::GetFrameHeight();
    const ImVec2 at = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, {s, s});
    if (clicked)
        on = !on;
    const bool hovered = ImGui::IsItemHovered();
    auto *d = ImGui::GetWindowDrawList();
    if (hovered)
        d->AddRectFilled(at, {at.x + s, at.y + s}, ImGui::GetColorU32(ImGuiCol_FrameBgHovered), ui(3));
    const ImVec4 off = palette().muted;
    const ImU32 ink = ImGui::GetColorU32(on ? onColor : ImVec4(off.x, off.y, off.z, hovered ? 1.f : .75f));
    const ImVec2 c(at.x + s * .5f, at.y + s * .5f);
    const float k = s * .5f, w = std::max(1.f, ui(1.6f));
    if (icon == Icon::Lock) {
        const float lift = on ? 0 : k * .16f; // an open lock's shackle rides up
        d->AddRectFilled({c.x - k * .42f, c.y - k * .02f}, {c.x + k * .42f, c.y + k * .52f}, ink, ui(1.5f));
        d->PathArcTo({c.x, c.y - k * .06f - lift}, k * .26f, 3.14159f, 2 * 3.14159f, 10);
        d->PathStroke(ink, 0, w);
        d->AddLine({c.x - k * .26f, c.y - k * .06f - lift}, {c.x - k * .26f, c.y - k * .02f - (on ? 0 : lift)}, ink, w);
        d->AddLine({c.x + k * .26f, c.y - k * .06f - lift}, {c.x + k * .26f, c.y - k * .02f}, ink, w);
    } else {
        for (int i = 0; i <= 24; ++i) { // an almond: two arcs
            const float a = 3.14159f * float(i) / 24;
            d->PathLineTo({c.x - k * .55f + k * 1.1f * float(i) / 24, c.y - std::sin(a) * k * .34f});
        }
        for (int i = 24; i >= 0; --i) {
            const float a = 3.14159f * float(i) / 24;
            d->PathLineTo({c.x - k * .55f + k * 1.1f * float(i) / 24, c.y + std::sin(a) * k * .34f});
        }
        d->PathStroke(ink, ImDrawFlags_Closed, w);
        d->AddCircleFilled(c, k * .16f, ink);
        if (on) // hidden: struck through
            d->AddLine({c.x - k * .55f, c.y + k * .5f}, {c.x + k * .55f, c.y - k * .5f}, ink, w * 1.2f);
    }
    return clicked;
}
bool inputNumber(const char *label, double &value, const char *format = "%.3f", float width = 0) {
    ImGui::SetNextItemWidth(width > 0 ? width : ui(90));
    return ImGui::InputDouble(label, &value, 0, 0, format);
}
} // namespace

PriorMapEditor::PriorMapEditor(fs::path stateDirectory, fs::path defaultConfig)
    : stateDirectory_(std::move(stateDirectory)) {
    const auto path = expandHome(defaultConfig.string());
    std::snprintf(pathField_, sizeof(pathField_), "%s", path.c_str());
    std::error_code error;
    if (fs::exists(path, error))
        openFile(path);
}

// A pool switch puts the map where it was last placed in that pool (else where the scenario has it). Undo steps
// keep their props but take the new pool's origin: an old pool's origin means nothing in this one.
void PriorMapEditor::setPool(const Pool &pool) {
    pool_ = pool;
    useOrigin();
    for (auto *steps : {&undo_, &redo_})
        for (auto &step : *steps)
            step.origin = origin_;
}

void PriorMapEditor::rememberOrigin() {
    if (pool_.id.empty())
        return;
    origins_[pool_.id] = origin_;
    origins_[pool_.id + (origin_.robot ? "/robot" : "/tag")] = origin_;
}

void PriorMapEditor::useOrigin() {
    const auto saved = origins_.find(pool_.id);
    origin_ = saved != origins_.end() ? saved->second : pool_.scenarioOrigin;
}

void PriorMapEditor::setMeshes(std::vector<MappingMarker> markers) {
    meshes_.clear();
    for (auto &marker : markers)
        meshes_.emplace(marker.frame, std::move(marker));
}

// ------------------------------------------------------------------ files

void PriorMapEditor::openFile(const fs::path &path) {
    try {
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("cannot read " + path.string());
        std::stringstream text;
        text << in.rdbuf();
        doc_ = pm::load(text.str());
        configPath_ = path;
        std::snprintf(pathField_, sizeof(pathField_), "%s", path.c_str());
        selected_.clear();
        undo_.clear();
        redo_.clear();
        dirty_ = false;
        loadState();
        message_ = "Loaded " + std::to_string(doc_.objects.size()) + " objects (" + doc_.ns + ")";
        messageError_ = false;
    } catch (const std::exception &error) {
        message_ = error.what();
        messageError_ = true;
    }
}

void PriorMapEditor::saveFile() {
    if (!loaded())
        return;
    try {
        const auto text = pm::save(doc_);
        {
            std::ofstream out(configPath_);
            out << text;
            if (!out)
                throw std::runtime_error("cannot write " + configPath_.string());
        }
        auto reloaded = pm::load(text, doc_.ns);
        for (auto &o : reloaded.objects) // the editor's own flags survive the reload
            if (const auto *before = pm::find(doc_.objects, o.name)) {
                o.locked = before->locked;
                o.hidden = before->hidden;
            }
        doc_ = std::move(reloaded);
        dirty_ = false;
        saveState();
        message_ = "Saved " + configPath_.filename().string() + " (read back and checked)";
        messageError_ = false;
    } catch (const std::exception &error) {
        message_ = std::string("Not saved: ") + error.what();
        messageError_ = true;
    }
}

fs::path PriorMapEditor::statePath() const {
    return stateDirectory_ / (sanitized(configPath_) + ".yaml");
}

// The editor's own state per config file: origin, locks, hidden props, display options.
void PriorMapEditor::loadState() {
    std::error_code error;
    if (stateDirectory_.empty() || !fs::exists(statePath(), error))
        return;
    try {
        const auto state = YAML::LoadFile(statePath().string());
        origins_.clear();
        for (const auto &entry : state["origins"]) {
            const auto &o = entry.second;
            pm::Origin origin;
            origin.x = o["x"].as<double>(0);
            origin.y = o["y"].as<double>(0);
            origin.z = o["z"].as<double>(0);
            origin.basePhi = o["base_phi"].as<double>(0);
            origin.yawOffset = o["yaw_offset"].as<double>(0);
            origin.wall = o["wall"].as<std::string>("W")[0];
            origin.robot = o["robot"].as<bool>(false);
            origins_[entry.first.as<std::string>()] = origin;
        }
        useOrigin();
        poolLock_ = state["pool_lock"].as<bool>(poolLock_);
        labels_ = state["labels"].as<int>(labels_);
        poseColumns_ = state["pose_columns"].as<bool>(poseColumns_);
        hideCourse_ = state["hide_course"].as<bool>(hideCourse_);
        const auto names = [&](const char *key) {
            std::set<std::string> out;
            for (const auto &n : state[key])
                out.insert(n.as<std::string>());
            return out;
        };
        if (state["locked"]) {
            const auto locked = names("locked");
            for (auto &o : doc_.objects)
                o.locked = locked.count(o.name) > 0;
        }
        const auto hidden = names("hidden");
        for (auto &o : doc_.objects)
            o.hidden = hidden.count(o.name) > 0;
    } catch (const std::exception &e) {
        std::cerr << "nereus-viewer: prior map editor state " << statePath() << ": " << e.what() << '\n';
    }
}

void PriorMapEditor::saveState() const {
    if (stateDirectory_.empty() || configPath_.empty())
        return;
    YAML::Node state;
    state["config"] = configPath_.string();
    state["namespace"] = doc_.ns;
    for (const auto &[pool, origin] : origins_) {
        auto o = state["origins"][pool];
        o["x"] = origin.x;
        o["y"] = origin.y;
        o["z"] = origin.z;
        o["base_phi"] = origin.basePhi;
        o["yaw_offset"] = origin.yawOffset;
        o["wall"] = std::string(1, origin.wall);
        o["robot"] = origin.robot;
    }
    state["pool_lock"] = poolLock_;
    state["labels"] = labels_;
    state["pose_columns"] = poseColumns_;
    state["hide_course"] = hideCourse_;
    state["locked"] = YAML::Node(YAML::NodeType::Sequence);
    state["hidden"] = YAML::Node(YAML::NodeType::Sequence);
    for (const auto &o : doc_.objects) {
        if (o.locked)
            state["locked"].push_back(o.name);
        if (o.hidden)
            state["hidden"].push_back(o.name);
    }
    std::error_code error;
    fs::create_directories(stateDirectory_, error);
    std::ofstream(statePath()) << state << '\n';
}

// ------------------------------------------------------------------ edits

void PriorMapEditor::record() {
    undo_.push_back({doc_.objects, origin_});
    if (undo_.size() > kUndoDepth)
        undo_.erase(undo_.begin());
    redo_.clear();
}
void PriorMapEditor::undo() {
    if (undo_.empty())
        return;
    redo_.push_back({doc_.objects, origin_});
    doc_.objects = undo_.back().objects;
    origin_ = undo_.back().origin;
    undo_.pop_back();
    changed();
}
void PriorMapEditor::redo() {
    if (redo_.empty())
        return;
    undo_.push_back({doc_.objects, origin_});
    doc_.objects = redo_.back().objects;
    origin_ = redo_.back().origin;
    redo_.pop_back();
    changed();
}
void PriorMapEditor::changed() {
    dirty_ = true;
    if (!pool_.id.empty())
        rememberOrigin(); // an undo or redo can move the origin too
    if (!selected_.empty() && !pm::find(doc_.objects, selected_))
        selected_.clear();
    saveState();
}

void PriorMapEditor::setOrigin(const pm::Origin &next) {
    record();
    startObjects_ = doc_.objects;
    if (poolLock_)
        pm::keepInPool(doc_.objects, origin_, next); // the props stay where they are in the pool
    origin_ = next;
    originMoved();
}

// The origin is the editor's own (not in the file); the file changes only when pinned props really moved (a mode
// switch re-expresses the same origin).
void PriorMapEditor::originMoved() {
    for (std::size_t i = 0; poolLock_ && i < doc_.objects.size() && i < startObjects_.size(); ++i) {
        const auto &a = doc_.objects[i].pose, &b = startObjects_[i].pose;
        if (std::abs(a.x - b.x) > 1e-6 || std::abs(a.y - b.y) > 1e-6 || std::abs(a.z - b.z) > 1e-6 ||
            std::abs(pm::wrapDegrees(a.yaw - b.yaw)) > 1e-6)
            dirty_ = true;
    }
    rememberOrigin();
    saveState();
}

float PriorMapEditor::handleLength(const View &view, const glm::vec3 &center) const {
    return ui(90) / pixelsPerMetre(view.viewProjection, view.origin, view.size, center);
}

bool PriorMapEditor::onRobotOrigin(const View &view, const glm::vec2 &mouse) const {
    if (!origin_.robot)
        return false;
    const glm::vec3 at(worldFromMap()[3]);
    ImVec2 pixel;
    if (project(view.viewProjection, view.origin, view.size, at, pixel) &&
        glm::length(glm::vec2(pixel.x, pixel.y) - mouse) < ui(9))
        return true;
    if (!view.pointer)
        return false;
    // on the robot's hull: about Talos' size around base_link
    const glm::vec3 local(glm::inverse(worldFromMap()) * glm::vec4(*view.pointer, 1));
    return std::abs(local.x) < .45f && std::abs(local.y) < .4f && (view.plan || std::abs(local.z) < .3f);
}

bool PriorMapEditor::onOriginRing(const View &view, const glm::vec2 &mouse) const {
    if (!origin_.robot || !originSelected_)
        return false;
    const glm::vec3 center(worldFromMap()[3]);
    const float radius = handleLength(view, center) * .8f;
    for (int step = 0; step < 72; ++step) {
        const float a0 = float(step) * 2 * float(kPi) / 72, a1 = float(step + 1) * 2 * float(kPi) / 72;
        ImVec2 p0, p1;
        if (!project(view.viewProjection, view.origin, view.size, center + glm::vec3(std::cos(a0), std::sin(a0), 0) * radius,
                     p0) ||
            !project(view.viewProjection, view.origin, view.size, center + glm::vec3(std::cos(a1), std::sin(a1), 0) * radius,
                     p1))
            continue;
        float fraction;
        if (segmentDistance(mouse, {p0.x, p0.y}, {p1.x, p1.y}, fraction) < ui(8))
            return true;
    }
    return false;
}

// The origin's X / Y arrows as drawn (2D: 64 px long; 3D: 0.7 m), clear of the origin itself (the robot drags it).
int PriorMapEditor::onOriginArrow(const View &view, const glm::vec2 &mouse) const {
    if (!origin_.robot)
        return -1;
    const auto map = worldFromMap();
    const glm::vec3 o(map[3]);
    ImVec2 po;
    if (!project(view.viewProjection, view.origin, view.size, o, po))
        return -1;
    const float metres = view.plan ? ui(64) / pixelsPerMetre(view.viewProjection, view.origin, view.size, o) : .7f;
    int best = -1;
    float nearest = ui(8);
    for (int axis = 0; axis < 2; ++axis) {
        ImVec2 tip;
        if (!project(view.viewProjection, view.origin, view.size, o + glm::normalize(glm::vec3(map[axis])) * metres, tip))
            continue;
        float fraction = 0;
        const float d = segmentDistance(mouse, {po.x, po.y}, {tip.x, tip.y}, fraction);
        if (d < nearest && glm::length(mouse - glm::vec2(po.x, po.y)) > ui(10)) {
            nearest = d;
            best = axis;
        }
    }
    return best;
}

// A nudge in the pool frame (dx, dy, dz in metres, dyaw in degrees), the subtree riding along.
void PriorMapEditor::moveSelected(double dx, double dy, double dz, double dyaw) {
    auto *o = pm::find(doc_.objects, selected_);
    if (!o || o->locked)
        return;
    const double a = -origin_.yaw() * kPi / 180;
    auto pose = pm::mapPoses(doc_.objects).at(selected_);
    pose.x += dx * std::cos(a) - dy * std::sin(a);
    pose.y += dx * std::sin(a) + dy * std::cos(a);
    pose.z += dz;
    pose.yaw = pm::wrapDegrees(pose.yaw + dyaw);
    pm::setMapPose(doc_.objects, selected_, pose);
}

// ------------------------------------------------------------------ geometry

glm::mat4 PriorMapEditor::worldFromMap() const {
    return pool_.poolToWorld * poseMatrix(origin_.x, origin_.y, origin_.z, origin_.yaw());
}
glm::mat4 PriorMapEditor::worldOf(const pm::Pose &p) const {
    return worldFromMap() * poseMatrix(p.x, p.y, p.z, p.yaw);
}
pm::Pose PriorMapEditor::mapFromWorld(const glm::vec3 &world, double yawWorld) const {
    const auto map = glm::inverse(worldFromMap());
    const glm::vec3 p(map * glm::vec4(world, 1));
    return {p.x, p.y, p.z, pm::wrapDegrees(yawWorld - yawOf(worldFromMap()))};
}

void PriorMapEditor::addMarkers(std::vector<MarkerDraw> &markers) const {
    if (!active())
        return;
    const auto poses = pm::mapPoses(doc_.objects);
    const auto meshed = [&](const std::string &name) {
        const auto found = meshes_.find(name + "_frame");
        return found != meshes_.end() && found->second.visible;
    };
    // A mesh-less frame on a meshed assembly (the bin's targets, the torpedo's holes) is part of that mesh.
    const auto onMesh = [&](const pm::Object &o) {
        std::set<std::string> seen;
        for (auto *parent = pm::find(doc_.objects, o.parent); parent && seen.insert(parent->name).second;
             parent = pm::find(doc_.objects, parent->parent))
            if (meshed(parent->name))
                return true;
        return false;
    };
    for (const auto &o : doc_.objects) {
        if (o.hidden)
            continue;
        const auto world = worldOf(poses.at(o.name));
        MarkerDraw draw;
        draw.observerOnly = true;
        const auto mesh = meshes_.find(o.name + "_frame");
        if (meshed(o.name)) {
            draw.mesh = mesh->second.path;
            draw.world = world * mesh->second.local;
        } else if (onMesh(o) || plan_) {
            continue; // part of a meshed assembly, or 2D: drawn as a dot / badge by the overlay
        } else { // no mesh: a coloured box at the pose
            draw.world = world;
            draw.scale = kBox;
            draw.tint = colorFor(o.name);
        }
        markers.push_back(std::move(draw));
    }
}

void PriorMapEditor::focusOn(const std::string &name) {
    if (pm::find(doc_.objects, name))
        focus_ = glm::vec3(worldOf(pm::mapPoses(doc_.objects).at(name))[3]);
}

PriorMapEditor::Look PriorMapEditor::lookOf(const pm::Object &o) const {
    const auto meshed = [&](const std::string &name) {
        const auto found = meshes_.find(name + "_frame");
        return found != meshes_.end() && found->second.visible;
    };
    if (meshed(o.name))
        return Look::Mesh;
    std::set<std::string> seen;
    for (auto *parent = pm::find(doc_.objects, o.parent); parent && seen.insert(parent->name).second;
         parent = pm::find(doc_.objects, parent->parent))
        if (meshed(parent->name))
            return Look::OnAssembly;
    return Look::Bare;
}

bool PriorMapEditor::labelShown(const pm::Object &o) const {
    return !o.hidden && (o.name == selected_ || labels_ == 2 || (labels_ == 1 && o.parent == pm::kMap));
}

// A drawn prop's extent: its mesh's bounds, the box drawn for a stand-alone prop with no mesh, or none for a
// frame on a meshed assembly (drawn as part of that mesh).
std::optional<PriorMapEditor::Extent> PriorMapEditor::extentOf(const pm::Object &o) const {
    const auto mesh = meshes_.find(o.name + "_frame");
    if (mesh != meshes_.end() && mesh->second.visible) {
        const auto found = extents_.find(o.name + "_frame");
        return found != extents_.end() ? std::optional<Extent>(found->second) : std::nullopt;
    }
    std::set<std::string> seen;
    for (auto *parent = pm::find(doc_.objects, o.parent); parent && seen.insert(parent->name).second;
         parent = pm::find(doc_.objects, parent->parent)) {
        const auto found = meshes_.find(parent->name + "_frame");
        if (found != meshes_.end() && found->second.visible)
            return std::nullopt;
    }
    return Extent{kBox * -.5f, kBox * .5f};
}

// The object under the pointer: an unlocked, shown prop whose label or origin dot is under it, else the smallest
// whose extent holds the scene point under it (2D: across the pool, any height). Locked props are click-through,
// as in Dead Reckoning.
std::string PriorMapEditor::pick(const View &view, const glm::vec2 &mouse) const {
    const auto poses = pm::mapPoses(doc_.objects);
    std::string best;
    for (const auto &label : labelBoxes_) // the label boxes as last drawn
        if (const auto *o = pm::find(doc_.objects, label.name);
            o && !o->hidden && !o->locked && mouse.x >= label.min.x && mouse.x <= label.max.x &&
            mouse.y >= label.min.y && mouse.y <= label.max.y)
            return label.name;
    float bestPixels = ui(view.plan ? 9.f : 7.f); // the origin dot / badge
    for (const auto &o : doc_.objects) {
        if (o.hidden || o.locked)
            continue;
        ImVec2 at;
        if (!project(view.viewProjection, view.origin, view.size, glm::vec3(worldOf(poses.at(o.name))[3]), at))
            continue;
        const float d = glm::length(glm::vec2(at.x, at.y) - mouse);
        if (d < bestPixels) {
            bestPixels = d;
            best = o.name;
        }
    }
    if (!best.empty() || !view.pointer)
        return best;
    constexpr float margin = .03f;
    float smallest = 1e30f;
    for (const auto &o : doc_.objects) {
        if (o.hidden || o.locked)
            continue;
        const auto extent = extentOf(o);
        if (!extent)
            continue;
        const glm::vec3 local(glm::inverse(worldOf(poses.at(o.name))) * glm::vec4(*view.pointer, 1));
        const auto inside = [&](int axis) {
            return local[axis] >= extent->low[axis] - margin && local[axis] <= extent->high[axis] + margin;
        };
        if (!inside(0) || !inside(1) || (!view.plan && !inside(2)))
            continue;
        const glm::vec3 size = extent->high - extent->low;
        const float area = (size.x + margin) * (size.y + margin) * (view.plan ? 1 : size.z + margin);
        if (area < smallest) {
            smallest = area;
            best = o.name;
        }
    }
    return best;
}

bool PriorMapEditor::input(const View &view) {
    if (!active())
        return false;
    auto &io = ImGui::GetIO();
    const glm::vec2 mouse(io.MousePos.x, io.MousePos.y);
    const auto ray = screenRay(view.viewProjection, mouse - view.origin, view.size);
    // Placing the origin: a click picks the AprilTag spot (or the free point) under the pointer.
    if (placing_) {
        if (!view.hovered || !ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            return false; // the camera still orbits and pans
        {
            // the scene point under the pointer, else where the pointer meets the origin's height
            glm::vec3 world;
            if (view.pointer)
                world = *view.pointer;
            else if (!planeHit(ray, glm::vec3(worldFromMap()[3]), {0, 0, 1}, world)) {
                message_ = "Click on the pool to place the origin";
                messageError_ = true;
                return true;
            }
            const glm::vec3 inPool(glm::inverse(pool_.poolToWorld) * glm::vec4(world, 1));
            auto next = origin_;
            if (origin_.robot) {
                next.x = inPool.x;
                next.y = inPool.y;
            } else {
                const auto spot = pm::nearestSpot(pm::tagSpots(pool_.length, pool_.width, pool_.lines), inPool.x, inPool.y);
                if (!spot) {
                    message_ = "No line / wall intersection or corner within 3 m of that point";
                    messageError_ = true;
                    return true;
                }
                next.x = spot->x;
                next.y = spot->y;
                next.basePhi = spot->phi;
                next.yawOffset = 0; // +X out of the wall
                next.wall = spot->wall;
            }
            setOrigin(next);
            placing_ = false;
            message_ = origin_.robot ? "Robot-frame origin placed"
                                     : std::string("AprilTag on the ") + origin_.wall + " wall, facing into the pool";
            messageError_ = false;
        }
        return true;
    }
    auto *selected = pm::find(doc_.objects, selected_);
    const bool movable = selected && !selected->locked && !selected->hidden;
    // Dragging the robot-frame origin (the robot): across the pool, along one map axis by its arrow, or turning it
    // with its ring.
    if (drag_ == Handle::OriginBody || drag_ == Handle::OriginYaw || drag_ == Handle::OriginX ||
        drag_ == Handle::OriginY) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) { // put it back; no undo step
            origin_ = startOrigin_;
            doc_.objects = startObjects_;
            drag_ = Handle::None;
            if (!undo_.empty())
                undo_.pop_back();
            return true;
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            drag_ = Handle::None;
            originMoved();
            return true;
        }
        const glm::vec3 center(pool_.poolToWorld *
                               glm::vec4(float(startOrigin_.x), float(startOrigin_.y), float(startOrigin_.z), 1));
        auto next = startOrigin_;
        glm::vec3 hit;
        if (planeHit(ray, center, {0, 0, 1}, hit)) {
            if (drag_ == Handle::OriginBody || drag_ == Handle::OriginX || drag_ == Handle::OriginY) {
                glm::vec3 moved = hit - grab_;
                if (drag_ != Handle::OriginBody) { // only along the arrow's map axis (a move never turns the origin)
                    const auto map = worldFromMap();
                    const glm::vec3 axis = glm::normalize(glm::vec3(map[drag_ == Handle::OriginX ? 0 : 1]));
                    moved = axis * glm::dot(moved, axis);
                }
                const glm::vec3 inPool(glm::inverse(pool_.poolToWorld) * glm::vec4(center + moved, 1));
                next.x = inPool.x;
                next.y = inPool.y;
            } else {
                double turn = pm::wrapDegrees(std::atan2(hit.y - center.y, hit.x - center.x) * 180 / kPi - grabAngle_);
                if (io.KeyShift)
                    turn = std::round(turn / 15) * 15;
                next.yawOffset = pm::wrapDegrees(startOrigin_.yawOffset + turn);
            }
        }
        doc_.objects = startObjects_;
        if (poolLock_)
            pm::keepInPool(doc_.objects, startOrigin_, next);
        origin_ = next;
        return true;
    }
    // A gesture in progress follows the pointer until the button comes up (Esc puts it back).
    if (drag_ != Handle::None) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) { // put it back; no undo step
            pm::setMapPose(doc_.objects, selected_, startMap_);
            drag_ = Handle::None;
            if (!undo_.empty())
                undo_.pop_back();
            return true;
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) || !movable) {
            drag_ = Handle::None;
            return true;
        }
        const auto start = worldOf(startMap_);
        const glm::vec3 center(start[3]);
        auto pose = startMap_;
        glm::vec3 hit;
        float along = 0;
        if (drag_ == Handle::Body && planeHit(ray, center, {0, 0, 1}, hit)) {
            const auto moved = mapFromWorld(center + (hit - grab_), 0);
            pose.x = moved.x;
            pose.y = moved.y;
        } else if (drag_ == Handle::X || drag_ == Handle::Y || drag_ == Handle::Z) {
            const glm::vec3 axis = drag_ == Handle::Z ? glm::vec3(0, 0, 1)
                                                      : glm::normalize(glm::vec3(worldFromMap()[drag_ == Handle::X ? 0 : 1]));
            if (axisHit(ray, center, axis, along)) {
                const auto moved = mapFromWorld(center + axis * (along - grab_.x), 0);
                pose.x = moved.x;
                pose.y = moved.y;
                pose.z = moved.z;
            }
        } else if (drag_ == Handle::Yaw && planeHit(ray, center, {0, 0, 1}, hit)) {
            const double angle = std::atan2(hit.y - center.y, hit.x - center.x) * 180 / kPi;
            double turn = pm::wrapDegrees(angle - grabAngle_);
            if (io.KeyShift)
                turn = std::round(turn / 15) * 15; // Shift: 15 degree steps
            pose.yaw = pm::wrapDegrees(startMap_.yaw + turn);
        }
        pm::setMapPose(doc_.objects, selected_, pose);
        dirty_ = true;
        return true;
    }
    if (!view.hovered)
        return false;
    // A double-click on a prop looks at it.
    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        const auto hit = pick(view, mouse);
        if (!hit.empty()) {
            focusOn(hit);
            pressed_ = false;
            return true;
        }
    }
    // Handles of the selected prop: X / Y / Z arrows and the yaw ring.
    if (movable && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const auto poses = pm::mapPoses(doc_.objects);
        const auto start = worldOf(poses.at(selected_));
        const glm::vec3 center(start[3]);
        ImVec2 c;
        if (project(view.viewProjection, view.origin, view.size, center, c)) {
            const float length = ui(90) / pixelsPerMetre(view.viewProjection, view.origin, view.size, center);
            Handle hit = Handle::None;
            float best = ui(10);
            const glm::vec3 axes[] = {glm::normalize(glm::vec3(worldFromMap()[0])),
                                      glm::normalize(glm::vec3(worldFromMap()[1])), {0, 0, 1}};
            const Handle names[] = {Handle::X, Handle::Y, Handle::Z};
            for (int i = 0; i < (view.plan ? 0 : 3); ++i) { // 2D: drag the prop itself, turn with the ring
                ImVec2 tip;
                if (!project(view.viewProjection, view.origin, view.size, center + axes[i] * length, tip))
                    continue;
                float fraction;
                const float d = segmentDistance(mouse, {c.x, c.y}, {tip.x, tip.y}, fraction);
                if (d < best && fraction > .15f) {
                    best = d;
                    hit = names[i];
                }
            }
            for (int step = 0; step < 72 && hit == Handle::None; ++step) {
                const float a0 = float(step) * 2 * float(kPi) / 72, a1 = float(step + 1) * 2 * float(kPi) / 72;
                ImVec2 p0, p1;
                if (!project(view.viewProjection, view.origin, view.size,
                             center + glm::vec3(std::cos(a0), std::sin(a0), 0) * length * .8f, p0) ||
                    !project(view.viewProjection, view.origin, view.size,
                             center + glm::vec3(std::cos(a1), std::sin(a1), 0) * length * .8f, p1))
                    continue;
                float fraction;
                if (segmentDistance(mouse, {p0.x, p0.y}, {p1.x, p1.y}, fraction) < ui(8))
                    hit = Handle::Yaw;
            }
            if (hit == Handle::None && pick(view, mouse) == selected_)
                hit = Handle::Body; // the prop itself (closer than a select): drag it across the pool
            if (hit != Handle::None) {
                record();
                drag_ = hit;
                startMap_ = poses.at(selected_);
                glm::vec3 point;
                float along = 0;
                if (hit == Handle::Body && planeHit(ray, center, {0, 0, 1}, point))
                    grab_ = point;
                else if (hit == Handle::Yaw && planeHit(ray, center, {0, 0, 1}, point))
                    grabAngle_ = std::atan2(point.y - center.y, point.x - center.x) * 180 / kPi;
                else if (hit != Handle::Body && hit != Handle::Yaw &&
                         axisHit(ray, center, axes[hit == Handle::X ? 0 : hit == Handle::Y ? 1 : 2], along))
                    grab_ = {along, 0, 0};
                else
                    grab_ = center;
                return true;
            }
        }
    }
    // The robot-frame origin (the robot): its ring turns it; the robot itself drags it (in 3D once selected, as props).
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && origin_.robot) {
        const bool ring = onOriginRing(view, mouse);
        const int arrow = ring ? -1 : onOriginArrow(view, mouse);
        if (ring || arrow >= 0 || (onRobotOrigin(view, mouse) && (view.plan || originSelected_))) {
            const glm::vec3 center(worldFromMap()[3]);
            glm::vec3 point;
            if (planeHit(ray, center, {0, 0, 1}, point)) {
                record();
                originSelected_ = true;
                selected_.clear();
                drag_ = ring ? Handle::OriginYaw : arrow == 0 ? Handle::OriginX : arrow == 1 ? Handle::OriginY : Handle::OriginBody;
                startOrigin_ = origin_;
                startObjects_ = doc_.objects;
                grab_ = point;
                grabAngle_ = std::atan2(point.y - center.y, point.x - center.x) * 180 / kPi;
                return true;
            }
        }
    }
    // 2D: pressing on any (unlocked) prop drags it at once, as in Dead Reckoning.
    if (view.plan && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        const auto grabbed = pick(view, mouse);
        glm::vec3 point;
        if (!grabbed.empty()) {
            const auto poses = pm::mapPoses(doc_.objects);
            const glm::vec3 center(worldOf(poses.at(grabbed))[3]);
            if (planeHit(ray, center, {0, 0, 1}, point)) {
                selected_ = grabbed;
                originSelected_ = false;
                std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
                record();
                drag_ = Handle::Body;
                startMap_ = poses.at(grabbed);
                grab_ = point;
                return true;
            }
        }
    }
    // A click (not a camera drag) selects; on empty space it clears the selection.
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        pressed_ = true;
        pressMouse_ = mouse;
    }
    if (pressed_ && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        pressed_ = false;
        if (glm::length(mouse - pressMouse_) < 4) {
            selected_ = pick(view, mouse);
            originSelected_ = selected_.empty() && onRobotOrigin(view, mouse); // the robot: select the origin
            if (!selected_.empty())
                std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
            return true;
        }
    }
    return false;
}

void PriorMapEditor::drawOverlay(const View &view, ImFont *small) const {
    if (!active())
        return;
    labelFont_ = typeRamp().smallStrong ? typeRamp().smallStrong : small;
    std::vector<std::pair<ImVec2, ImVec2>> taken; // what labels keep clear of: badges, chips, the scale bar
    // the view's own chip: in 2D, the scale bar (top right)
    if (view.plan)
        taken.push_back({{view.origin.x + view.size.x - ui(200), view.origin.y},
                         {view.origin.x + view.size.x, view.origin.y + ui(70)}});
    auto *d = ImGui::GetWindowDrawList();
    const auto &p = palette();
    const auto proj = [&](const glm::vec3 &point, ImVec2 &pixel) {
        return project(view.viewProjection, view.origin, view.size, point, pixel);
    };
    d->PushClipRect({view.origin.x, view.origin.y}, {view.origin.x + view.size.x, view.origin.y + view.size.y}, true);
    // The map origin: its axes (red x, green y, blue z) and, for an AprilTag, the tag on the wall.
    const auto map = worldFromMap();
    const glm::vec3 o(map[3]);
    ImVec2 po;
    if (proj(o, po)) {
        const ImU32 colors[] = {IM_COL32(240, 96, 100, 255), IM_COL32(84, 208, 144, 255), IM_COL32(80, 145, 255, 255)};
        // 3D: axes 0.7 m long; 2D: X and Y as arrows a fixed 64 px long (the map frame reads at any zoom)
        const float metres = view.plan ? ui(64) / pixelsPerMetre(view.viewProjection, view.origin, view.size, o) : .7f;
        for (int i = 0; i < (view.plan ? 2 : 3); ++i) {
            ImVec2 tip;
            if (proj(o + glm::normalize(glm::vec3(map[i])) * metres, tip)) {
                const bool dragged = (i == 0 && drag_ == Handle::OriginX) || (i == 1 && drag_ == Handle::OriginY);
                arrow(d, po, tip, colors[i], (view.plan ? ui(3.5f) : ui(3)) * (dragged ? 1.6f : 1.f));
            }
        }
        if (!origin_.robot)
            d->AddRect({po.x - ui(7), po.y - ui(7)}, {po.x + ui(7), po.y + ui(7)}, IM_COL32(255, 255, 255, 230), 0, 0,
                       ui(2));
        const char *name = origin_.robot ? "map \u00b7 robot start" : "map \u00b7 AprilTag";
        const ImVec2 text = small->CalcTextSizeA(small->FontSize, 1e9f, 0, name);
        const ImVec2 chip(po.x - text.x * .5f - ui(6), po.y + ui(12));
        d->AddRectFilled(chip, {chip.x + text.x + ui(12), chip.y + text.y + ui(6)}, IM_COL32(232, 237, 243, 240), ui(3));
        taken.push_back({chip, {chip.x + text.x + ui(12), chip.y + text.y + ui(6)}});
        d->AddText(small, small->FontSize, {chip.x + ui(6), chip.y + ui(3)}, IM_COL32(14, 18, 24, 255), name);
        // the selected robot-frame origin: its turning ring
        const bool turning = drag_ == Handle::OriginYaw, moving = drag_ == Handle::OriginBody;
        if (origin_.robot && (originSelected_ || turning || moving)) {
            const float radius = handleLength(view, o) * .8f;
            ImVec2 previous;
            bool havePrevious = false;
            for (int step = 0; step <= 72; ++step) {
                const float a = float(step) * 2 * float(kPi) / 72;
                ImVec2 point;
                const bool ok = proj(o + glm::vec3(std::cos(a), std::sin(a), 0) * radius, point);
                if (ok && havePrevious)
                    d->AddLine(previous, point, IM_COL32(250, 210, 90, 255), turning ? ui(4) : ui(2.5f));
                previous = point;
                havePrevious = ok;
            }
        }
    }
    // While placing: the AprilTag spots.
    if (placing_ && !origin_.robot)
        for (const auto &spot : pm::tagSpots(pool_.length, pool_.width, pool_.lines)) {
            ImVec2 s;
            if (proj(glm::vec3(pool_.poolToWorld * glm::vec4(spot.x, spot.y, origin_.z, 1)), s))
                d->AddCircleFilled(s, ui(4), ImGui::GetColorU32(p.accent));
        }
    const auto poses = pm::mapPoses(doc_.objects);
    // 2D: each meshed prop's footprint (its mesh bounds from above) outlined in its colour, with a notch on the +X
    // side for its heading.
    if (view.plan)
        for (const auto &obj : doc_.objects) {
            // top-level props (an assembly's parts ride with it); the selected one whatever its depth
            if (obj.hidden || lookOf(obj) != Look::Mesh || (obj.parent != pm::kMap && obj.name != selected_))
                continue;
            const auto extent = extents_.find(obj.name + "_frame");
            if (extent == extents_.end())
                continue;
            const auto pose = worldOf(poses.at(obj.name));
            const auto &lo = extent->second.low, &hi = extent->second.high;
            const float z = (lo.z + hi.z) * .5f;
            ImVec2 corners[4];
            bool ok = true;
            const glm::vec2 xy[] = {{lo.x, lo.y}, {hi.x, lo.y}, {hi.x, hi.y}, {lo.x, hi.y}};
            for (int i = 0; i < 4 && ok; ++i)
                ok = proj(glm::vec3(pose * glm::vec4(xy[i], z, 1)), corners[i]);
            if (!ok)
                continue;
            const auto color = colorFor(obj.name);
            const bool chosen = obj.name == selected_;
            const float fade = obj.locked ? .45f : 1;
            // corner brackets only, so the mesh stays visible (the selected prop's in the accent, heavier)
            {
                const ImU32 ink = chosen ? ImGui::GetColorU32(p.accent) : colorU32(color, .9f * fade);
                const float width = chosen ? ui(3) : ui(2), reach = chosen ? ui(12) : ui(9);
                for (int i = 0; i < 4; ++i) {
                    const glm::vec2 c(corners[i].x, corners[i].y);
                    for (const int j : {(i + 1) % 4, (i + 3) % 4}) {
                        const glm::vec2 edge = glm::vec2(corners[j].x, corners[j].y) - c;
                        const float length = glm::length(edge);
                        if (length < 1)
                            continue;
                        const glm::vec2 end = c + edge * (std::min(reach, length * .3f) / length);
                        d->AddLine(corners[i], {end.x, end.y}, ink, width);
                    }
                }
            }
            ImVec2 mid, ahead;
            if (proj(glm::vec3(pose * glm::vec4(hi.x, (lo.y + hi.y) * .5f, z, 1)), mid) &&
                proj(glm::vec3(pose * glm::vec4(hi.x + 1, (lo.y + hi.y) * .5f, z, 1)), ahead)) {
                glm::vec2 dir(ahead.x - mid.x, ahead.y - mid.y);
                if (glm::length(dir) > 1e-3f) {
                    dir = glm::normalize(dir);
                    const glm::vec2 side(-dir.y, dir.x), m(mid.x, mid.y), tip = m + dir * ui(7),
                                                           b1 = m + side * ui(5), b2 = m - side * ui(5);
                    d->AddTriangleFilled({tip.x, tip.y}, {b1.x, b1.y}, {b2.x, b2.y}, colorU32(color, fade));
                }
            }
        }
    // Markers in each prop's colour: a frame on a meshed assembly is a small dot; in 2D a stand-alone prop without a
    // mesh is a badge with a triangle along its heading (3D draws it as a box). A meshed prop is its mesh.
    for (const auto &obj : doc_.objects) {
        if (obj.hidden)
            continue;
        const auto look = lookOf(obj);
        if (look == Look::Mesh || (look == Look::Bare && !view.plan))
            continue;
        const auto pose = worldOf(poses.at(obj.name));
        ImVec2 at, ahead;
        if (!proj(glm::vec3(pose[3]), at))
            continue;
        const auto color = colorFor(obj.name);
        const float fade = obj.locked ? .55f : 1;
        const bool chosen = obj.name == selected_;
        if (look == Look::OnAssembly) {
            d->AddCircleFilled(at, ui(3.5f), colorU32(color, fade));
            d->AddCircle(at, ui(3.5f), IM_COL32(8, 22, 29, 200), 0, ui(1));
            continue;
        }
        const float half = ui(8);
        d->AddRectFilled({at.x - half, at.y - half}, {at.x + half, at.y + half}, colorU32(color * .8f, fade), ui(3));
        d->AddRect({at.x - half, at.y - half}, {at.x + half, at.y + half},
                   chosen ? ImGui::GetColorU32(p.accent) : IM_COL32(8, 22, 29, 220), ui(3), 0, chosen ? ui(2.5f) : ui(1));
        if (proj(glm::vec3(pose * glm::vec4(1, 0, 0, 1)), ahead)) { // the heading, as seen
            glm::vec2 dir(ahead.x - at.x, ahead.y - at.y);
            if (glm::length(dir) > 1e-3f) {
                dir = glm::normalize(dir);
                const glm::vec2 side(-dir.y, dir.x), c(at.x, at.y);
                const glm::vec2 tip = c + dir * ui(5.5f), b1 = c - dir * ui(4) + side * ui(4.5f),
                                b2 = c - dir * ui(4) - side * ui(4.5f);
                d->AddTriangleFilled({tip.x, tip.y}, {b1.x, b1.y}, {b2.x, b2.y}, IM_COL32(255, 255, 255, int(235 * fade)));
            }
        }
    }
    // Labels: map roots, everything, or nothing; the selected one always. Each takes the first free spot around its
    // prop (the selection first), clear of other labels, the props' badges and the chips; a label with no free spot
    // is left out (its prop still shows, and hovering the list names it).
    auto *font = labelFont_;
    std::vector<std::pair<const pm::Object *, ImVec2>> wanted;
    for (const auto &obj : doc_.objects) {
        ImVec2 at;
        if (obj.hidden || !proj(glm::vec3(worldOf(poses.at(obj.name))[3]), at))
            continue;
        taken.push_back({{at.x - ui(7.5f), at.y - ui(7.5f)}, {at.x + ui(7.5f), at.y + ui(7.5f)}}); // its badge
        if (labelShown(obj))
            wanted.push_back({&obj, at});
    }
    std::stable_partition(wanted.begin(), wanted.end(), [&](const auto &w) { return w.first->name == selected_; });
    const auto overlaps = [&](ImVec2 a, ImVec2 b) { // or would be cut off by the view's edge
        if (a.x < view.origin.x || a.y < view.origin.y || b.x > view.origin.x + view.size.x ||
            b.y > view.origin.y + view.size.y)
            return true;
        return std::any_of(taken.begin(), taken.end(), [&](const auto &r) {
            return a.x < r.second.x && b.x > r.first.x && a.y < r.second.y && b.y > r.first.y;
        });
    };
    labelBoxes_.clear();
    for (const auto &[obj, at] : wanted) {
        const bool chosen = obj->name == selected_;
        const ImVec2 size = font->CalcTextSizeA(font->FontSize, 1e9f, 0, obj->name.c_str());
        const ImVec2 box(labelWidth(size), size.y + ui(6));
        // below, above, right, left of the prop in 2D; up-right, up-left, down-right, down-left in 3D
        const ImVec2 spots[] = {
            labelCorner(at, size, view.plan),
            view.plan ? ImVec2(at.x - box.x * .5f, at.y - ui(11) - box.y) : ImVec2(at.x - ui(8) - box.x, at.y - ui(22)),
            view.plan ? ImVec2(at.x + ui(12), at.y - box.y * .5f) : ImVec2(at.x + ui(8), at.y + ui(8)),
            view.plan ? ImVec2(at.x - ui(12) - box.x, at.y - box.y * .5f) : ImVec2(at.x - ui(8) - box.x, at.y + ui(8))};
        const ImVec2 *spot = nullptr;
        for (const auto &candidate : spots)
            if (!overlaps(candidate, {candidate.x + box.x, candidate.y + box.y})) {
                spot = &candidate;
                break;
            }
        if (!spot && !chosen)
            continue;
        const ImVec2 min = spot ? *spot : spots[0], max{min.x + box.x, min.y + box.y};
        taken.push_back({min, max});
        labelBoxes_.push_back({obj->name, min, max});
        // the theme's window colour and text (a light chart gets light chips) edged in the prop's colour; locked
        // props quieter
        const ImVec4 chip = ImGui::GetStyle().Colors[ImGuiCol_WindowBg];
        const auto color = colorFor(obj->name);
        d->AddRectFilled(min, max,
                         chosen ? ImGui::GetColorU32(p.active) : ImGui::GetColorU32(ImVec4(chip.x, chip.y, chip.z, .94f)),
                         ui(3));
        if (!chosen)
            d->AddRect(min, max, colorU32(color, obj->locked ? .35f : .75f), ui(3), 0, ui(1));
        d->AddCircleFilled({min.x + ui(9), min.y + box.y * .5f}, ui(4), colorU32(color, obj->locked ? .6f : 1.f));
        d->AddText(font, font->FontSize, {min.x + ui(17), min.y + ui(3)},
                   ImGui::GetColorU32(chosen ? p.activeText : obj->locked ? p.muted : p.text), obj->name.c_str());
    }
    // The selected prop's handles: X / Y / Z arrows and the yaw ring, sized to the screen.
    const auto *sel = pm::find(doc_.objects, selected_);
    if (sel && !sel->hidden) {
        const glm::vec3 center(worldOf(poses.at(selected_))[3]);
        ImVec2 c;
        if (proj(center, c)) {
            const float length = ui(90) / pixelsPerMetre(view.viewProjection, view.origin, view.size, center);
            const auto alpha = sel->locked ? 90 : 255;
            const glm::vec3 axes[] = {glm::normalize(glm::vec3(map[0])), glm::normalize(glm::vec3(map[1])), {0, 0, 1}};
            const ImU32 colors[] = {IM_COL32(235, 75, 75, alpha), IM_COL32(90, 215, 110, alpha),
                                    IM_COL32(80, 145, 255, alpha)};
            const Handle names[] = {Handle::X, Handle::Y, Handle::Z};
            for (int i = 0; i < (view.plan ? 0 : 3); ++i) {
                ImVec2 tip;
                if (!proj(center + axes[i] * length, tip))
                    continue;
                const float width = drag_ == names[i] ? ui(5) : ui(3);
                // a shaft and an arrowhead pointing along the axis (a head-on axis is too short to draw)
                const glm::vec2 from(c.x, c.y), to(tip.x, tip.y);
                const float shown = glm::length(to - from);
                if (shown < ui(14))
                    continue;
                const glm::vec2 along = (to - from) / shown, across(-along.y, along.x);
                const float head = ui(drag_ == names[i] ? 16.f : 13.f), half = head * .55f;
                const glm::vec2 base = to - along * head;
                d->AddLine(c, {base.x, base.y}, colors[i], width);
                d->AddTriangleFilled(tip, {base.x + across.x * half, base.y + across.y * half},
                                     {base.x - across.x * half, base.y - across.y * half}, colors[i]);
            }
            ImVec2 previous;
            bool havePrevious = false;
            for (int step = 0; step <= 72; ++step) {
                const float a = float(step) * 2 * float(kPi) / 72;
                ImVec2 point;
                const bool ok = proj(center + glm::vec3(std::cos(a), std::sin(a), 0) * length * .8f, point);
                if (ok && havePrevious)
                    d->AddLine(previous, point, IM_COL32(250, 210, 90, alpha), drag_ == Handle::Yaw ? ui(4) : ui(2.5f));
                previous = point;
                havePrevious = ok;
            }
            const auto &mp = poses.at(selected_);
            char readout[160];
            std::snprintf(readout, sizeof(readout), "%s%s  map x %.2f  y %.2f  z %.2f  yaw %.1f", selected_.c_str(),
                          sel->locked ? " (locked)" : "", mp.x, mp.y, mp.z, mp.yaw);
            const ImVec2 size = small->CalcTextSizeA(small->FontSize, 1e9f, 0, readout);
            const ImVec2 box(c.x - size.x / 2 - ui(6), c.y + ui(18));
            d->AddRectFilled(box, {box.x + size.x + ui(12), box.y + size.y + ui(8)}, IM_COL32(8, 22, 29, 225), ui(4));
            d->AddText(small, small->FontSize, {box.x + ui(6), box.y + ui(4)}, IM_COL32(230, 240, 245, 255), readout);
        }
    }
    // What the pointer does now.
    const char *hint = placing_ ? (origin_.robot ? "Click in the pool to place the map origin  (Esc cancels)"
                                                 : "Click near a line end on a wall to place the AprilTag  (Esc cancels)")
                       : originSelected_ ? "Drag the robot to move the map origin, its ring to turn it  /  Esc deselects"
                       : view.plan ? (sel ? "Drag props to move them, the ring to turn  /  arrows nudge, Q E turn  /  "
                                            "Esc deselects"
                                          : nullptr)
                       : sel ? "Drag the prop or its handles  /  arrows nudge, Q E turn, PgUp PgDn height  /  Esc deselects"
                             : nullptr;
    // Only when it says something the controls strip does not: a mode or a selection. Bottom centre, above the strip.
    if (hint) {
        const ImVec2 size = small->CalcTextSizeA(small->FontSize, 1e9f, 0, hint);
        const ImVec2 box(view.origin.x + (view.size.x - size.x) / 2 - ui(8),
                         view.origin.y + view.size.y - ui(30) - ui(12) - size.y - ui(10));
        d->AddRectFilled(box, {box.x + size.x + ui(16), box.y + size.y + ui(10)}, ImGui::GetColorU32(p.active), ui(4));
        d->AddText(small, small->FontSize, {box.x + ui(8), box.y + ui(5)}, ImGui::GetColorU32(p.activeText), hint);
    }
    d->PopClipRect();
}

void PriorMapEditor::shortcuts(bool viewHovered) {
    if (!active() || ImGui::GetIO().WantTextInput)
        return;
    const bool here = viewHovered || windowFocused_;
    windowFocused_ = false; // the editor's windows set it again as they draw
    if (!here)
        return;
    auto &io = ImGui::GetIO();
    if (ImGui::IsKeyPressed(ImGuiKey_Escape) && drag_ == Handle::None) {
        if (placing_)
            placing_ = false;
        else {
            selected_.clear();
            originSelected_ = false;
        }
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z))
        io.KeyShift ? redo() : undo();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y))
        redo();
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S))
        saveFile();
    auto *o = pm::find(doc_.objects, selected_);
    if (!o || io.KeyCtrl)
        return;
    const double step = io.KeyShift ? .1 : .01, turn = io.KeyShift ? 15 : 1;
    // Held keys repeat; one undo step per run of key presses on the same prop.
    const auto nudge = [&](double dx, double dy, double dz, double dyaw) {
        const auto now = std::chrono::steady_clock::now();
        if (lastNudgeObject_ != selected_ || now - lastNudge_ > std::chrono::milliseconds(800))
            record();
        lastNudge_ = now;
        lastNudgeObject_ = selected_;
        moveSelected(dx, dy, dz, dyaw);
        changed();
    };
    if (!o->locked) {
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))
            nudge(-step, 0, 0, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow))
            nudge(step, 0, 0, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow))
            nudge(0, step, 0, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow))
            nudge(0, -step, 0, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_PageUp))
            nudge(0, 0, step, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_PageDown))
            nudge(0, 0, -step, 0);
        if (ImGui::IsKeyPressed(ImGuiKey_Q))
            nudge(0, 0, 0, turn);
        if (ImGui::IsKeyPressed(ImGuiKey_E))
            nudge(0, 0, 0, -turn);
    }
    if (ImGui::IsKeyPressed(ImGuiKey_L, false)) {
        o->locked = !o->locked;
        saveState();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_H, false)) {
        o->hidden = !o->hidden;
        saveState();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
        record();
        pm::remove(doc_.objects, selected_);
        message_ = "Deleted " + selected_ + " (Ctrl+Z restores it)";
        messageError_ = false;
        selected_.clear();
        changed();
    }
}

// ------------------------------------------------------------------ the window

void PriorMapEditor::drawObjectsWindow(const char *name, const std::function<void()> &insideWindow) {
    if (!objectsOpen_)
        return;
    if (!ImGui::Begin(name, &objectsOpen_)) {
        ImGui::End();
        return;
    }
    if (insideWindow)
        insideWindow();
    windowFocused_ = windowFocused_ || ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    drawFileBar();
    if (loaded()) {
        ImGui::Separator();
        drawObjects();
    }
    ImGui::End();
}

void PriorMapEditor::drawInspectorWindow(const char *name, const std::function<void()> &insideWindow) {
    if (!inspectorOpen_)
        return;
    if (!ImGui::Begin(name, &inspectorOpen_)) {
        ImGui::End();
        return;
    }
    if (insideWindow)
        insideWindow();
    windowFocused_ = windowFocused_ || ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    if (!loaded())
        ImGui::TextDisabled("Open the prior map (Map objects) to edit it.");
    else if (pm::find(doc_.objects, selected_))
        drawInspector();
    else
        drawOriginInspector();
    ImGui::End();
}

void PriorMapEditor::drawViewTools() {
    ImGui::BeginDisabled(!loaded());
    pushActiveColors(placing_);
    if (ImGui::Button(placing_ ? "Placing origin (Esc)###place" : "Place origin###place"))
        placing_ = !placing_;
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(origin_.robot ? "Click in the pool to put the map origin there"
                                        : "Click near a line end on a wall; the AprilTag snaps to it");
    sameLineIfFits(ui(120)); // the toolbar wraps on a narrow view
    ImGui::SetNextItemWidth(ui(120));
    if (ImGui::Combo("##labels", &labels_, "No labels\0Root labels\0All labels\0"))
        saveState();
    sameLineIfFits(ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x +
                   ImGui::CalcTextSize("Hide sim course").x);
    if (ImGui::Checkbox("Hide sim course", &hideCourse_))
        saveState();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Hide the simulator's own course, leaving only the prior map's props");
    ImGui::EndDisabled();
}

// The file: path, open / reload / save, the robot (namespace), undo / redo / add.
void PriorMapEditor::drawFileBar() {
    ImGui::SetNextItemWidth(-1);
    const bool enter = ImGui::InputTextWithHint("##path", "riptide_mapping config.yaml", pathField_,
                                                sizeof(pathField_), ImGuiInputTextFlags_EnterReturnsTrue);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", (std::string(pathField_) + "\nThe robot's prior map (Enter opens it)").c_str());
    if (ImGui::Button("Open") || enter)
        openFile(expandHome(pathField_));
    ImGui::SameLine();
    ImGui::BeginDisabled(!loaded());
    if (ImGui::Button("Reload")) {
        if (dirty_)
            ImGui::OpenPopup("reload_confirm");
        else
            openFile(configPath_);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Read the file again");
    if (ImGui::BeginPopup("reload_confirm")) {
        ImGui::TextUnformatted("Discard the unsaved changes and read the file again?");
        if (ImGui::Button("Discard and reload")) {
            openFile(configPath_);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Keep editing"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SameLine();
    pushActiveColors(dirty_);
    if (ImGui::Button(dirty_ ? "Save*###save" : "Save###save"))
        saveFile();
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Write the poses into the file (Ctrl+S); comments, order and other sections are kept");
    ImGui::EndDisabled();
    if (loaded()) {
        if (doc_.namespaces.size() > 1) {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##ns", doc_.ns.c_str())) {
                for (const auto &ns : doc_.namespaces)
                    if (ImGui::Selectable(ns.c_str(), ns == doc_.ns) && ns != doc_.ns) {
                        try {
                            auto next = pm::load(doc_.text, ns);
                            if (!dirty_)
                                doc_ = std::move(next);
                            else {
                                message_ = "Save or reload before switching robots";
                                messageError_ = true;
                            }
                        } catch (const std::exception &e) {
                            message_ = e.what();
                            messageError_ = true;
                        }
                    }
                ImGui::EndCombo();
            }
        }
        ImGui::BeginDisabled(undo_.empty());
        if (ImGui::Button("Undo"))
            undo();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Ctrl+Z");
        ImGui::SameLine();
        ImGui::BeginDisabled(redo_.empty());
        if (ImGui::Button("Redo"))
            redo();
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Ctrl+Shift+Z");
        ImGui::SameLine();
        if (ImGui::Button("+ Add")) {
            record();
            const auto centre = pm::poolToMap({pool_.length / 2, pool_.width / 2, origin_.z - 1, 0}, origin_);
            selected_ = pm::add(doc_.objects, pm::kMap, centre);
            std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
            changed();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("A new prop in the middle of the pool");
    }
    if (!message_.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, messageError_ ? palette().error : palette().muted);
        ImGui::TextWrapped("%s", message_.c_str());
        ImGui::PopStyleColor();
    }
}

void PriorMapEditor::drawObjects() {
    const float toggle = buttonWidth("Pose");
    ImGui::SetNextItemWidth(-(toggle + ImGui::GetStyle().ItemSpacing.x));
    char filter[64];
    std::snprintf(filter, sizeof(filter), "%s", filter_.c_str());
    if (ImGui::InputTextWithHint("##filter", "Filter objects", filter, sizeof(filter)))
        filter_ = filter;
    ImGui::SameLine();
    pushActiveColors(poseColumns_);
    if (ImGui::Button("Pose")) {
        poseColumns_ = !poseColumns_;
        saveState();
    }
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(poseColumns_ ? "Hide the x / y / z / yaw columns"
                                       : "Show each prop's x / y / z / yaw (relative to its parent, as the file "
                                         "stores it)");
    const auto matches = [&](const pm::Object &o) {
        return filter_.empty() || o.name.find(filter_) != std::string::npos;
    };
    // The name column fits the longest name at its depth in the tree; a narrow panel scrolls sideways (the names stay
    // whole and the column stays put).
    const std::string heading = "Object (" + std::to_string(doc_.objects.size()) + ")";
    float names = ImGui::CalcTextSize(heading.c_str()).x;
    for (const auto &o : doc_.objects) {
        int depth = 0;
        std::set<std::string> seen;
        for (auto *p = pm::find(doc_.objects, o.parent); p && seen.insert(p->name).second;
             p = pm::find(doc_.objects, p->parent))
            ++depth;
        names = std::max(names, float(depth) * ImGui::GetStyle().IndentSpacing + ImGui::GetTreeNodeToLabelSpacing() +
                                    ImGui::CalcTextSize(o.name.c_str()).x);
    }
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, {ui(2), ImGui::GetStyle().CellPadding.y}); // seven columns
    const bool table = ImGui::BeginTable("##tree", poseColumns_ ? 7 : 3,
                                         ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                             ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX);
    ImGui::PopStyleVar();
    if (!table)
        return;
    ImGui::TableSetupScrollFreeze(1, 1); // the header row and the names stay
    ImGui::TableSetupColumn(heading.c_str(), ImGuiTableColumnFlags_WidthFixed, names + ui(6));
    ImGui::TableSetupColumn("Lock", ImGuiTableColumnFlags_WidthFixed, ui(40));
    ImGui::TableSetupColumn("Hide", ImGuiTableColumnFlags_WidthFixed, ui(40));
    if (poseColumns_)
        for (const char *axis : {"x", "y", "z", "yaw"})
            ImGui::TableSetupColumn(axis, ImGuiTableColumnFlags_WidthFixed, ui(axis[1] ? 46.f : 40.f));
    // the header row: the pose headings right-aligned over their right-aligned figures
    if (typeRamp().strong)
        ImGui::PushFont(typeRamp().strong);
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    for (int column = 0; column < ImGui::TableGetColumnCount(); ++column) {
        ImGui::TableSetColumnIndex(column);
        const char *name = ImGui::TableGetColumnName(column);
        if (column >= 3)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.f, ImGui::GetContentRegionAvail().x -
                                                                            ImGui::CalcTextSize(name).x - ui(3)));
        ImGui::TableHeader(name);
    }
    if (typeRamp().strong)
        ImGui::PopFont();
    ruleUnderHeaders();
    std::function<void(const pm::Object &)> row = [&](const pm::Object &o) {
        std::vector<const pm::Object *> children;
        for (const auto &c : doc_.objects)
            if (c.parent == o.name)
                children.push_back(&c);
        const bool show = matches(o) || std::any_of(children.begin(), children.end(), [&](const pm::Object *c) {
                              return matches(*c);
                          });
        if (!show)
            return;
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        // the checkboxes make the row a frame tall: centre the name in it, and highlight the whole row
        ImGui::AlignTextToFramePadding();
        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAllColumns |
                                   ImGuiTreeNodeFlags_AllowOverlap | ImGuiTreeNodeFlags_DefaultOpen;
        if (children.empty())
            flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
        if (o.name == selected_)
            flags |= ImGuiTreeNodeFlags_Selected;
        if (o.hidden || o.locked)
            ImGui::PushStyleColor(ImGuiCol_Text, palette().muted);
        // the prop's colour (its badge / dot in the view) as a dot before the name
        // (the label starts after room for the dot, in whole spaces of this font)
        const std::string room(std::size_t(std::ceil(ui(17) / std::max(1.f, ImGui::CalcTextSize(" ").x))), ' ');
        const bool opened = ImGui::TreeNodeEx(o.name.c_str(), flags, "%s%s", room.c_str(), o.name.c_str());
        {
            const ImVec2 min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
            const float x = min.x + ImGui::GetTreeNodeToLabelSpacing() + ui(5);
            ImGui::GetWindowDrawList()->AddCircleFilled({x, (min.y + max.y) * .5f}, ui(4),
                                                        colorU32(colorFor(o.name), o.hidden ? .4f : 1.f));
        }
        if (o.hidden || o.locked)
            ImGui::PopStyleColor();
        if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
            selected_ = o.name;
            originSelected_ = false;
            std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            focusOn(o.name);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s  (frame %s_frame, parent %s)\nDouble-click to look at it", o.name.c_str(),
                              o.name.c_str(), o.parent == pm::kMap ? "map" : (o.parent + "_frame").c_str());
        ImGui::TableNextColumn();
        ImGui::PushID(o.name.c_str());
        auto *mutableObject = pm::find(doc_.objects, o.name);
        if (iconToggle("##lock", mutableObject->locked, Icon::Lock, palette().accent))
            saveState();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(mutableObject->locked ? "Locked: immovable and click-through in the view (still editable "
                                                      "here). Click to unlock"
                                                    : "Lock: immovable and click-through in the view");
        ImGui::TableNextColumn();
        if (iconToggle("##hide", mutableObject->hidden, Icon::Eye, palette().warn))
            saveState();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(mutableObject->hidden ? "Hidden from the view (still editable here). Click to show"
                                                    : "Hide from the view");
        ImGui::PopID();
        if (poseColumns_) {
            // each cell is the stored pose, editable in place: click, type, Enter (one undo step per edit);
            // frameless until hovered so the table still reads as a table
            ImGui::PushStyleColor(ImGuiCol_Text, o.hidden || o.locked ? palette().muted : palette().text);
            ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {ui(3), ImGui::GetStyle().FramePadding.y});
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0); // cells, not boxes, in an outlined theme
            ImGui::PushID(o.name.c_str()); // one ID per row's cells
            for (int i = 0; i < 4; ++i) {
                ImGui::TableNextColumn();
                auto *editable = pm::find(doc_.objects, o.name);
                double *field[] = {&editable->pose.x, &editable->pose.y, &editable->pose.z, &editable->pose.yaw};
                double value = *field[i];
                ImGui::PushID(i);
                ImGui::SetNextItemWidth(-1);
                // at rest the figure is drawn right-aligned (an input's own text is left-aligned); typing shows it
                const char *format = i == 3 ? "%.1f" : "%.2f";
                const bool editing = ImGui::GetActiveID() == ImGui::GetID("##cell");
                if (!editing)
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 0, 0, 0));
                const bool entered = ImGui::InputDouble("##cell", &value, 0, 0, format,
                                                        ImGuiInputTextFlags_EnterReturnsTrue |
                                                            ImGuiInputTextFlags_CharsScientific);
                if (!editing) {
                    ImGui::PopStyleColor();
                    char figure[32];
                    std::snprintf(figure, sizeof(figure), format, value);
                    const ImVec2 min = ImGui::GetItemRectMin(), max = ImGui::GetItemRectMax();
                    ImGui::GetWindowDrawList()->AddText(
                        {max.x - ui(3) - ImGui::CalcTextSize(figure).x, min.y + ImGui::GetStyle().FramePadding.y},
                        ImGui::GetColorU32(ImGuiCol_Text), figure);
                }
                if (ImGui::IsItemHovered() && !ImGui::IsItemActive())
                    ImGui::SetTooltip("%s %s: click to type a new value (stored relative to %s)", o.name.c_str(),
                                      i == 3 ? "yaw (deg)" : i == 0 ? "x (m)" : i == 1 ? "y (m)" : "z (m)",
                                      o.parent == pm::kMap ? "the map" : (o.parent + "_frame").c_str());
                if (entered && std::isfinite(value) && value != *field[i]) {
                    record();
                    *field[i] = i == 3 ? pm::wrapDegrees(value) : value;
                    changed();
                }
                ImGui::PopID();
            }
            ImGui::PopID();
            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(2);
        }
        if (opened && !children.empty()) {
            for (const auto *c : children)
                row(*c);
            ImGui::TreePop();
        }
    };
    for (const auto &o : doc_.objects)
        if (o.parent == pm::kMap || !pm::find(doc_.objects, o.parent))
            row(o);
    ImGui::EndTable();
}

void PriorMapEditor::drawInspector() {
    auto *o = pm::find(doc_.objects, selected_);
    const auto poses = pm::mapPoses(doc_.objects);
    const auto section = [](const char *title) { sectionTitle(title); };
    // Name (Enter renames; children follow), lock and hide.
    const auto checkboxWidth = [](const char *label) {
        return ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(label).x +
               ImGui::GetStyle().ItemSpacing.x;
    };
    ImGui::SetNextItemWidth(
        std::max(ui(100), ImGui::GetContentRegionAvail().x - checkboxWidth("Locked") - checkboxWidth("Hidden")));
    if (ImGui::InputText("##name", renameField_, sizeof(renameField_), ImGuiInputTextFlags_EnterReturnsTrue)) {
        record();
        std::string error;
        if (pm::rename(doc_.objects, selected_, renameField_, &error)) {
            selected_ = renameField_;
            changed();
        } else {
            undo_.pop_back();
            message_ = error.empty() ? "Not renamed" : error;
            messageError_ = !error.empty();
        }
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Rename (Enter); children are re-parented to the new name");
    ImGui::SameLine();
    if (ImGui::Checkbox("Locked", &o->locked))
        saveState();
    ImGui::SameLine();
    if (ImGui::Checkbox("Hidden", &o->hidden))
        saveState();
    // The parent, then the pose: the file stores it relative to the parent; a child also shows where that puts it.
    section("Parent");
    ImGui::SetNextItemWidth(std::min(ui(220), ImGui::GetContentRegionAvail().x - ImGui::CalcTextSize("Parent").x -
                                                  ImGui::GetStyle().ItemInnerSpacing.x));
    if (ImGui::BeginCombo("Parent", o->parent.c_str())) {
        const auto below = pm::descendants(doc_.objects, o->name);
        std::vector<std::string> options{pm::kMap};
        for (const auto &c : doc_.objects)
            if (c.name != o->name && !below.count(c.name))
                options.push_back(c.name);
        for (const auto &option : options)
            if (ImGui::Selectable(option.c_str(), option == o->parent) && option != o->parent) {
                record();
                std::string error;
                if (pm::reparent(doc_.objects, selected_, option, &error))
                    changed();
                else {
                    undo_.pop_back();
                    message_ = error;
                    messageError_ = true;
                }
                o = pm::find(doc_.objects, selected_);
            }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Re-parenting keeps the prop where it is in the pool");
    const auto fieldRow = [&](double values[4], const char *formatYaw, const std::function<void()> &apply) {
        const char *labels[] = {"x##f", "y##f", "z##f", "yaw##f"};
        // four fields sharing the column, each followed by its label
        const auto &style = ImGui::GetStyle();
        float text = 0;
        for (const char *label : {"x", "y", "z", "yaw"})
            text += ImGui::CalcTextSize(label).x + style.ItemInnerSpacing.x;
        const float width =
            std::max(ui(48), (ImGui::GetContentRegionAvail().x - text - 3 * style.ItemSpacing.x) / 4 - 1);
        for (int i = 0; i < 4; ++i) {
            ImGui::PushID(i);
            if (i)
                ImGui::SameLine();
            const bool edited = inputNumber(labels[i], values[i], i == 3 ? formatYaw : "%.3f", width);
            if (ImGui::IsItemActivated())
                record();
            if (edited) {
                apply();
                changed();
            }
            ImGui::PopID();
        }
    };
    const bool child = o->parent != pm::kMap;
    section(child ? ("Stored: relative to " + o->parent + "_frame").c_str() : "Pose in the map (as stored)");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(child ? "What config.yaml holds for this prop: its pose in its parent's frame"
                                : "What config.yaml holds: the parent is the map, so this is the map pose");
    double rel[4] = {o->pose.x, o->pose.y, o->pose.z, o->pose.yaw};
    ImGui::PushID("relative");
    fieldRow(rel, "%.2f", [&] { o->pose = {rel[0], rel[1], rel[2], pm::wrapDegrees(rel[3])}; });
    ImGui::PopID();
    const auto mp = poses.at(o->name);
    if (child) { // where the parent's pose and this one put it: editable too (the stored pose follows)
        section("Resulting map position");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Where the prop ends up in the map: its parents' poses and its own combined");
        double inMap[4] = {mp.x, mp.y, mp.z, mp.yaw};
        ImGui::PushID("map");
        fieldRow(inMap, "%.2f",
                 [&] { pm::setMapPose(doc_.objects, selected_, {inMap[0], inMap[1], inMap[2], inMap[3]}); });
        ImGui::PopID();
    }
    const auto inPool = pm::mapToPool(mp, origin_);
    ImGui::TextDisabled("In the pool: x %.2f  y %.2f  yaw %.1f  (%.2f m deep)", inPool.x, inPool.y, inPool.yaw,
                        pool_.waterLevel - worldOf(mp)[3].z);
    // Config flags and covariance, as the mapping node reads them.
    section("Config");
    if (ImGui::Checkbox("lock_orientation_to_config", &o->lockOrientation)) {
        dirty_ = true;
    }
    if (ImGui::IsItemActivated())
        record();
    if (ImGui::Checkbox("point_yaw_at_parent", &o->pointYawAtParent))
        dirty_ = true;
    char cls[64];
    std::snprintf(cls, sizeof(cls), "%s", o->cls.c_str());
    ImGui::SetNextItemWidth(ui(150));
    if (ImGui::InputTextWithHint("class", "none", cls, sizeof(cls), ImGuiInputTextFlags_EnterReturnsTrue)) {
        record();
        o->cls = cls;
        changed();
    }
    double cov[4] = {o->covar.x, o->covar.y, o->covar.z, o->covar.yaw};
    ImGui::TextDisabled("Covariance");
    ImGui::PushID("covar");
    fieldRow(cov, "%.3f", [&] { o->covar = {cov[0], cov[1], cov[2], cov[3]}; });
    ImGui::PopID();
    // Swap with another prop (e.g. the two gate sides), classes (fire / blood).
    section("Swap");
    static int other = 0;
    std::vector<std::string> names;
    for (const auto &c : doc_.objects)
        if (c.name != o->name && c.parent == o->parent)
            names.push_back(c.name);
    if (names.empty())
        ImGui::TextDisabled("No sibling under the same parent");
    else {
        other = std::clamp(other, 0, int(names.size()) - 1);
        ImGui::SetNextItemWidth(ui(180));
        if (ImGui::BeginCombo("##other", names[std::size_t(other)].c_str())) {
            for (int i = 0; i < int(names.size()); ++i)
                if (ImGui::Selectable(names[std::size_t(i)].c_str(), i == other))
                    other = i;
            ImGui::EndCombo();
        }
        sameLineIfFits(buttonWidth("Swap poses"));
        if (ImGui::Button("Swap poses")) {
            record();
            std::string error;
            if (pm::swapPoses(doc_.objects, selected_, names[std::size_t(other)], &error))
                changed();
            else {
                undo_.pop_back();
                message_ = error;
                messageError_ = true;
            }
        }
        sameLineIfFits(buttonWidth("Swap classes"));
        const auto *b = pm::find(doc_.objects, names[std::size_t(other)]);
        ImGui::BeginDisabled(o->cls.empty() || !b || b->cls.empty());
        if (ImGui::Button("Swap classes")) {
            record();
            pm::swapClasses(doc_.objects, selected_, names[std::size_t(other)]);
            changed();
        }
        ImGui::EndDisabled();
    }
    section("Object");
    if (ImGui::Button("Add child")) {
        record();
        auto at = poses.at(o->name);
        at.x += .5;
        at.y += .5;
        selected_ = pm::add(doc_.objects, o->name, at);
        std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
        changed();
        return;
    }
    ImGui::SameLine();
    if (ImGui::Button("Duplicate")) {
        record();
        selected_ = pm::duplicate(doc_.objects, o->name);
        std::snprintf(renameField_, sizeof(renameField_), "%s", selected_.c_str());
        changed();
        return;
    }
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, palette().dangerPressed);
    ImGui::PushStyleColor(ImGuiCol_Text, palette().dangerText);
    if (ImGui::Button("Delete"))
        ImGui::OpenPopup("delete_prop");
    ImGui::PopStyleColor(2);
    if (ImGui::BeginPopup("delete_prop")) {
        ImGui::Text("Delete %s?", selected_.c_str());
        ImGui::TextDisabled("Its children move to the map where they are.");
        if (ImGui::Button("Delete")) {
            record();
            pm::remove(doc_.objects, selected_);
            selected_.clear();
            changed();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::Spacing();
    if (ImGui::Button("Deselect (Esc)"))
        selected_.clear();
}

void PriorMapEditor::drawOriginInspector() {
    sectionTitle("Map origin");
    ImGui::TextWrapped("%s", origin_.robot ? "Robot frame: the map origin is the sub's start pose, anywhere in the pool. "
                                                             "The robot is drawn there: drag it in the view to move "
                                                             "the origin, its ring to turn it."
                                           : "AprilTag: the map origin is the tag on a wall, at a line / wall "
                                             "intersection, +X into the pool.");
    int kind = origin_.robot ? 1 : 0;
    if (pins::Switch("Origin##origin_kind", &kind, {"AprilTag", "Robot frame"})) {
        const bool robot = kind == 1;
        {
            // Back to where the last origin of that kind was; the first time, from here: a robot frame keeps the
            // heading, a tag snaps to the nearest spot facing +X out of the wall.
            const auto remembered = origins_.find(pool_.id + (robot ? "/robot" : "/tag"));
            auto next = origin_;
            next.robot = robot;
            if (remembered != origins_.end())
                next = remembered->second;
            else if (robot) {
                next.yawOffset = origin_.yaw();
                next.basePhi = 0;
            } else {
                next.yawOffset = 0;
                if (const auto spot = pm::nearestSpot(pm::tagSpots(pool_.length, pool_.width, pool_.lines),
                                                      origin_.x, origin_.y, 1e9)) {
                    next.x = spot->x;
                    next.y = spot->y;
                    next.basePhi = spot->phi;
                    next.wall = spot->wall;
                }
            }
            setOrigin(next);
        }
    }
    // the action on its own line: the switch above is a mode, this places it
    pushActiveColors(placing_);
    if (ImGui::Button(placing_ ? "Placing... (Esc)###place" : "Place origin in the view###place"))
        placing_ = !placing_;
    popActiveColors();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip(origin_.robot ? "Click in the pool view to put the origin there"
                                        : "Click near a line end on a wall in the pool view; the tag snaps to it");
    double pos[2] = {origin_.x, origin_.y};
    ImGui::TextDisabled("Pool position (m)");
    for (int i = 0; i < 2; ++i) {
        ImGui::PushID(i);
        if (i)
            ImGui::SameLine();
        const bool edited = inputNumber(i ? "y##o" : "x##o", pos[i]);
        if (edited) {
            auto next = origin_;
            (i ? next.y : next.x) = pos[i];
            setOrigin(next);
        }
        ImGui::PopID();
    }
    ImGui::TextDisabled("Heading %.1f deg%s", origin_.yaw(),
                        origin_.robot ? "" : (std::string("  (") + origin_.wall + " wall)").c_str());
    if (ImGui::Button("Turn +90")) {
        auto next = origin_;
        next.yawOffset = pm::wrapDegrees(next.yawOffset + 90);
        setOrigin(next);
    }
    ImGui::SameLine();
    if (ImGui::Button("Turn -90")) {
        auto next = origin_;
        next.yawOffset = pm::wrapDegrees(next.yawOffset - 90);
        setOrigin(next);
    }
    ImGui::SameLine();
    double offset = origin_.yawOffset;
    if (inputNumber("yaw offset", offset, "%.2f")) {
        auto next = origin_;
        next.yawOffset = pm::wrapDegrees(offset);
        setOrigin(next);
    }
    if (ImGui::Checkbox("Pin props to the pool", &poolLock_))
        saveState();
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Moving or turning the origin keeps the props where they are in the pool\n"
                          "(their map poses change) instead of carrying them along");
    if (ImGui::Button("Reset to the scenario's origin")) {
        setOrigin(pool_.scenarioOrigin);
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Where the scenario has the map (its calibration board placement)");
    ImGui::Spacing();
    ImGui::TextDisabled("Select a prop in the list or the view to edit it.");
}
} // namespace nereus::ros_viewer::host
