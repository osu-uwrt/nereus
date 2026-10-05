// The prior-map editor (Dead Reckoning in 3D): the robot's riptide_mapping config.yaml laid out in the pool view.
// Props are drawn as their RViz meshes at their map poses; the operator selects, drags, turns and types poses,
// places the map origin (an AprilTag at a line / wall intersection, or a free robot-frame pose) and saves the
// file with its comments and layout untouched. The pool view is the scenario's world, which is the map frame:
// props are drawn through the editor's origin relative to the scenario's, so moving the origin moves them.
#pragma once
#include "mapping_markers.hpp"
#include "prior_map.hpp"
#include "visual_state.hpp"
#include <chrono>
#include <filesystem>
#include <functional>
#include <glm/glm.hpp>
#include <imgui.h>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {
class PriorMapEditor {
  public:
    // What the editor needs of the pool (set when the scenario loads).
    struct Pool {
        std::string id; // the pool pack (the origin is kept per pool)
        glm::mat4 poolToWorld{1};
        float length = 50, width = 22.86f, waterLevel = 0; // waterLevel: world z of the surface
        std::vector<prior_map::Line> lines; // floor lines, pool coordinates
        prior_map::Origin scenarioOrigin;   // where the scenario has the map
    };
    // The pool view this frame.
    struct View {
        glm::mat4 viewProjection{1};
        glm::vec2 origin{0}, size{1};
        bool hovered = false;                  // the pointer is on the view, nothing in the way
        bool plan = false;                     // the 2D top-down view: moves in x / y only (no height handle)
        std::optional<glm::vec3> pointer;      // the scene point under the pointer (last frame's depth), if any
    };
    PriorMapEditor(std::filesystem::path stateDirectory, std::filesystem::path defaultConfig);

    void setPool(const Pool &);
    void setMeshes(std::vector<MappingMarker> markers); // by TF frame: "<name>_frame"
    // Each meshed prop's extent in its own frame (the mesh's bounds through its marker pose): what a click on the
    // prop hits. By TF frame.
    struct Extent {
        glm::vec3 low{0}, high{0};
    };
    void setExtents(std::map<std::string, Extent> byFrame) {
        extents_ = std::move(byFrame);
    }
    // The Map workspace is shown: the props are drawn in the pool view and take its pointer.
    void setActive(bool on) {
        active_ = on;
    }
    bool active() const {
        return active_ && loaded();
    }
    bool &objectsOpen() {
        return objectsOpen_;
    }
    bool &inspectorOpen() {
        return inspectorOpen_;
    }
    bool hidesCourse() const {
        return active() && hideCourse_;
    }
    bool loaded() const {
        return !doc_.text.empty();
    }
    bool dirty() const {
        return dirty_;
    }
    void save() {
        saveFile();
    }
    const std::filesystem::path &file() const {
        return configPath_;
    }
    const prior_map::Document &document() const {
        return doc_;
    }
    const std::string &selected() const {
        return selected_;
    }

    // The Map workspace's two windows: the file and its object tree; the selected prop (or the map origin).
    // `insideWindow` runs right after each window begins (the host's context menu).
    void drawObjectsWindow(const char *name, const std::function<void()> &insideWindow = {});
    void drawInspectorWindow(const char *name, const std::function<void()> &insideWindow = {});
    // The pool view toolbar's map tools: place the origin, labels, hide the simulator's course.
    void drawViewTools();
    void addMarkers(std::vector<MarkerDraw> &) const;
    // Pointer input in the pool view; true when the editor used it (the camera must not).
    bool input(const View &);
    void drawOverlay(const View &, ImFont *small) const;
    void shortcuts(bool viewHovered);
    // A prop double-clicked (view or list): where the camera should look, once.
    // A robot-frame origin is the robot's start pose: the robot is drawn there (base_link), frozen, and dragging
    // it moves the origin. None with an AprilTag origin.
    const prior_map::Origin &origin() const {
        return origin_;
    }
    void setOrigin(const prior_map::Origin &); // one undo step
    std::optional<glm::mat4> robotStart() const {
        if (!active() || !origin_.robot)
            return std::nullopt;
        return worldFromMap();
    }
    void setPlan(bool plan) { // the 2D view: stand-alone props without a mesh are badges, not boxes
        plan_ = plan;
    }
    std::optional<glm::vec3> takeFocus() {
        auto out = focus_;
        focus_.reset();
        return out;
    }

  private:
    struct Snapshot {
        std::vector<prior_map::Object> objects;
        prior_map::Origin origin;
    };
    enum class Handle { None, Body, X, Y, Z, Yaw, OriginBody, OriginYaw, OriginX, OriginY };

    // files
    void openFile(const std::filesystem::path &);
    void saveFile();
    void loadState();
    void saveState() const;
    std::filesystem::path statePath() const;
    // edits (each records an undo step)
    void record();
    void undo();
    void redo();
    void changed();
    void originMoved();
    void moveSelected(double dx, double dy, double dz, double dyaw); // pool frame
    // geometry
    glm::mat4 worldFromMap() const;
    glm::mat4 worldOf(const prior_map::Pose &mapPose) const;
    prior_map::Pose mapFromWorld(const glm::vec3 &world, double yawWorld) const;
    std::string pick(const View &, const glm::vec2 &mouse) const;
    bool labelShown(const prior_map::Object &) const;
    // How a prop is drawn: its mesh, as a frame on a meshed assembly (a small dot), or stand-alone without a mesh
    // (a box in 3D, a badge with its heading in 2D).
    enum class Look { Mesh, OnAssembly, Bare };
    Look lookOf(const prior_map::Object &) const;
    std::optional<Extent> extentOf(const prior_map::Object &) const; // none: a frame on a meshed assembly
    bool onRobotOrigin(const View &, const glm::vec2 &mouse) const; // robot-frame origin under the pointer
    bool onOriginRing(const View &, const glm::vec2 &mouse) const;
    // The map axis (0 x, 1 y) whose origin arrow is under the pointer, or -1 (robot-frame origins drag by them).
    int onOriginArrow(const View &, const glm::vec2 &mouse) const;
    float handleLength(const View &, const glm::vec3 &center) const; // metres for ~90 px on screen
    // window parts
    void drawFileBar();
    void drawObjects();
    void drawInspector();
    void drawOriginInspector();

    std::filesystem::path stateDirectory_, configPath_;
    char pathField_[512]{};
    prior_map::Document doc_;
    prior_map::Origin origin_;
    Pool pool_;
    std::map<std::string, MappingMarker> meshes_;
    std::map<std::string, Extent> extents_;
    mutable ImFont *labelFont_ = nullptr; // the overlay's (labels are click targets)
    struct LabelBox {
        std::string name;
        ImVec2 min, max;
    };
    mutable std::vector<LabelBox> labelBoxes_; // last drawn label boxes, for picking
    std::vector<Snapshot> undo_, redo_;
    std::string selected_, message_, filter_;
    bool messageError_ = false, active_ = false, objectsOpen_ = true, inspectorOpen_ = true, hideCourse_ = true,
         poolLock_ = false, dirty_ = false;
    bool placing_ = false;
    std::map<std::string, prior_map::Origin> origins_; // by pool id: where the map is in each pool
    void useOrigin();                                   // origin_ for pool_ (its saved one, else the scenario's)
    void rememberOrigin(); // origins_ for the pool, and for its kind (AprilTag / robot) to switch back to
    bool plan_ = false;    // the pool view is the 2D top-down one
    int labels_ = 1;           // 0 none, 1 roots, 2 all
    bool poseColumns_ = true;  // the object table's x / y / z / yaw columns (relative to the parent, as stored)
    // gesture
    Handle drag_ = Handle::None;
    bool pressed_ = false;
    glm::vec2 pressMouse_{0};
    glm::vec3 grab_{0};
    prior_map::Pose startMap_; // the selected object's map pose at the gesture start
    prior_map::Origin startOrigin_;               // the origin at an origin gesture's start
    std::vector<prior_map::Object> startObjects_; // and the props (pinned to the pool, they move back)
    bool originSelected_ = false;                 // the robot-frame origin is selected (its ring is shown)
    double grabAngle_ = 0;
    // keyboard nudges: one undo step per run of presses on one prop
    std::chrono::steady_clock::time_point lastNudge_{};
    std::string lastNudgeObject_;
    bool windowFocused_ = false;
    std::optional<glm::vec3> focus_;
    void focusOn(const std::string &name);
    char renameField_[128]{};
};
} // namespace nereus::ros_viewer::host
