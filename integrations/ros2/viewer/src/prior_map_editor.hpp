// The prior-map editor (Dead Reckoning in 3D): the robot's riptide_mapping config.yaml laid out in the pool view.
// Props are drawn as their RViz meshes at their map poses; the operator selects, drags, turns and types poses,
// places the map origin (an AprilTag at a line / wall intersection, or a free robot-frame pose) and saves the
// file with its comments and layout untouched. The pool view is the scenario's world, which is the map frame:
// props are drawn through the editor's origin relative to the scenario's, so moving the origin moves them.
//
// It edits a second layer the same way: the Sim course, the simulator's truth (the scenario's task placements,
// the tasks' loose objects and the fixed run options). One layer is edited at a time; the other is drawn for
// reference. Copy buttons carry poses and classes from one to the other through the host's links (task id ->
// the prop at the task's origin).
#pragma once
#include "file_picker.hpp"
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
#include <set>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host {

// Owned by the viewer host; all calls on the UI thread. `stateDirectory` keeps per-config editor state (origins,
// locks, display options); `defaultConfig` is opened at start if it exists.
class PriorMapEditor {
  public:
    // What the editor needs of the pool (set when the scenario loads).
    struct Pool {
        std::string id; // the pool pack (the origin is kept per pool)
        glm::mat4 poolToWorld{1};
        float length = 50, width = 22.86f, waterLevel = 0; // waterLevel: world z of the surface
        std::vector<prior_map::Line> lines;                // floor lines, pool coordinates
        prior_map::Origin scenarioOrigin;                  // where the scenario has the map
    };

    // The pool view this frame.
    struct View {
        glm::mat4 viewProjection{1};
        glm::vec2 origin{0}, size{1};
        bool hovered = false;             // the pointer is on the view, nothing in the way
        bool plan = false;                // the 2D top-down view: moves in x / y only (no height handle)
        std::optional<glm::vec3> pointer; // the scene point under the pointer (last frame's depth), if any
    };

    // The simulator's course, from the loaded scenario: each task at its world placement (a root; "map" is the
    // world here) with its loose objects (rigid-body frames) under it, in the task's frame, and the tasks pack's
    // fixed choice run options.
    struct Course {
        std::string label;              // the scenario, for the bar
        std::filesystem::path scenario; // the scenario pack Save writes (empty: none found; Save is off)
        std::vector<prior_map::Object> objects;
        std::map<std::string, std::string> frameOf; // loose object name -> its task frame id
        struct Option {
            std::string key, value;
            std::vector<std::string> choices;
        };
        std::vector<Option> options;
        std::map<std::string, MappingMarker> meshes; // by "<name>_frame": loose objects' meshes; a task's has no
                                                     // path (the scene draws the task, moved by courseMoves())
        std::map<std::string, glm::vec3> low, high;  // by "<name>_frame": what a click on it hits, in its frame
    };
    // Which prior map prop stands for which task (at the task's origin), for the copy buttons. A loose object or
    // a class matches by name: a prop named like the object, or a prop whose class sets the `<name>_class` option.
    struct Links {
        std::map<std::string, std::string> tasks; // task id -> prop name
        std::set<std::string> floating;           // tasks whose height is never copied (the octagon floats)
    };
    // Saving the Sim course: the host writes the edit (set-course JSON) into `scenario` and restarts the
    // simulator on it, then reports back through courseSaved().
    using CourseSaver = std::function<void(const std::filesystem::path &scenario, const std::string &edit)>;

    PriorMapEditor(std::filesystem::path stateDirectory, std::filesystem::path defaultConfig);

    void setCourse(Course);
    void setCourseLinks(Links links) {
        links_ = std::move(links);
    }
    void setCourseSaver(CourseSaver saver) {
        courseSaver_ = std::move(saver);
    }
    void courseSaved(const std::string &error); // empty: written
    // The scenario pack Save writes, once known (the simulator's supervisor names it after the course is set).
    void setCourseScenario(std::filesystem::path scenario);
    const std::filesystem::path &courseScenario() const {
        return courseScenario_;
    }
    // Whether the simulator's course is drawn at all (not on a robot whose course comes from mapping).
    void setCourseShown(bool shown) {
        courseShown_ = shown;
    }
    bool savingCourse() const {
        return savingCourse_;
    }
    // The Sim course's unsaved changes as set-course JSON (empty: none).
    std::string courseEdit() const;
    // Which layer is edited: the robot's map (false) or the Sim course (true).
    bool editingCourse() const {
        return course_;
    }
    void editCourse(bool on);
    // Copy into the edited layer from the other one (one undo step): the linked tasks' poses, their loose objects
    // and the classes. Returns what was copied, for the status line.
    std::string copyFromOtherLayer();

    void setPool(const Pool &);
    // The X11 window the file picker's dialog stays above (Browse in the file bar).
    void setDialogParent(unsigned long x11Window) {
        picker_.setParent(x11Window);
    }
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

    // The two windows' open flags (for the host's window menu).
    bool &objectsOpen() {
        return objectsOpen_;
    }
    bool &inspectorOpen() {
        return inspectorOpen_;
    }
    // The simulator's own course should be hidden (the robot's map is edited and "Hide sim course" is on).
    bool hidesCourse() const {
        return active() && hideCourse_ && !course_;
    }
    // The Sim course is drawn as edited (Map workspace, not hidden): its tasks moved by courseMoves() and its loose
    // objects by the editor (the simulator's own props are not drawn).
    bool drawsCourse() const {
        return active_ && courseShown_ && !courseDoc().text.empty() && !hidesCourse();
    }
    // Each moved task's correction for the scene: world from its edited placement, times the inverse of the one
    // the scene was built with.
    std::map<std::string, glm::mat4> courseMoves() const;
    bool loaded() const {
        return !doc_.text.empty();
    }
    bool dirty() const { // either layer
        return dirty_ || other_.dirty;
    }
    bool mapDirty() const {
        return course_ ? other_.dirty : dirty_;
    }
    bool courseDirty() const {
        return course_ ? dirty_ : other_.dirty;
    }
    const std::string &courseLabel() const {
        return courseLabel_;
    }

    void save(); // both layers' unsaved changes (the Sim course through the host's saver)
    void open(const std::filesystem::path &file) { // a riptide_mapping config.yaml (errors go to the status line)
        editCourse(false);
        openFile(file);
    }
    void reload() { // the file again, dropping unsaved edits
        editCourse(false);
        openFile(configPath_);
    }

    // The editor's last status line ("Saved ...", "Loaded ...", or what went wrong).
    const std::string &message() const {
        return message_;
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
    // The props' meshes (boxes for mesh-less stand-alone props in 3D) for the pool view's renderer.
    void addMarkers(std::vector<MarkerDraw> &) const;

    // Pointer input in the pool view; true when the editor used it (the camera must not).
    bool input(const View &);
    // The 2D layer over the pool view: origin, badges, labels, the selection's handles and a hint line.
    void drawOverlay(const View &, ImFont *small) const;
    // Keyboard shortcuts (undo / redo / save, nudges, lock / hide / delete) when the view or a window has focus.
    void shortcuts(bool viewHovered);

    const prior_map::Origin &origin() const {
        return origin_;
    }
    void setOrigin(const prior_map::Origin &); // one undo step

    // A robot-frame origin is the robot's start pose: the robot is drawn there (base_link), frozen, and dragging
    // it moves the origin. None with an AprilTag origin.
    std::optional<glm::mat4> robotStart() const {
        if (!active() || !origin_.robot)
            return std::nullopt;
        return worldFromMap();
    }
    void setPlan(bool plan) { // the 2D view: stand-alone props without a mesh are badges, not boxes
        plan_ = plan;
    }

    // A prop double-clicked (view or list): where the camera should look, once.
    std::optional<glm::vec3> takeFocus() {
        auto out = focus_;
        focus_.reset();
        return out;
    }

  private:
    // One undo / redo step: the objects, the origin and (Sim course) the option values.
    struct Snapshot {
        std::vector<prior_map::Object> objects;
        prior_map::Origin origin;
        std::vector<Course::Option> options;
    };

    // What one layer is: swapped into the members below while it is edited, kept in other_ while it is not.
    struct Layer {
        prior_map::Document doc;
        std::vector<Snapshot> undo, redo;
        std::string selected, message;
        bool dirty = false, messageError = false;
        prior_map::Origin origin;
        std::map<std::string, MappingMarker> meshes;
        std::map<std::string, Extent> extents;
    };
    void swapLayers();
    const prior_map::Document &courseDoc() const {
        return course_ ? doc_ : other_.doc;
    }

    // What a pointer gesture is dragging: the selected prop (body, axis arrows, yaw ring) or the robot-frame origin.
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
    bool onRobotOrigin(const View &, const glm::vec2 &mouse) const;  // robot-frame origin under the pointer
    bool onOriginRing(const View &, const glm::vec2 &mouse) const;
    // The map axis (0 x, 1 y) whose origin arrow is under the pointer, or -1 (robot-frame origins drag by them).
    int onOriginArrow(const View &, const glm::vec2 &mouse) const;
    float handleLength(const View &, const glm::vec3 &center) const; // meters for ~90 px on screen

    // the Sim course
    void saveCourse();
    void drawCourseBar();
    void drawCourseInspector();
    void addLayerMarkers(const prior_map::Document &, const std::map<std::string, MappingMarker> &meshes,
                         const prior_map::Origin &, bool reference, std::vector<MarkerDraw> &) const;
    glm::mat4 worldFrom(const prior_map::Origin &) const;

    // window parts
    void drawFileBar();
    void drawObjects();
    void drawInspector();
    void drawOriginInspector();

    // The file and its contents.
    std::filesystem::path stateDirectory_, configPath_;
    char pathField_[512]{}; // the Objects window's path field
    FilePicker picker_;     // Browse: the desktop's file picker
    prior_map::Document doc_;
    prior_map::Origin origin_;
    Pool pool_;
    std::map<std::string, MappingMarker> meshes_;
    std::map<std::string, Extent> extents_;

    // Labels drawn last frame (drawOverlay() is const but records them for pick()).
    mutable ImFont *labelFont_ = nullptr; // the overlay's (labels are click targets)
    struct LabelBox {
        std::string name;
        ImVec2 min, max;
    };
    mutable std::vector<LabelBox> labelBoxes_; // last drawn label boxes, for picking

    // The layers: the one not edited, and the Sim course's own state.
    Layer other_;
    bool course_ = false, savingCourse_ = false, courseShown_ = true;
    Links links_;
    CourseSaver courseSaver_;
    std::string courseLabel_;
    std::filesystem::path courseScenario_;
    std::map<std::string, std::string> frameOf_;               // loose object -> task frame
    std::vector<Course::Option> options_, savedOptions_;       // the course's fixed options: now, as last saved
    std::vector<prior_map::Object> savedCourse_, sceneCourse_; // as last saved; as the scene was built

    // Edit state and options.
    std::vector<Snapshot> undo_, redo_;
    std::string selected_, message_, filter_;
    bool messageError_ = false, active_ = false, objectsOpen_ = true, inspectorOpen_ = true, hideCourse_ = true,
         poolLock_ = false, dirty_ = false;
    bool placing_ = false;
    std::map<std::string, prior_map::Origin> origins_; // by pool id: where the map is in each pool
    void useOrigin();                                  // origin_ for pool_ (its saved one, else the scenario's)
    void rememberOrigin();    // origins_ for the pool, and for its kind (AprilTag / robot) to switch back to
    bool plan_ = false;       // the pool view is the 2D top-down one
    int labels_ = 1;          // 0 none, 1 roots, 2 all
    bool poseColumns_ = true; // the object table's x / y / z / yaw columns (relative to the parent, as stored)

    // gesture
    Handle drag_ = Handle::None;
    bool pressed_ = false;
    glm::vec2 pressMouse_{0};
    glm::vec3 grab_{0};
    prior_map::Pose startMap_;                    // the selected object's map pose at the gesture start
    prior_map::Origin startOrigin_;               // the origin at an origin gesture's start
    std::vector<prior_map::Object> startObjects_; // and the props (pinned to the pool, they move back)
    bool originSelected_ = false;                 // the robot-frame origin is selected (its ring is shown)
    double grabAngle_ = 0;

    // keyboard nudges: one undo step per run of presses on one prop
    std::chrono::steady_clock::time_point lastNudge_{};
    std::string lastNudgeObject_;

    bool windowFocused_ = false; // an editor window had focus this frame (keyboard shortcuts apply)
    std::optional<glm::vec3> focus_;
    void focusOn(const std::string &name);
    char renameField_[128]{};
};

} // namespace nereus::ros_viewer::host
