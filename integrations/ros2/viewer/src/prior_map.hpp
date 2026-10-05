// The robot's prior map (riptide_mapping config.yaml, the Dead Reckoning tool's file): task props in a parent /
// child frame tree, each pose relative to its parent (x, y, z in metres, yaw in degrees about +Z). Loading,
// editing and a comment-preserving save; the map origin (an AprilTag on a pool wall, or a free robot-frame pose)
// relating the map to the pool. No GL or ImGui: the editor window draws on top of this.
#pragma once
#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace nereus::ros_viewer::host::prior_map {
inline constexpr const char *kMap = "map";

// x, y, z (m) and yaw (deg, CCW about +Z, wrapped to (-180, 180]).
struct Pose {
    double x = 0, y = 0, z = 0, yaw = 0;
};
double wrapDegrees(double degrees);
Pose compose(const Pose &parent, const Pose &childInParent); // parent's map pose and a child's pose under it
Pose decompose(const Pose &parent, const Pose &childInMap);  // a map pose under `parent`

struct Covariance {
    double x = 1, y = 1, z = 1, yaw = 1;
};
struct Object {
    std::string name;          // the frame is "<name>_frame"
    std::string parent = kMap; // "map" or another object's name
    Pose pose;                 // relative to the parent
    Covariance covar;
    bool lockOrientation = false;  // lock_orientation_to_config (kept, not interpreted)
    bool pointYawAtParent = false; // point_yaw_at_parent (kept, not interpreted)
    std::string cls;               // class (kept); empty: none
    // Editor only (never written to the config): immovable and click-through in the view / not drawn.
    bool locked = false, hidden = false;
};

// One `<ns>/riptide_mapping2: ros__parameters: init_data:` section of a config file.
struct Document {
    std::string text;                     // the file as loaded (or last saved)
    std::string ns;                       // the namespace being edited
    std::vector<std::string> namespaces;  // every namespace with init_data (liltank skipped)
    std::vector<Object> objects;          // in file order
    std::vector<std::string> loadedNames; // names the file had (a name gone from `objects` is a deletion)
};
// Not shown or edited, kept untouched on save.
bool deprecated(const std::string &name);
// Parses `text`; preferredNs if present, else the first talos namespace, else the first. Throws on bad input.
Document load(const std::string &text, const std::string &preferredNs = {});
// The new file text for `doc.objects`: only changed values are rewritten, comments, order, other namespaces
// and unrelated keys stay byte-identical. The result is re-parsed and checked; throws if it would not load
// back as `doc.objects` or would change anything else.
std::string save(const Document &doc);

// ------------------------------------------------------------------ the tree

const Object *find(const std::vector<Object> &, const std::string &name);
Object *find(std::vector<Object> &, const std::string &name);
// Map-frame pose of every object (a missing parent or a cycle: the pose is taken as map-relative).
std::map<std::string, Pose> mapPoses(const std::vector<Object> &);
std::set<std::string> descendants(const std::vector<Object> &, const std::string &name);
// The pose under `parent` that puts an object at `mapPose` (identity for the map).
Pose relativeUnder(const std::string &parent, const std::map<std::string, Pose> &poses, const Pose &mapPose);
std::string uniqueName(const std::vector<Object> &, const std::string &base);

// Edits, as Dead Reckoning: false (and no change) when not allowed; `error` says why.
bool reparent(std::vector<Object> &, const std::string &name, const std::string &parent, std::string *error = nullptr);
bool rename(std::vector<Object> &, const std::string &name, const std::string &next, std::string *error = nullptr);
void remove(std::vector<Object> &, const std::string &name); // children move to the map, where they are
// Exchange two objects' map poses, each re-expressed under its own parent.
bool swapPoses(std::vector<Object> &, const std::string &a, const std::string &b, std::string *error = nullptr);
void swapClasses(std::vector<Object> &, const std::string &a, const std::string &b);
void setMapPose(std::vector<Object> &, const std::string &name, const Pose &mapPose);
std::string add(std::vector<Object> &, const std::string &parent, const Pose &mapPose);
std::string duplicate(std::vector<Object> &, const std::string &name);

// ------------------------------------------------------------------ the map origin

// Where the map frame sits in the pool (pool frame: origin at a corner, +X along the length, +Y across, +Z up,
// z = 0 at the water surface). AprilTag: on a wall at a line / wall intersection, +X into the pool; robot: free.
struct Origin {
    double x = 0, y = 0, z = 0;
    double basePhi = 0;   // wall-normal heading into the pool (deg); a robot origin keeps its heading here as 0
    double yawOffset = 0; // fine turn on top (deg)
    char wall = 'W';      // N, S, E or W
    bool robot = false;
    double yaw() const {
        return wrapDegrees(basePhi + yawOffset);
    }
};
Pose mapToPool(const Pose &inMap, const Origin &);
Pose poolToMap(const Pose &inPool, const Origin &);
// Moving the origin with the objects pinned to the pool: the map roots re-expressed under the new origin.
void keepInPool(std::vector<Object> &, const Origin &from, const Origin &to);

// A floor line, pool coordinates (m).
struct Line {
    double x0, y0, x1, y1;
};
struct TagSpot {
    double x, y, phi;
    char wall;
};
// AprilTag spots: where each floor line, carried on along its axis, meets the walls, and the four corners (twice,
// once per wall). Lines shorter than `minimumLength` (T bars) are skipped.
std::vector<TagSpot> tagSpots(double length, double width, const std::vector<Line> &, double minimumLength = 1.5);
// The spot nearest (x, y) within `reach` (two at one corner: the wall the point is closer to).
std::optional<TagSpot> nearestSpot(const std::vector<TagSpot> &, double x, double y, double reach = 3);
} // namespace nereus::ros_viewer::host::prior_map
