#pragma once
// Renderer scene composition from a resolved scenario: pool, task static visuals (with UV
// cutouts and per-visual texture overrides), robot visuals, plus caller-posed dynamic instances.
// One implementation shared by the ROS viewer and the simulator's cameras; it mirrors
// python/src/robotics_platform/pack_cameras.py, which stays the reference.
//
// Robot poses are the robot frame root (`robot.frames.root`, the physics COM) in the scenario
// world frame. Meshes are loaded once and shared (immutable) by every Scene composed.
// The object is immutable after construction except for the mesh cache, which is locked, so
// compose()/mesh() may be called from several threads.
#include <robotics/rendering/scene.hpp>
#include <robotics/session/scenario.hpp>
#include <robotics/spatial/frames.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace robotics::pack_scene {
using Matrix4d = Eigen::Matrix4d;

struct Options {
    // true: an unreadable asset / inconsistent cutout throws (cameras reject unsupported
    // content); false: it is skipped and recorded in warnings() (interactive viewer).
    bool strict = true;
};

Matrix4d toMatrix(const spatial::Pose &);
// Pack placement {position_m, orientation_wxyz} (validated like the native pose).
spatial::Pose placement(const session::Json &item);
// Upright pose: position_m and yaw_deg about +Z.
spatial::Pose upright(const session::Json &position_m, double yaw_deg);

struct RobotVisual {
    std::string asset, frame;
    std::shared_ptr<const rendering::MeshAsset> mesh; // null when skipped (non-strict)
    Matrix4d root_from_frame = Matrix4d::Identity();
    Matrix4d frame_from_asset = Matrix4d::Identity(); // the visual's placement in its frame
    Matrix4d rootFromAsset() const {
        return root_from_frame * frame_from_asset;
    }
};
// Replaces the reset placement of robot visual `index` (moving claw jaws, spinning rotors).
struct RobotOverride {
    std::size_t index = 0;
    Matrix4d root_from_asset = Matrix4d::Identity();
};
// A moving (rigid_body) prop that names a display mesh; the caller supplies its pose.
struct PropVisual {
    std::string task, prop, asset;
    std::shared_ptr<const rendering::MeshAsset> mesh;
    Matrix4d world_from_asset_at_reset = Matrix4d::Identity();
};

class PackScene {
  public:
    explicit PackScene(const session::ResolvedScenario &, Options = {});

    const rendering::Appearance &appearance() const {
        return appearance_;
    }
    // Pool geometry (floor, walls, decks, coping) and task visuals; static for the run.
    const rendering::Scene &staticScene() const {
        return static_;
    }
    // The first poolInstanceCount() instances of staticScene() are the pool (floor, 4 walls, ...).
    std::size_t poolInstanceCount() const {
        return pool_instances_;
    }
    const std::string &rootFrame() const {
        return frames_.root();
    }
    const spatial::FixedFrames &frames() const {
        return frames_;
    }
    Matrix4d rootFromFrame(const std::string &frame) const {
        return toMatrix(frames_.fromRoot(frame));
    }
    const std::vector<RobotVisual> &robotVisuals() const {
        return robot_;
    }
    const std::vector<PropVisual> &propVisuals() const {
        return props_;
    }

    // Cached mesh of a declared pack asset ("robot" | "pool" | "tasks"); optional texture asset id
    // of the same pack replaces the diffuse texture of every submesh. Throws when strict and the
    // asset is unusable, else returns null and records a warning.
    std::shared_ptr<const rendering::MeshAsset> mesh(const std::string &role, const std::string &asset,
                                                     const std::string &texture = {}) const;
    // Convenience for dynamic content: instance of a pack asset at a world pose (float cast last).
    rendering::Instance instance(const std::string &role, const std::string &asset,
                                 const Matrix4d &world_from_asset) const;

    // Static scene, robot visuals at world_from_root (with overrides), then the dynamic instances.
    rendering::Scene compose(const Matrix4d &world_from_root,
                             const std::vector<rendering::Instance> &dynamic = {},
                             const std::vector<RobotOverride> &overrides = {}) const;
    rendering::Scene compose(const spatial::Pose &world_from_root,
                             const std::vector<rendering::Instance> &dynamic = {},
                             const std::vector<RobotOverride> &overrides = {}) const {
        return compose(toMatrix(world_from_root), dynamic, overrides);
    }

    const std::vector<std::string> &warnings() const {
        return warnings_;
    }
    // JSON-ready record: pool, counts, cutouts, texture overrides, props without visuals.
    session::Json describe() const;

  private:
    const session::ResolvedScenario resolved_; // copy: the caller's document need not outlive us
    Options options_;
    spatial::FixedFrames frames_;
    rendering::Appearance appearance_;
    rendering::Scene static_;
    std::size_t pool_instances_ = 0;
    std::vector<RobotVisual> robot_;
    std::vector<PropVisual> props_;
    mutable std::mutex mutex_;
    mutable std::map<std::string, std::shared_ptr<const rendering::MeshAsset>> cache_;
    mutable std::vector<std::string> warnings_;
    session::Json pool_record_, cutouts_ = session::Json::array(), textures_ = session::Json::array(),
                                unrendered_ = session::Json::array();

    void warn(const std::string &) const;
    void buildPool();
    void buildTasks();
};
} // namespace robotics::pack_scene
