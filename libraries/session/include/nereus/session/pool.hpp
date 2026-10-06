#pragma once
// What a resolved pool document describes, interpreted once: its floor (flat or profiled), its placed `box` and
// `mesh` fixtures, and its static contact boxes. Geometry only: colours and finishes stay in the document for
// the renderer's adapter (pack_scene). Every position is pool-local: origin at a floor-plan corner, +x along
// the length, +y across, +z up, the water surface at `parameters.water_level_m`.
#include <nereus/session/scenario.hpp>
#include <nereus/simulation/floor_profile.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace nereus::session {

// A `box` fixture, placed: centre (a box given only x, y rests on the floor at its centre, else its z is
// relative to the water surface), size, orientation (pool from box; rpy_deg about the box's x, y, z axes
// applied yaw, pitch, roll).
struct PoolFixtureBox {
    std::string id;
    Eigen::Vector3d center = Eigen::Vector3d::Zero(), size = Eigen::Vector3d::Ones();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool on_floor = false, contact = false;
};

// A `mesh` fixture, placed like a box: `center` is the mesh origin (on the floor at x, y when only those are
// given), `asset` the pool asset id.
struct PoolFixtureMesh {
    std::string id, asset;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool on_floor = false;
};

// A static contact box. `floor` marks the boxes that are the pool's floor (where a dropped prop lands "on the
// floor"): every box generated from a floor profile, and any other box whose top face (centre z + half its z
// size) lies within 1 mm of the flat floor at `depth_m` below the surface, which is how a flat pool's floor box
// is recognised.
struct PoolContactBox {
    std::string id;
    Eigen::Vector3d size = Eigen::Vector3d::Ones(), center = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool floor = false;
};

// The whole interpreted pool document.
struct PoolModel {
    double surface_z = 0;  // parameters.water_level_m
    bool profiled = false; // the document has a floor_profile; otherwise `floor` is flat at depth_m
    simulation::PoolFloor floor;
    // The pool's `collision_boxes`, then its `box` fixtures with `contact: true`, then (profiled floors) one box
    // per floor segment with ids floor_0, floor_1, ...
    std::vector<PoolContactBox> contacts;
    std::vector<PoolFixtureBox> boxes;   // `box` fixtures, in document order
    std::vector<PoolFixtureMesh> meshes; // `mesh` fixtures, in document order
};

// Build once per consumer and reuse: the floor is sampled from its curves here.
PoolModel poolModel(const Json &pool);
// Just the floor: `parameters.floor_profile` (one profile or a list, the floor being the shallowest of them),
// or flat at `depth_m`.
simulation::PoolFloor poolFloor(const Json &pool);
} // namespace nereus::session
