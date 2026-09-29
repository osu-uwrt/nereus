#include <algorithm>
#include <cmath>
#include <limits>
#include <robotics/integrations/simulation_view.hpp>
#include <set>
#include <stdexcept>

namespace robotics::integrations {
void publishSimulationPose(visualization::LivePoseSource &destination,
                           const simulation::Snapshot &snapshot) {
    destination.publish({snapshot.generation,
                         snapshot.elapsed.count(),
                         {snapshot.body.position, snapshot.body.orientation}});
}
SimulationPosePublisher::SimulationPosePublisher(visualization::LivePoseSource &destination,
                                                 std::vector<std::string> plant_inputs,
                                                 std::optional<visualization::RotorRig> rig)
    : destination_(destination), plant_input_count_(plant_inputs.size()), rig_(std::move(rig)) {
    if (!destination_.colorChannels().empty())
        throw std::invalid_argument(
            "simulation color channels require a separate presentation provider");
    if (std::set<std::string>(plant_inputs.begin(), plant_inputs.end()).size() !=
        plant_inputs.size())
        throw std::invalid_argument("duplicate plant force channel IDs");
    if (!rig_) {
        if (!destination_.movingFrames().empty())
            throw std::invalid_argument("moving frames require a presentation provider");
        return;
    }
    visualization::validate(*rig_);
    const auto &frames = destination_.movingFrames();
    if (frames.size() != rig_->mounts.size())
        throw std::invalid_argument("source topology does not match rotor rig");
    for (std::size_t i = 0; i < frames.size(); ++i)
        if (frames[i].parent != rig_->mounts[i].parent_frame ||
            frames[i].child != rig_->mounts[i].child_frame)
            throw std::invalid_argument("source rotor frame order does not match rig");
    for (const auto &id : rig_->inputs) {
        const auto found = std::find(plant_inputs.begin(), plant_inputs.end(), id);
        if (found == plant_inputs.end())
            throw std::invalid_argument("missing plant force channel: " + id);
        inputs_.push_back(static_cast<std::size_t>(found - plant_inputs.begin()));
    }
    animator_.emplace(rig_->animation);
    staged_animator_.emplace(rig_->animation);
    forces_.resize(inputs_.size());
    packet_.moving_poses.resize(rig_->mounts.size());
}
void SimulationPosePublisher::publish(const simulation::Snapshot &snapshot) {
    const auto stamp = snapshot.elapsed.count();
    if (stamp < 0 ||
        (have_packet_ && (snapshot.generation < packet_.generation ||
                          (snapshot.generation == packet_.generation && stamp <= packet_.time_ns))))
        throw std::invalid_argument("presentation observations must increase within a generation");
    const spatial::Pose pose{snapshot.body.position, snapshot.body.orientation};
    spatial::validate(pose);
    if (rig_) {
        if (have_packet_ && snapshot.generation == packet_.generation &&
            (previous_tick_ == std::numeric_limits<std::uint64_t>::max() ||
             snapshot.tick != previous_tick_ + 1))
            throw std::invalid_argument("rotor presentation requires every simulation tick");
        if (static_cast<std::size_t>(snapshot.thruster_forces.size()) != plant_input_count_)
            throw std::invalid_argument("plant force vector does not match configured channels");
        for (std::size_t i = 0; i < inputs_.size(); ++i) {
            const double force = snapshot.thruster_forces[static_cast<Eigen::Index>(inputs_[i])];
            if (!std::isfinite(force) || std::abs(force) > std::numeric_limits<float>::max())
                throw std::invalid_argument(
                    "realized force cannot be represented for presentation");
            forces_[i] = static_cast<float>(force);
        }
        // Reuse equally sized buffers: stage the entire update before committing phase.
        // A rejected force, generation reset or out-of-bounds pivot transform must not
        // partially advance the next accepted observation.
        *staged_animator_ = *animator_;
        if (have_packet_ && snapshot.generation != packet_.generation)
            staged_animator_->reset();
        staged_animator_->receive(forces_, static_cast<double>(stamp) / 1e9);
        for (std::size_t i = 0; i < rig_->mounts.size(); ++i) {
            const auto &mount = rig_->mounts[i];
            packet_.moving_poses[i] = visualization::pivotRotation(mount.pivot, mount.axis,
                                                                   staged_animator_->angles()[i]);
        }
    }
    packet_.generation = snapshot.generation;
    packet_.time_ns = stamp;
    packet_.pose = pose;
    destination_.publish(packet_);
    animator_.swap(staged_animator_);
    previous_tick_ = snapshot.tick;
    have_packet_ = true;
}
} // namespace robotics::integrations
