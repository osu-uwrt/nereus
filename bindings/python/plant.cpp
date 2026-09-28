#include "robotics/simulation/plant.hpp"
#include "bindings.hpp"

namespace robotics::python {
using namespace simulation;
void bindPlant(py::module_ &m) {
    py::enum_<ContactModel>(m, "ContactModel")
        .value("DISABLED", ContactModel::Disabled)
        .value("SPHERE_POOL", ContactModel::SpherePool)
        .value("BOX_SCENE", ContactModel::BoxScene);
    auto box =
        py::class_<BoxProxy>(m, "BoxProxy").def(py::init<>()).def_readwrite("id", &BoxProxy::id);
    valueProperty(box, "size", &BoxProxy::size);
    valueProperty(box, "center", &BoxProxy::center);
    box.def_property(
        "orientation_wxyz",
        [](const BoxProxy &self) {
            return Eigen::Vector4d(self.orientation.w(), self.orientation.x(), self.orientation.y(),
                                   self.orientation.z());
        },
        [](BoxProxy &self, const Eigen::Vector4d &q) {
            self.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        });
    auto contacts = py::class_<ContactParameters>(m, "ContactParameters")
                        .def(py::init<>())
                        .def_readwrite("model", &ContactParameters::model)
                        .def_readwrite("restitution", &ContactParameters::restitution)
                        .def_readwrite("friction", &ContactParameters::friction);
    valueProperty(contacts, "body_boxes", &ContactParameters::body_boxes);
    valueProperty(contacts, "world_boxes", &ContactParameters::world_boxes);
    auto body = py::class_<BodyState>(m, "BodyState").def(py::init<>());
    valueProperty(body, "position", &BodyState::position);
    valueProperty(body, "linear_velocity", &BodyState::linear_velocity);
    valueProperty(body, "angular_velocity", &BodyState::angular_velocity);
    body.def_property(
        "orientation_wxyz",
        [](const BodyState &self) {
            return Eigen::Vector4d(self.orientation.w(), self.orientation.x(), self.orientation.y(),
                                   self.orientation.z());
        },
        [](BodyState &self, const Eigen::Vector4d &q) {
            self.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        });
    auto bp = py::class_<BodyParameters>(m, "BodyParameters")
                  .def(py::init<>())
                  .def_readwrite("mass", &BodyParameters::mass)
                  .def_readwrite("displaced_volume", &BodyParameters::displaced_volume)
                  .def_readwrite("collision_radius", &BodyParameters::collision_radius);
    valueProperty(bp, "inertia", &BodyParameters::inertia);
    valueProperty(bp, "added_mass", &BodyParameters::added_mass);
    valueProperty(bp, "linear_damping", &BodyParameters::linear_damping);
    valueProperty(bp, "quadratic_damping", &BodyParameters::quadratic_damping);
    valueProperty(bp, "damping_center", &BodyParameters::damping_center);
    valueProperty(bp, "buoyancy_center", &BodyParameters::buoyancy_center);
    valueProperty(bp, "buoyancy_radii", &BodyParameters::buoyancy_radii);
    auto pool =
        py::class_<Pool>(m, "Pool")
            .def(py::init<>())
            .def_readwrite("length", &Pool::length)
            .def_readwrite("width", &Pool::width)
            .def_readwrite("depth", &Pool::depth)
            .def_readwrite("water_level", &Pool::water_level)
            .def_readwrite("water_density", &Pool::water_density)
            .def_readwrite("current_oscillation_frequency", &Pool::current_oscillation_frequency);
    valueProperty(pool, "current_velocity", &Pool::current_velocity);
    valueProperty(pool, "current_oscillation_amplitude", &Pool::current_oscillation_amplitude);
    auto thruster = py::class_<Thruster>(m, "Thruster")
                        .def(py::init<>())
                        .def_readwrite("id", &Thruster::id)
                        .def_readwrite("delay", &Thruster::delay)
                        .def_readwrite("rise_time", &Thruster::rise_time)
                        .def_readwrite("fall_time", &Thruster::fall_time)
                        .def_readwrite("slew_rate", &Thruster::slew_rate)
                        .def_readwrite("forward_limit", &Thruster::forward_limit)
                        .def_readwrite("reverse_limit", &Thruster::reverse_limit)
                        .def_readwrite("propeller_radius", &Thruster::propeller_radius)
                        .def_readwrite("deadband", &Thruster::deadband)
                        .def_readwrite("forward_scale", &Thruster::forward_scale)
                        .def_readwrite("reverse_scale", &Thruster::reverse_scale)
                        .def_readwrite("efficiency", &Thruster::efficiency);
    valueProperty(thruster, "position", &Thruster::position);
    valueProperty(thruster, "direction", &Thruster::direction);
    auto parameters =
        py::class_<PlantParameters>(m, "PlantParameters")
            .def(py::init<>())
            .def_readwrite("body", &PlantParameters::body)
            .def_readwrite("pool", &PlantParameters::pool)
            .def_readwrite("contacts", &PlantParameters::contacts)
            .def_readwrite("command_timeout", &PlantParameters::command_timeout)
            .def_property(
                "timestep_ns", [](const PlantParameters &self) { return self.timestep.count(); },
                [](PlantParameters &self, std::int64_t ns) {
                    self.timestep = std::chrono::nanoseconds(ns);
                });
    valueProperty(parameters, "thrusters", &PlantParameters::thrusters);
    auto snapshot = py::class_<Snapshot>(m, "Snapshot")
                        .def_readonly("generation", &Snapshot::generation)
                        .def_readonly("tick", &Snapshot::tick)
                        .def_property_readonly("elapsed_ns", [](const Snapshot &self) {
                            return self.elapsed.count();
                        });
    readCopy(snapshot, "body", &Snapshot::body);
    readCopy(snapshot, "thruster_forces", &Snapshot::thruster_forces);
}
} // namespace robotics::python
