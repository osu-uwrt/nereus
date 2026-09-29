#include "robotics/simulation/payload.hpp"
#include "bindings.hpp"

namespace robotics::python {
using namespace simulation;
void bindPayload(py::module_ &m) {
    py::enum_<PayloadModel>(m, "PayloadModel")
        .value("FIXED_AXIS", PayloadModel::FixedAxis)
        .value("FINNED", PayloadModel::Finned);
    py::class_<PayloadParameters>(m, "PayloadParameters")
        .def(py::init<>())
        .def_readwrite("model", &PayloadParameters::model)
        .def_readwrite("mass", &PayloadParameters::mass)
        .def_readwrite("displaced_volume", &PayloadParameters::displaced_volume)
        .def_readwrite("neutral_buoyancy", &PayloadParameters::neutral_buoyancy)
        .def_readwrite("added_mass", &PayloadParameters::added_mass)
        .def_readwrite("length", &PayloadParameters::length)
        .def_readwrite("radius", &PayloadParameters::radius)
        .def_readwrite("drag_axial", &PayloadParameters::drag_axial)
        .def_readwrite("drag_lateral", &PayloadParameters::drag_lateral)
        .def_readwrite("center_of_mass", &PayloadParameters::center_of_mass)
        .def_readwrite("center_of_buoyancy", &PayloadParameters::center_of_buoyancy)
        .def_readwrite("center_of_drag", &PayloadParameters::center_of_drag)
        .def_readwrite("angular_damping", &PayloadParameters::angular_damping)
        .def_readwrite("spring_energy", &PayloadParameters::spring_energy);
    auto state = py::class_<PayloadState>(m, "PayloadState").def(py::init<>());
    valueProperty(state, "position", &PayloadState::position);
    valueProperty(state, "velocity", &PayloadState::velocity);
    valueProperty(state, "angular_velocity", &PayloadState::angular_velocity);
    state.def_property(
        "orientation_wxyz",
        [](const PayloadState &self) {
            return Eigen::Vector4d(self.orientation.w(), self.orientation.x(), self.orientation.y(),
                                   self.orientation.z());
        },
        [](PayloadState &self, const Eigen::Vector4d &q) {
            self.orientation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        });
    auto environment = py::class_<PayloadEnvironment>(m, "PayloadEnvironment")
                           .def(py::init<>())
                           .def_readwrite("water_density", &PayloadEnvironment::water_density)
                           .def_readwrite("water_level", &PayloadEnvironment::water_level);
    valueProperty(environment, "water_velocity", &PayloadEnvironment::water_velocity);
    py::class_<PayloadDynamics>(m, "PayloadDynamics")
        .def(py::init<PayloadParameters>(), py::arg("parameters"))
        .def_property_readonly("parameters",
                               [](const PayloadDynamics &self) { return self.parameters(); })
        .def(
            "advance",
            [](const PayloadDynamics &self, PayloadState input, PayloadEnvironment conditions,
               double dt) {
                py::gil_scoped_release release;
                return self.advance(input, conditions, dt);
            },
            py::arg("state"), py::arg("environment"), py::arg("dt_s"))
        .def("launch_speed", &PayloadDynamics::launch_speed, py::arg("environment"))
        .def("support_extent", &PayloadDynamics::support_extent, py::arg("axis"));
}
} // namespace robotics::python
