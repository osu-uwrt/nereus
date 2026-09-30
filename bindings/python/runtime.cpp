#include "robotics/sensors/runtime.hpp"
#include "bindings.hpp"
#include "robotics/sensors/models.hpp"

namespace robotics::python {
using namespace sensors;
namespace {
template <class Model> void addModel(py::class_<Runtime> &type) {
    type.def(
        "add",
        [](Runtime &self, Device device, Model model) {
            return self.add(std::move(device), std::move(model));
        },
        py::arg("device"), py::arg("model"));
}
} // namespace
void bindRuntime(py::module_ &m) {
    auto runtime =
        py::class_<Runtime>(m, "Runtime")
            .def(py::init<const simulation::PlantParameters &, const simulation::BodyState &,
                          std::uint64_t>(),
                 py::arg("parameters"), py::arg("initial"), py::arg("seed") = 0)
            .def("command", &Runtime::command, py::arg("forces"))
            .def("stop_thrusters", &Runtime::stopThrusters)
            .def("advance", &Runtime::advance, py::arg("ticks") = 1)
            .def("observe", &Runtime::observe)
            .def("place", &Runtime::place, py::arg("state"), py::arg("clear_actuators") = true)
            .def("reset", &Runtime::reset, py::arg("initial"), py::arg("seed"))
            .def_property_readonly("faulted", &Runtime::faulted)
            .def("imu_stream", &Runtime::stream<ImuReading>, py::arg("id"))
            .def("attitude_stream", &Runtime::stream<AttitudeReading>, py::arg("id"))
            .def("ahrs_stream", &Runtime::stream<AhrsReading>, py::arg("id"))
            .def("fog_stream", &Runtime::stream<FogReading>, py::arg("id"))
            .def("velocity_stream", &Runtime::stream<VelocityReading>, py::arg("id"))
            .def("dvl_stream", &Runtime::stream<DvlReading>, py::arg("id"))
            .def("altitude_stream", &Runtime::stream<AltitudeReading>, py::arg("id"))
            .def("pressure_stream", &Runtime::stream<PressureReading>, py::arg("id"));
    addModel<Imu>(runtime);
    addModel<Attitude>(runtime);
    addModel<Ahrs>(runtime);
    addModel<Fog>(runtime);
    addModel<Dvl>(runtime);
    addModel<ReferenceVelocity>(runtime);
    addModel<Pressure>(runtime);
    addModel<ReferenceAltitude>(runtime);
}
} // namespace robotics::python
