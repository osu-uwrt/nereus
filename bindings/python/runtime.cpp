#include "bindings.hpp"
#include "robotics/config/scenario.hpp"
#include "robotics/sensors/models.hpp"
#include <pybind11/stl/filesystem.h>

namespace robotics::python {
using namespace sensors;
namespace {
struct SensorInfo {
    Device device;
    std::string model;
};
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
    auto command = py::class_<config::ScheduledCommand>(m, "ScheduledCommand")
                       .def_readonly("tick", &config::ScheduledCommand::tick);
    readCopy(command, "forces", &config::ScheduledCommand::forces);
    auto info = py::class_<SensorInfo>(m, "SensorInfo").def_readonly("model", &SensorInfo::model);
    readCopy(info, "device", &SensorInfo::device);
    auto scenario =
        py::class_<config::Scenario>(m, "Scenario")
            .def_readonly("seed", &config::Scenario::seed)
            .def_readonly("ticks", &config::Scenario::ticks)
            .def_readonly("surface_pressure", &config::Scenario::surface_pressure)
            .def_property_readonly("sensors",
                                   [](const config::Scenario &self) {
                                       std::vector<SensorInfo> infos;
                                       for (const auto &sensor : self.sensors) {
                                           infos.push_back({sensor.device, sensor.model});
                                       }
                                       return infos;
                                   })
            .def(
                "create_runtime",
                [](const config::Scenario &self, std::optional<simulation::BodyState> initial,
                   std::optional<std::uint64_t> seed) {
                    auto configured = self;
                    if (initial) {
                        configured.initial = *initial;
                    }
                    if (seed) {
                        configured.seed = *seed;
                    }
                    return config::makeRuntime(configured);
                },
                py::arg("initial") = py::none(), py::arg("seed") = py::none());
    readCopy(scenario, "body_frames", &config::Scenario::body_frames);
    readCopy(scenario, "plant", &config::Scenario::plant);
    readCopy(scenario, "initial", &config::Scenario::initial);
    readCopy(scenario, "commands", &config::Scenario::commands);
    readCopy(scenario, "sources", &config::Scenario::sources);
    m.def("load_scenario", &config::loadScenario, py::arg("path"));
}
} // namespace robotics::python
