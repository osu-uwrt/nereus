#include "bindings.hpp"

PYBIND11_MODULE(_native, module) {
    module.doc() =
        "Native simulation bindings. Time is integer nanoseconds; observations are copies.";
    robotics::python::bindSpatial(module);
    robotics::python::bindPlant(module);
    robotics::python::bindPayload(module);
    robotics::python::bindSensors(module);
    robotics::python::bindRuntime(module);
}
