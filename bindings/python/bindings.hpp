#pragma once
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
namespace robotics::python {
// Explicit value returns prevent Eigen/member views from escaping into Python.
template <class T, class Value>
void valueProperty(py::class_<T> &type, const char *name, Value T::*member) {
    type.def_property(
        name, [member](const T &self) -> Value { return self.*member; },
        [member](T &self, const Value &value) { self.*member = value; });
}
template <class T, class Value>
void readCopy(py::class_<T> &type, const char *name, Value T::*member) {
    type.def_property_readonly(name, [member](const T &self) -> Value { return self.*member; });
}
void bindPlant(py::module_ &);
void bindSensors(py::module_ &);
void bindRuntime(py::module_ &);
} // namespace robotics::python
