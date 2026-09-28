#include "bindings.hpp"
#include <robotics/spatial/frames.hpp>

namespace robotics::python {
void bindSpatial(py::module_ &m) {
    using namespace spatial;
    auto pose = py::class_<Pose>(m, "Pose").def(py::init<>());
    valueProperty(pose, "translation", &Pose::translation);
    pose.def_property(
        "orientation_wxyz",
        [](const Pose &self) {
            return Eigen::Vector4d(self.rotation.w(), self.rotation.x(), self.rotation.y(),
                                   self.rotation.z());
        },
        [](Pose &self, const Eigen::Vector4d &q) {
            self.rotation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        });
    auto frame = py::class_<FixedFrame>(m, "FixedFrame")
                     .def(py::init<>())
                     .def_readwrite("parent", &FixedFrame::parent)
                     .def_readwrite("child", &FixedFrame::child);
    valueProperty(frame, "pose", &FixedFrame::pose);
    py::class_<FixedFrames>(m, "FixedFrames")
        .def(py::init<std::string, std::vector<FixedFrame>>(), py::arg("root"), py::arg("edges"))
        .def_property_readonly("root", &FixedFrames::root, py::return_value_policy::copy)
        .def_property_readonly("edges", &FixedFrames::edges, py::return_value_policy::copy)
        .def("from_root", &FixedFrames::fromRoot, py::arg("frame"), py::return_value_policy::copy)
        .def("lookup", &FixedFrames::lookup, py::arg("target"), py::arg("from_frame"));
}
} // namespace robotics::python
