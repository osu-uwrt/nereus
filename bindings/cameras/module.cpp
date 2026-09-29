#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <robotics/cameras/camera.hpp>
#include <robotics/rendering/offscreen.hpp>
#include <memory>
#include <mutex>
#include <set>
#include <tuple>

namespace py = pybind11;
namespace r = robotics::rendering;
namespace c = robotics::cameras;
namespace s = robotics::spatial;
namespace {
struct Processor {
    explicit Processor(std::uint32_t seed) : camera(seed) {}
    c::Processor camera;
    std::mutex mutex;
};
template<class T, class Owner>
py::array pixels(std::vector<T> &values, Owner &owner, std::vector<py::ssize_t> shape) {
    if (values.empty()) {
        py::array result = py::array_t<T>(0);
        result.attr("setflags")(false);
        return result;
    }
    std::vector<py::ssize_t> strides(shape.size(), sizeof(T));
    for (std::size_t i = shape.size() - 1; i > 0; --i)
        strides[i - 1] = strides[i] * shape[i];
    py::array result(py::dtype::of<T>(), shape, strides, values.data(),
                     py::cast(&owner, py::return_value_policy::reference));
    result.attr("setflags")(false);
    return result;
}
}
PYBIND11_MODULE(_camera, m) {
    m.doc() = "Optional standalone camera capture; no ROS or simulation dependency.";
    py::class_<c::Intrinsics>(m, "Intrinsics")
        .def(py::init<>())
        .def_readwrite("width", &c::Intrinsics::width)
        .def_readwrite("height", &c::Intrinsics::height)
        .def_readwrite("fx", &c::Intrinsics::fx)
        .def_readwrite("fy", &c::Intrinsics::fy)
        .def_readwrite("cx", &c::Intrinsics::cx)
        .def_readwrite("cy", &c::Intrinsics::cy)
        .def_readwrite("near_plane", &c::Intrinsics::near_plane)
        .def_readwrite("far_plane", &c::Intrinsics::far_plane)
        .def("projection", &c::Intrinsics::projection);
    // rp_spatial types with the robotics_platform._native API and validation. Module-local so
    // both extensions coexist in one process; each accepts the other's values (same C++ type).
    auto pose = py::class_<s::Pose>(m, "Pose", py::module_local()).def(py::init<>());
    pose.def_property(
            "translation", [](const s::Pose &self) -> Eigen::Vector3d { return self.translation; },
            [](s::Pose &self, const Eigen::Vector3d &value) { self.translation = value; })
        .def_property(
            "orientation_wxyz",
            [](const s::Pose &self) {
                return Eigen::Vector4d(self.rotation.w(), self.rotation.x(), self.rotation.y(),
                                       self.rotation.z());
            },
            [](s::Pose &self, const Eigen::Vector4d &q) {
                self.rotation = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
            })
        .def("compose", [](const s::Pose &self, const s::Pose &child) {
            s::validate(self);
            s::validate(child);
            auto result = s::compose(self, child);
            s::validate(result);
            return result;
        }, py::arg("child"))
        .def("inverse", [](const s::Pose &self) {
            s::validate(self);
            return s::inverse(self);
        })
        .def("apply", [](const s::Pose &self, const Eigen::Vector3d &point) {
            s::validate(self);
            const Eigen::Vector3d result = s::apply(self, point);
            if (!point.allFinite() || !result.allFinite())
                throw std::invalid_argument("transformed point must be finite");
            return result;
        }, py::arg("point"))
        .def("matrix", [](const s::Pose &self) {
            s::validate(self);
            Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
            result.topLeftCorner<3, 3>() = self.rotation.toRotationMatrix();
            result.topRightCorner<3, 1>() = self.translation;
            return result;
        }, "Validated 4x4 child-to-parent transform.");
    py::class_<s::FixedFrame>(m, "FixedFrame", py::module_local())
        .def(py::init<>())
        .def_readwrite("parent", &s::FixedFrame::parent)
        .def_readwrite("child", &s::FixedFrame::child)
        .def_property(
            "pose", [](const s::FixedFrame &self) -> s::Pose { return self.pose; },
            [](s::FixedFrame &self, const s::Pose &value) { self.pose = value; });
    py::class_<s::FixedFrames>(m, "FixedFrames", py::module_local())
        .def(py::init<std::string, std::vector<s::FixedFrame>>(), py::arg("root"), py::arg("edges"))
        .def_property_readonly("root", &s::FixedFrames::root, py::return_value_policy::copy)
        .def_property_readonly("edges", &s::FixedFrames::edges, py::return_value_policy::copy)
        .def("from_root", &s::FixedFrames::fromRoot, py::arg("frame"),
             py::return_value_policy::copy)
        .def("lookup", &s::FixedFrames::lookup, py::arg("target"), py::arg("from_frame"));
    m.def("optical_view", [](const Eigen::Vector3d &position, const Eigen::Vector4d &wxyz) {
        s::Pose pose;
        pose.translation = position;
        pose.rotation = Eigen::Quaterniond(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
        return c::opticalView(pose);
    });
    m.def("optical_view", &c::opticalView, py::arg("world_from_optical"));
    py::class_<c::DepthNoise>(m, "DepthNoise")
        .def(py::init<>())
        .def_readwrite("enabled", &c::DepthNoise::enabled)
        .def_readwrite("base_sigma", &c::DepthNoise::base_sigma)
        .def_readwrite("range_sigma", &c::DepthNoise::range_sigma)
        .def_readwrite("exponent", &c::DepthNoise::exponent)
        .def_readwrite("min_range", &c::DepthNoise::min_range)
        .def_readwrite("max_range", &c::DepthNoise::max_range)
        .def_readwrite("bias", &c::DepthNoise::bias)
        .def_readwrite("dropout", &c::DepthNoise::dropout)
        .def_readwrite("range_dropout", &c::DepthNoise::range_dropout)
        .def_readwrite("edge_dropout", &c::DepthNoise::edge_dropout)
        .def_readwrite("outliers", &c::DepthNoise::outliers)
        .def_readwrite("correlation", &c::DepthNoise::correlation)
        .def_readwrite("patch_size", &c::DepthNoise::patch_size);
    py::class_<c::Frame>(m, "Frame", "Owned top-down RGB8 and optical-axis depth in metres.")
        .def_readonly("width", &c::Frame::width)
        .def_readonly("height", &c::Frame::height)
        .def_property_readonly("rgb", [](c::Frame &f) {
            return pixels(f.rgb, f, {f.height, f.width, 3});
        })
        .def_property_readonly("depth", [](c::Frame &f) {
            return pixels(f.depth, f, {f.height, f.width});
        })
        .def_property_readonly("jpeg", [](const c::Frame &f) {
            return py::bytes(reinterpret_cast<const char *>(f.jpeg.data()), f.jpeg.size());
        });
    py::class_<r::ImageCapture>(m, "ImageCapture", "Owned bottom-up RGB8 and nonlinear depth.")
        .def_readonly("width", &r::ImageCapture::width)
        .def_readonly("height", &r::ImageCapture::height)
        .def_property_readonly("rgb", [](r::ImageCapture &f) {
            return pixels(f.rgb, f, {f.height, f.width, 3});
        })
        .def_property_readonly("depth", [](r::ImageCapture &f) {
            return pixels(f.depth, f, {f.height, f.width});
        });
    py::class_<Processor>(m, "Processor")
        .def(py::init<std::uint32_t>(), py::arg("seed") = 7)
        .def("reset", [](Processor &p, std::uint32_t seed) {
            py::gil_scoped_release release;
            std::lock_guard<std::mutex> lock(p.mutex);
            p.camera.reset(seed);
        })
        .def("process", [](Processor &p, c::Intrinsics intrinsics, c::DepthNoise noise,
                            const r::ImageCapture &input, bool jpeg, int quality) {
            py::gil_scoped_release release;
            if (input.width != intrinsics.width || input.height != intrinsics.height)
                throw std::invalid_argument("capture dimensions differ from calibration");
            std::lock_guard<std::mutex> lock(p.mutex);
            return p.camera.process(intrinsics, noise, input.rgb, input.depth, jpeg, quality);
        }, py::arg("intrinsics"), py::arg("noise"), py::arg("input"),
           py::arg("jpeg") = false, py::arg("quality") = 93);
    py::class_<r::MeshAsset, std::shared_ptr<r::MeshAsset>>(m, "Mesh")
        .def_property_readonly("dependencies", [](const r::MeshAsset &mesh) {
            return mesh.dependencies;
        }, "Sorted canonical files opened by the importer, including mesh and sidecars.")
        .def_property_readonly("textures", [](const r::MeshAsset &mesh) {
            std::set<std::filesystem::path> paths;
            for (const auto &part : mesh.submeshes)
                if (part.material.diffuse_texture)
                    paths.insert(*part.material.diffuse_texture);
            return std::vector<std::filesystem::path>(paths.begin(), paths.end());
        }, "Sorted external texture files the renderer opens at first draw.")
        .def("with_texture", [](const r::MeshAsset &mesh, const std::filesystem::path &texture) {
            auto copy = std::make_shared<r::MeshAsset>(mesh);
            bool textured = false;
            for (const auto &part : copy->submeshes)
                textured = textured || part.material.diffuse_texture.has_value();
            for (auto &part : copy->submeshes)
                if (!textured || part.material.diffuse_texture)
                    part.material.diffuse_texture = texture;
            return copy;
        }, py::arg("texture"),
           "Copy whose textured submeshes (every submesh when none is textured) use this PNG.");
    m.def("load_mesh", [](const std::filesystem::path &path) {
        return std::make_shared<r::MeshAsset>(r::loadMesh(path));
    }, py::call_guard<py::gil_scoped_release>());
    m.def("perforate_mesh", [](const std::shared_ptr<r::MeshAsset> &mesh,
                                const Eigen::Matrix4f &asset_to_panel,
                                const std::vector<float> &faces_x, float half_size,
                                const std::vector<std::tuple<float, float, float>> &cutouts,
                                float tolerance) {
        if (!mesh)
            throw std::invalid_argument("mesh must not be None");
        r::PanelCutouts panel;
        panel.asset_to_panel = asset_to_panel;
        panel.faces_x = faces_x;
        panel.half_size = half_size;
        panel.tolerance = tolerance;
        for (const auto &[u, v, radius] : cutouts)
            panel.cutouts.push_back({{u, v}, radius});
        r::PerforatedMesh output;
        {
            py::gil_scoped_release release;
            output = r::perforatePanel(*mesh, panel);
        }
        const std::vector<py::ssize_t> counts(output.face_triangles.begin(),
                                              output.face_triangles.end());
        return py::make_tuple(std::make_shared<r::MeshAsset>(std::move(output.mesh)), counts);
    }, py::arg("mesh"), py::arg("asset_to_panel"), py::arg("faces_x"), py::arg("half_size"),
       py::arg("cutouts"), py::arg("tolerance") = 5e-4f);
    m.def("box_mesh", &r::makeBoxMesh);
    py::enum_<r::SurfaceMaterial>(m, "SurfaceMaterial")
        .value("ASSET", r::SurfaceMaterial::Asset)
        .value("TILES", r::SurfaceMaterial::Tiles)
        .value("DECK", r::SurfaceMaterial::Deck)
        .value("LINER", r::SurfaceMaterial::Liner);
    py::class_<r::Instance>(m, "Instance")
        .def(py::init<>())
        .def_readwrite("mesh", &r::Instance::mesh)
        .def_readwrite("transform", &r::Instance::transform)
        .def_readwrite("tint", &r::Instance::tint)
        .def_readwrite("material", &r::Instance::material)
        .def_readwrite("visible", &r::Instance::visible)
        .def_readwrite("casts_shadow", &r::Instance::casts_shadow);
    py::class_<r::Scene>(m, "Scene")
        .def(py::init<>())
        .def_property("instances", [](const r::Scene &s) { return s.instances; },
                      [](r::Scene &s, std::vector<r::Instance> instances) {
                          s.instances = std::move(instances);
                      }, "Owned instance copies: assign the whole list to edit.")
        .def("add", [](r::Scene &s, r::Instance i) { s.instances.push_back(std::move(i)); })
        .def_readwrite("lighting_center", &r::Scene::lighting_center);
    py::class_<r::PoolGeometry>(m, "PoolGeometry")
        .def(py::init<>())
        .def_readwrite("dimensions", &r::PoolGeometry::dimensions)
        .def_readwrite("water_level", &r::PoolGeometry::water_level)
        .def_readwrite("deck_height", &r::PoolGeometry::deck_height)
        .def_readwrite("local_to_world", &r::PoolGeometry::local_to_world);
    m.def("pool_scene", &r::makePoolScene);
    py::class_<r::View>(m, "View")
        .def(py::init<>())
        .def_readwrite("view", &r::View::view)
        .def_readwrite("projection", &r::View::projection)
        .def_readwrite("eye", &r::View::eye);
    py::class_<r::WaterOptics>(m, "WaterOptics")
        .def(py::init<>())
        .def_readwrite("tint", &r::WaterOptics::tint)
        .def_readwrite("absorption", &r::WaterOptics::absorption)
        .def_readwrite("scattering", &r::WaterOptics::scattering)
        .def_readwrite("distance_scale", &r::WaterOptics::distance_scale)
        .def_readwrite("distance_power", &r::WaterOptics::distance_power)
        .def_readwrite("clear_distance", &r::WaterOptics::clear_distance);
    py::class_<r::Appearance>(m, "Appearance")
        .def(py::init<>())
        .def_readwrite("water", &r::Appearance::water)
        .def_readwrite("caustics", &r::Appearance::caustics)
        .def_readwrite("exposure", &r::Appearance::exposure)
        .def_readwrite("surface", &r::Appearance::surface)
        .def_readwrite("shadows", &r::Appearance::shadows)
        .def_readwrite("reflections", &r::Appearance::reflections)
        .def_readwrite("outdoor", &r::Appearance::outdoor)
        .def_readwrite("sun_azimuth", &r::Appearance::sun_azimuth)
        .def_readwrite("sun_elevation", &r::Appearance::sun_elevation)
        .def_readwrite("direct_light", &r::Appearance::direct_light)
        .def_readwrite("ambient_light", &r::Appearance::ambient_light)
        .def_readwrite("glare", &r::Appearance::glare);
    py::class_<r::OffscreenRenderer>(m, "OffscreenRenderer",
        "Serialized display-free capture. Destroy after workers join; teardown may wait for capture.")
        .def(py::init<const std::filesystem::path &>(), py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("device", &r::OffscreenRenderer::device)
        .def("capture", [](r::OffscreenRenderer &host, r::Scene scene, r::View view,
                            r::Appearance appearance, float time, int width, int height,
                            bool color, bool depth) {
            py::gil_scoped_release release;
            return host.capture(scene, view, appearance, time, width, height, color, depth);
        }, py::arg("scene"), py::arg("view"), py::arg("appearance"), py::arg("time"),
           py::arg("width"), py::arg("height"), py::arg("color") = true,
           py::arg("depth") = true);
}
