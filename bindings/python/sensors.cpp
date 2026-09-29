#include "bindings.hpp"
#include "robotics/sensors/models.hpp"
#include "robotics/sensors/runtime.hpp"

namespace robotics::python {
using namespace sensors;
namespace {
template <class Reading>
void bindStream(py::module_ &m, const char *sample_name, const char *stream_name) {
    using S = Sample<Reading>;
    auto sample =
        py::class_<S>(m, sample_name)
            .def_property_readonly("value", [](const S &self) { return self.measurement.value; })
            .def_property_readonly("unavailable_reason", [](const S &self) {
                return self.measurement.unavailable_reason;
            });
    readCopy(sample, "header", &S::header);
    using Stream = SensorStream<Reading>;
    py::class_<Stream, std::shared_ptr<Stream>>(m, stream_name)
        .def("latest", &Stream::latest)
        .def("drain", &Stream::drain)
        .def_property_readonly("active", &Stream::active)
        .def_property_readonly("stats", &Stream::stats);
}
} // namespace
void bindSensors(py::module_ &m) {
    py::enum_<OverflowPolicy>(m, "OverflowPolicy")
        .value("FAIL", OverflowPolicy::Fail)
        .value("DROP_OLDEST", OverflowPolicy::DropOldest);
    py::class_<Device>(m, "Device")
        .def(py::init([](std::string id, std::string frame, std::int64_t period,
                         std::int64_t latency, std::size_t capacity, OverflowPolicy overflow) {
                 return Device{std::move(id),        std::move(frame), Nanoseconds(period),
                               Nanoseconds(latency), capacity,         overflow};
             }),
             py::arg("id"), py::arg("frame"), py::arg("period_ns") = 10'000'000,
             py::arg("latency_ns") = 0, py::arg("capacity") = 64,
             py::arg("overflow") = OverflowPolicy::Fail)
        .def_readwrite("id", &Device::id)
        .def_readwrite("frame", &Device::frame)
        .def_readwrite("capacity", &Device::capacity)
        .def_readwrite("overflow", &Device::overflow)
        .def_property(
            "period_ns", [](const Device &self) { return self.period.count(); },
            [](Device &self, std::int64_t ns) { self.period = Nanoseconds(ns); })
        .def_property(
            "latency_ns", [](const Device &self) { return self.latency.count(); },
            [](Device &self, std::int64_t ns) { self.latency = Nanoseconds(ns); });
    py::class_<SampleHeader>(m, "SampleHeader")
        .def_readonly("device_id", &SampleHeader::device_id)
        .def_readonly("frame", &SampleHeader::frame)
        .def_readonly("generation", &SampleHeader::generation)
        .def_readonly("sequence", &SampleHeader::sequence)
        .def_readonly("tick", &SampleHeader::tick)
        .def_property_readonly("scheduled_ns",
                               [](const SampleHeader &self) { return self.scheduled.count(); })
        .def_property_readonly("acquired_ns",
                               [](const SampleHeader &self) { return self.acquired.count(); })
        .def_property_readonly("delivered_ns",
                               [](const SampleHeader &self) { return self.delivered.count(); });
    py::class_<StreamStats>(m, "StreamStats")
        .def_readonly("acquired", &StreamStats::acquired)
        .def_readonly("delivered", &StreamStats::delivered)
        .def_readonly("unavailable", &StreamStats::unavailable)
        .def_readonly("dropped_pending", &StreamStats::dropped_pending)
        .def_readonly("dropped_delivered", &StreamStats::dropped_delivered);
    auto mount = py::class_<Mount>(m, "Mount").def(py::init<>());
    valueProperty(mount, "position_body", &Mount::position_body);
    mount.def_property(
        "orientation_wxyz",
        [](const Mount &self) {
            const auto &q = self.sensor_to_body;
            return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
        },
        [](Mount &self, const Eigen::Vector4d &q) {
            self.sensor_to_body = Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
        });
    auto noise = py::class_<NoiseParameters>(m, "NoiseParameters").def(py::init<>());
    valueProperty(noise, "bias", &NoiseParameters::bias);
    valueProperty(noise, "white_stddev", &NoiseParameters::white_stddev);
    valueProperty(noise, "walk_stddev", &NoiseParameters::walk_stddev);
    auto reporting = py::class_<ImuReporting>(m, "ImuReporting").def(py::init<>());
    valueProperty(reporting, "gravity_magnitude", &ImuReporting::gravity_magnitude);
    valueProperty(reporting, "force_variance", &ImuReporting::force_variance);
    valueProperty(reporting, "angular_variance", &ImuReporting::angular_variance);
    auto attitude_parameters =
        py::class_<AttitudeParameters>(m, "AttitudeParameters")
            .def(py::init<>())
            .def_readwrite("angle_stddev", &AttitudeParameters::angle_stddev)
            .def_readwrite("heading_drift_rate", &AttitudeParameters::heading_drift_rate);
    valueProperty(attitude_parameters, "heading_axis_world",
                  &AttitudeParameters::heading_axis_world);
    valueProperty(attitude_parameters, "reported_variance", &AttitudeParameters::reported_variance);
    auto ahrs_parameters = py::class_<AhrsParameters>(m, "AhrsParameters").def(py::init<>());
    valueProperty(ahrs_parameters, "acceleration_noise", &AhrsParameters::acceleration_noise);
    valueProperty(ahrs_parameters, "gyro_noise", &AhrsParameters::gyro_noise);
    valueProperty(ahrs_parameters, "inertial_reporting", &AhrsParameters::inertial_reporting);
    valueProperty(ahrs_parameters, "attitude", &AhrsParameters::attitude);
    py::class_<ScalarNoiseParameters>(m, "ScalarNoiseParameters")
        .def(py::init<>())
        .def_readwrite("bias", &ScalarNoiseParameters::bias)
        .def_readwrite("white_stddev", &ScalarNoiseParameters::white_stddev)
        .def_readwrite("walk_stddev", &ScalarNoiseParameters::walk_stddev);
    auto dvl = py::class_<DvlParameters>(m, "DvlParameters")
                   .def(py::init<>())
                   .def_readwrite("mount", &DvlParameters::mount)
                   .def_readwrite("minimum_range", &DvlParameters::minimum_range)
                   .def_readwrite("maximum_range", &DvlParameters::maximum_range)
                   .def_readwrite("velocity_noise", &DvlParameters::velocity_noise);
    valueProperty(dvl, "bottom_axis", &DvlParameters::bottom_axis);
    py::class_<PressureParameters>(m, "PressureParameters")
        .def(py::init<>())
        .def_readwrite("mount", &PressureParameters::mount)
        .def_readwrite("noise", &PressureParameters::noise)
        .def_readwrite("reference_pressure", &PressureParameters::reference_pressure)
        .def_readwrite("reference_density", &PressureParameters::reference_density)
        .def_readwrite("reference_gravity", &PressureParameters::reference_gravity)
        .def_readwrite("minimum_pressure", &PressureParameters::minimum_pressure)
        .def_readwrite("maximum_pressure", &PressureParameters::maximum_pressure);
    py::class_<HydrostaticPressure>(m, "HydrostaticPressure")
        .def(py::init<double, double, double, double>(), py::arg("water_level"),
             py::arg("density") = 1000, py::arg("surface_pressure") = 101325,
             py::arg("gravity") = 9.80665);
    py::class_<Imu>(m, "Imu").def(
        py::init<Mount, NoiseParameters, NoiseParameters, ImuReporting>(),
        py::arg("mount") = Mount{}, py::arg("acceleration_noise") = NoiseParameters{},
        py::arg("gyro_noise") = NoiseParameters{}, py::arg("reporting") = ImuReporting{});
    py::class_<Attitude>(m, "Attitude")
        .def(py::init<Mount, AttitudeParameters>(), py::arg("mount") = Mount{},
             py::arg("parameters") = AttitudeParameters{});
    py::class_<Ahrs>(m, "Ahrs").def(py::init<Mount, AhrsParameters>(), py::arg("mount") = Mount{},
                                    py::arg("parameters") = AhrsParameters{});
    py::class_<Fog>(m, "Fog").def(
        py::init<Mount, std::vector<Eigen::Vector3d>, NoiseParameters,
                 std::optional<Eigen::Vector3d>>(),
        py::arg("mount") = Mount{},
        py::arg("axes") = std::vector<Eigen::Vector3d>{Eigen::Vector3d::UnitZ()},
        py::arg("gyro_noise") = NoiseParameters{}, py::arg("reported_variance") = py::none());
    py::class_<Dvl>(m, "Dvl").def(
        py::init([](const DvlParameters &params, const simulation::Pool &pool) {
            return Dvl(params, PoolBottom(pool));
        }),
        py::arg("parameters"), py::arg("pool"));
    py::class_<Pressure>(m, "Pressure")
        .def(py::init([](const PressureParameters &params, const HydrostaticPressure &environment) {
                 return Pressure(params, environment);
             }),
             py::arg("parameters"), py::arg("environment"));

    auto imu_reading = py::class_<ImuReading>(m, "ImuReading");
    readCopy(imu_reading, "specific_force", &ImuReading::specific_force);
    readCopy(imu_reading, "angular_velocity", &ImuReading::angular_velocity);
    readCopy(imu_reading, "force_covariance", &ImuReading::force_covariance);
    readCopy(imu_reading, "angular_covariance", &ImuReading::angular_covariance);
    auto attitude_reading = py::class_<AttitudeReading>(m, "AttitudeReading");
    attitude_reading.def_property_readonly("orientation_wxyz", [](const AttitudeReading &self) {
        const auto &q = self.sensor_to_world;
        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    });
    readCopy(attitude_reading, "covariance", &AttitudeReading::covariance);
    auto ahrs_reading = py::class_<AhrsReading>(m, "AhrsReading");
    readCopy(ahrs_reading, "inertial", &AhrsReading::inertial);
    readCopy(ahrs_reading, "attitude", &AhrsReading::attitude);
    auto fog_reading = py::class_<FogReading>(m, "FogReading");
    readCopy(fog_reading, "angular_rates", &FogReading::angular_rates);
    readCopy(fog_reading, "covariance", &FogReading::covariance);
    auto dvl_reading = py::class_<DvlReading>(m, "DvlReading")
                           .def_readonly("bottom_distance", &DvlReading::bottom_distance);
    readCopy(dvl_reading, "bottom_relative_velocity", &DvlReading::bottom_relative_velocity);
    readCopy(dvl_reading, "covariance", &DvlReading::covariance);
    py::class_<PressureReading>(m, "PressureReading")
        .def_readonly("absolute_pressure", &PressureReading::absolute_pressure)
        .def_readonly("pressure_variance", &PressureReading::pressure_variance)
        .def_readonly("depth", &PressureReading::depth)
        .def_readonly("depth_variance", &PressureReading::depth_variance);
    bindStream<ImuReading>(m, "ImuSample", "ImuStream");
    bindStream<AttitudeReading>(m, "AttitudeSample", "AttitudeStream");
    bindStream<AhrsReading>(m, "AhrsSample", "AhrsStream");
    bindStream<FogReading>(m, "FogSample", "FogStream");
    bindStream<DvlReading>(m, "DvlSample", "DvlStream");
    bindStream<PressureReading>(m, "PressureSample", "PressureStream");
}
} // namespace robotics::python
