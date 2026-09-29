#include "robotics/sensors/models.hpp"
#include "robotics/sensors/runtime.hpp"
#include <iostream>

int main() {
    namespace sim = robotics::simulation;
    namespace sensors = robotics::sensors;
    using namespace std::chrono_literals;
    sim::PlantParameters parameters;
    sim::BodyState initial;
    initial.position = {5, 5, -2};
    initial.linear_velocity.x() = 0.5;
    sensors::Runtime runtime(parameters, initial, 42);
    auto imu = runtime.add({"imu", "imu_link", 10ms}, sensors::Imu{});
    auto velocity = runtime.add({"velocity", "velocity_link", 10ms}, sensors::ReferenceVelocity{});
    auto altitude = runtime.add({"altitude", "world", 50ms}, sensors::ReferenceAltitude{});
    sensors::AhrsParameters ahrs_parameters;
    ahrs_parameters.attitude.heading_drift_rate = .1;
    auto ahrs = runtime.add({"ahrs", "imu_link", 20ms}, sensors::Ahrs({}, ahrs_parameters));
    auto attitude = runtime.add({"attitude", "imu_link", 20ms},
                                sensors::Attitude({}, ahrs_parameters.attitude));
    auto fog = runtime.add({"fog", "fog_link", 20ms}, sensors::Fog{});
    auto dvl = runtime.add({"dvl", "dvl_link", 100ms, 20ms},
                           sensors::Dvl({}, sensors::PoolBottom(parameters.pool)));
    auto pressure = runtime.add(
        {"pressure", "pressure_link", 50ms},
        sensors::Pressure({}, sensors::HydrostaticPressure(parameters.pool.water_level,
                                                           parameters.pool.water_density)));
    std::cout << "device,acquired_ns,delivered_ns,x,y,z\n";
    for (int tick = 0; tick < 500; ++tick) {
        runtime.advance();
        for (const auto &sample : imu->drain()) {
            if (!sample.measurement.value) {
                return 1;
            }
            const auto &value = sample.measurement.value->specific_force;
            std::cout << "imu," << sample.header.acquired.count() << ','
                      << sample.header.delivered.count() << ',' << value.x() << ',' << value.y()
                      << ',' << value.z() << '\n';
        }
        for (const auto &sample : fog->drain()) {
            if (!sample.measurement.value) {
                return 1;
            }
        }
        for (const auto &sample : ahrs->drain()) {
            if (!sample.measurement.value || !attitude->latest() ||
                !attitude->latest()->measurement.value ||
                sample.measurement.value->attitude.sensor_to_world.angularDistance(
                    attitude->latest()->measurement.value->sensor_to_world) > 1e-12)
                return 1;
        }
        attitude->drain();
        for (const auto &sample : velocity->drain()) {
            if (!sample.measurement.value ||
                sample.measurement.value->reference_relative_velocity !=
                    runtime.observe().body.linear_velocity)
                return 1;
        }
        for (const auto &sample : altitude->drain()) {
            if (!sample.measurement.value ||
                sample.measurement.value->target_world_z != runtime.observe().body.position.z())
                return 1;
        }
        for (const auto &sample : pressure->drain()) {
            if (!sample.measurement.value) {
                return 1;
            }
        }
        for (const auto &sample : dvl->drain()) {
            if (!sample.measurement.value) {
                return 1;
            }
            const auto &value = sample.measurement.value->bottom_relative_velocity;
            std::cout << "dvl," << sample.header.acquired.count() << ','
                      << sample.header.delivered.count() << ',' << value.x() << ',' << value.y()
                      << ',' << value.z() << '\n';
        }
    }
    // The final DVL acquisition is still pending its configured 20ms latency.
    if (imu->stats().delivered != 100 || fog->stats().delivered != 50 ||
        dvl->stats().delivered != 9 || pressure->stats().delivered != 20 ||
        ahrs->stats().delivered != 50 || attitude->stats().delivered != 50 ||
        velocity->stats().delivered != 100 || altitude->stats().delivered != 20) {
        return 1;
    }
    runtime.reset(initial, 42);
    return imu->latest() || fog->latest() || dvl->latest() || pressure->latest() ||
                   ahrs->latest() || attitude->latest() || velocity->latest() || altitude->latest()
               ? 1
               : 0;
}
