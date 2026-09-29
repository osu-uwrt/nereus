#include "pool_viewer/status_lights.hpp"
#include "pool_viewer/thruster_visuals.hpp"
#include <fstream>
#include <iomanip>

int main(int argc, char **argv) {
    if (argc != 4)
        return 2;
    pool::ThrusterVisuals rotors(argv[1], 8);
    pool::StatusLights lights(argv[2]);
    std::ofstream rotorFile(std::string(argv[3]) + "_rotors.csv");
    rotorFile << std::setprecision(17);
    auto rotorRow = [&](bool receive, double now, const std::vector<float> &forces) {
        if (receive)
            rotors.receive(forces, now);
        else
            rotors.advance(now);
        rotorFile << (receive ? 1 : 0) << ',' << now;
        for (float force : forces)
            rotorFile << ',' << force;
        for (const auto &rotor : rotors.rotors) {
            rotorFile << ',' << rotor.angle;
            const auto transform = rotor.transform();
            for (int col = 0; col < 4; ++col)
                for (int row = 0; row < 4; ++row)
                    rotorFile << ',' << transform[col][row];
        }
        rotorFile << '\n';
    };
    std::vector<float> forces(8, 0);
    rotorRow(false, 0, forces);
    for (int sample = 0; sample < 48; ++sample) {
        const double now = sample * .037;
        for (int i = 0; i < 8; ++i)
            forces[i] = static_cast<float>(30 * std::sin(.19 * sample + .43 * i));
        rotorRow(true, now, forces);
        rotorRow(false, now + .011, forces);
        rotorRow(false, now + .011, forces); // paused source clock
    }
    rotorRow(false, 3, forces); // packet timeout, no render-rate clamp
    rotorRow(false, 4, forces);
    rotorRow(false, 0, forces); // rewind resets forces and phase
    rotorRow(false, .1, forces);
    for (float value : {0.f, .005f, .01f, .011f, -.011f, 4.f, -4.f, 24.f, -24.f, 100.f}) {
        forces.assign(8, value);
        const double now = .2 + .1 * (value + 100); // separate epochs deliberately exercise rewinds
        rotorRow(true, now, forces);
        rotorRow(false, now + .01, forces);
    }
    std::ofstream lightFile(std::string(argv[3]) + "_lights.csv");
    lightFile << std::setprecision(17);
    auto lightRow = [&](bool command, pool::LightMode mode, double now, glm::vec3 rgb) {
        if (command)
            lights.command(rgb, mode, UINT32_MAX, now);
        const auto color = lights.lights.front().state.color(now);
        lightFile << command << ',' << int(mode) << ',' << now << ',' << rgb.x << ',' << rgb.y
                  << ',' << rgb.z << ',' << color.x << ',' << color.y << ',' << color.z << '\n';
    };
    lightRow(false, pool::LightMode::Solid, 0, {});
    for (const auto mode : {pool::LightMode::Solid, pool::LightMode::SlowFlash,
                            pool::LightMode::FastFlash, pool::LightMode::Breath}) {
        lightRow(true, mode, 0, {.2f, .7f, 1.f});
        for (int tick = 0; tick <= 96; ++tick)
            lightRow(false, mode, tick / 32., {});
    }
    lightRow(true, pool::LightMode::Flash, 3, {1, 0, .5f});
    lightRow(false, pool::LightMode::Solid, 3.149999, {});
    lightRow(true, pool::LightMode::Solid, 3.1,
             {0, 1, 0});                               // pulse does not change underlying status
    lightRow(false, pool::LightMode::Solid, 3.15, {}); // exclusive end
    lightRow(true, pool::LightMode::Flash, 4, {1, 0, 0});
    lightRow(true, pool::LightMode::Flash, 4.1, {0, 0, 1}); // replacement pulse
    lightRow(false, pool::LightMode::Solid, 4.249999, {});
    lightRow(false, pool::LightMode::Solid, 4.25, {});
    lightRow(true, pool::LightMode::Solid, 5, {-1, 2, .5f}); // original input clamp
    if (!rotorFile || !lightFile)
        return 1;
}
