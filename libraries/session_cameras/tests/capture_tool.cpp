// Writes one always-mode capture as raw files for pixel comparison with pack_cameras.py
// (tests/compare_pack_cameras.py). Usage:
//   session_cameras_capture RESOLVED.json SENSOR X Y Z QW QX QY QZ TIME_S OUT_PREFIX [REPEAT]
// Outputs OUT_PREFIX.rgb (uint8, top-down, h*w*3), .depth (float32 h*w), .json (size, timings).
#include <robotics/session_cameras/session_cameras.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>

namespace sc = robotics::session_cameras;

int main(int argc, char **argv) {
    if (argc < 12) {
        std::cerr << "usage: " << argv[0] << " RESOLVED SENSOR X Y Z QW QX QY QZ TIME_S OUT [REPEAT]\n";
        return 2;
    }
    try {
        const auto resolved = robotics::session::loadResolvedScenario(argv[1]);
        sc::Options options;
        options.always = true;
        options.sensor_ids = {argv[2]};
        sc::SessionCameras cameras(resolved, options);
        robotics::spatial::Pose pose;
        pose.translation = {std::stod(argv[3]), std::stod(argv[4]), std::stod(argv[5])};
        pose.rotation =
            Eigen::Quaterniond(std::stod(argv[6]), std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9]));
        const double time = std::stod(argv[10]);
        const int repeat = argc > 12 ? std::stoi(argv[12]) : 1;
        std::mutex mutex;
        std::condition_variable condition;
        std::vector<sc::Products> products;
        cameras.start([&](sc::Products &&p) {
            std::lock_guard<std::mutex> lock(mutex);
            products.push_back(std::move(p));
            condition.notify_all();
        });
        double render = 0, process = 0, wall = 0;
        for (int i = 0; i < repeat; ++i) {
            const auto start = std::chrono::steady_clock::now();
            cameras.request(std::int64_t(time * 1e9) + std::int64_t(i) * 1'000'000'000, 0, pose);
            std::unique_lock<std::mutex> lock(mutex);
            condition.wait(lock, [&] { return products.size() > std::size_t(i); });
            wall += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            render += products.back().render_ms;
            process += products.back().process_ms;
        }
        cameras.close();
        const auto &first = products.front();
        const std::string prefix = argv[11];
        std::ofstream(prefix + ".rgb", std::ios::binary)
            .write(reinterpret_cast<const char *>(first.left->rgb.data()), std::streamsize(first.left->rgb.size()));
        std::ofstream(prefix + ".depth", std::ios::binary)
            .write(reinterpret_cast<const char *>(first.left->depth.data()),
                   std::streamsize(first.left->depth.size() * sizeof(float)));
        std::ofstream(prefix + ".json") << "{\"width\":" << first.left->width << ",\"height\":" << first.left->height
                                        << ",\"render_ms\":" << render / repeat
                                        << ",\"process_ms\":" << process / repeat << ",\"wall_ms\":" << wall / repeat
                                        << "}\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
