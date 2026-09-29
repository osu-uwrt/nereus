#pragma once
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <vector>

// Harness-only timing utility, shared with the independent reference wrapper.
// Measures complete GPU draws; no renderer implementation or scene math lives here.
template <class Draw, class Finish>
void benchmark(const std::filesystem::path &output, Draw draw, Finish finish) {
    for (int i = 0; i < 10; ++i) {
        draw(12.5f);
        finish();
    }
    std::vector<double> samples;
    for (int i = 0; i < 120; ++i) {
        const auto start = std::chrono::steady_clock::now();
        draw(12.5f + static_cast<float>(i) / 60.f);
        finish();
        samples.push_back(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count());
    }
    std::sort(samples.begin(), samples.end());
    std::ofstream file(output);
    file << "{\"width\":1280,\"height\":800,\"warmup\":10,\"samples\":120,\"unit\":\"ms per "
            "GPU-complete draw\",\"mean\":"
         << std::accumulate(samples.begin(), samples.end(), 0.) /
                static_cast<double>(samples.size())
         << ",\"p50\":" << samples[samples.size() / 2]
         << ",\"p95\":" << samples[samples.size() * 95 / 100]
         << ",\"p99\":" << samples[samples.size() * 99 / 100] << "}\n";
    file.close();
    if (!file)
        throw std::runtime_error("cannot write render benchmark");
}
