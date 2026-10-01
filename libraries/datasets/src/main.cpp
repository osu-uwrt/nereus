// nereus-dataset-render JOB.json [--shard I/N] [--limit K] [--shaders DIR] [--describe] [--no-resume]
//                       [--accept-scale F]
// Renders the job's samples k with k % N == I (default: all) into the job's output folder; see generator.hpp.
#include <nereus/datasets/generator.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <vector>

namespace {
namespace ds = nereus::datasets;

int usage(const char *what = nullptr) {
    if (what)
        std::cerr << "nereus-dataset-render: " << what << "\n";
    std::cerr << "usage: nereus-dataset-render JOB.json [--shard I/N] [--limit K] [--shaders DIR] [--describe]\n"
                 "                             [--no-resume] [--accept-scale F]\n";
    return 2;
}

// Temporary files of killed renderers (`<name>.tmp.<pid>` whose process is gone); live shards keep theirs.
void removeStaleTemporaries(const std::filesystem::path &output) {
    for (const char *folder : {"images", "ids", "records"})
        for (const auto &entry : std::filesystem::directory_iterator(output / folder)) {
            const auto name = entry.path().filename().string();
            const auto at = name.rfind(".tmp.");
            if (at == std::string::npos)
                continue;
            const auto pid = std::atol(name.c_str() + at + 5);
            if (pid > 0 && ::kill(static_cast<pid_t>(pid), 0) != 0 && errno == ESRCH)
                std::filesystem::remove(entry.path());
        }
}

double seconds(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}
} // namespace

int main(int argc, char **argv) {
    std::string jobPath;
    long long shard = 0, shards = 1, limit = -1;
    bool describe = false;
    ds::GeneratorOptions options;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            const auto value = [&]() -> std::string {
                if (i + 1 >= argc)
                    throw std::invalid_argument(arg + " needs a value");
                return argv[++i];
            };
            if (arg == "--shard") {
                const auto text = value();
                const auto slash = text.find('/');
                if (slash == std::string::npos)
                    throw std::invalid_argument("--shard expects I/N");
                shard = std::stoll(text.substr(0, slash));
                shards = std::stoll(text.substr(slash + 1));
                if (shards < 1 || shard < 0 || shard >= shards)
                    throw std::invalid_argument("--shard expects 0 <= I < N");
            } else if (arg == "--limit")
                limit = std::stoll(value());
            else if (arg == "--shaders")
                options.shader_directory = value();
            else if (arg == "--describe")
                describe = true;
            else if (arg == "--no-resume")
                options.resume = false;
            else if (arg == "--resume")
                options.resume = true;
            else if (arg == "--accept-scale")
                options.acceptance_scale = std::stod(value());
            else if (arg == "-h" || arg == "--help")
                return usage(), 0;
            else if (!arg.empty() && arg[0] == '-')
                throw std::invalid_argument("unknown option " + arg);
            else if (jobPath.empty())
                jobPath = arg;
            else
                throw std::invalid_argument("more than one job file");
        }
        if (jobPath.empty())
            throw std::invalid_argument("missing JOB.json");
    } catch (const std::exception &error) {
        return usage(error.what());
    }

    try {
        ds::Generator generator(ds::loadJob(jobPath), options);
        if (describe) {
            std::cout << generator.describe().dump(1) << "\n";
            return 0;
        }
        const auto &job = generator.job();
        for (const char *folder : {"images", "ids", "records", "logs"})
            std::filesystem::create_directories(job.output / folder);
        removeStaleTemporaries(job.output);
        const auto logPath =
            job.output / "logs" / ("shard_" + std::to_string(shard) + "_of_" + std::to_string(shards) + ".jsonl");
        std::ofstream log(logPath, std::ios::app);
        if (!log)
            throw std::runtime_error("cannot open " + logPath.string());

        const auto total = job.sampleCount();
        std::cerr << "nereus-dataset-render: " << job.dataset << ", " << total << " samples, shard " << shard << "/"
                  << shards << ", device " << generator.device() << "\n";
        const auto start = std::chrono::steady_clock::now();
        long long done = 0, accepted = 0, skipped = 0, existing = 0, attempts = 0;
        const auto progress = [&](const char *prefix) {
            const double elapsed = seconds(start), rendered = double(accepted + skipped);
            std::fprintf(stderr,
                         "%s%lld samples (%lld accepted, %lld skipped, %lld existing), %.2f samples/s, %.2f s/sample, "
                         "acceptance %.1f%% of attempts\n",
                         prefix, done, accepted, skipped, existing, rendered / std::max(elapsed, 1e-9),
                         rendered > 0 ? elapsed / rendered : 0.0, attempts ? 100.0 * accepted / attempts : 0.0);
        };
        // This shard's samples grouped by scenario (k % scenarios), then by k: one scenario's meshes stay resident
        // instead of alternating every sample. Each sample's output depends only on k.
        const auto scenarios = static_cast<long long>(job.scenarios.size());
        std::vector<long long> order;
        for (long long k = shard; k < total; k += shards)
            order.push_back(k);
        std::stable_sort(order.begin(), order.end(),
                         [&](long long a, long long b) { return a % scenarios < b % scenarios; });
        for (const long long k : order) {
            if (limit >= 0 && accepted + skipped >= limit)
                break;
            const auto line = generator.render(k);
            const auto status = line.at("status").get<std::string>();
            log << line.dump() << "\n" << std::flush;
            ++done;
            if (status == "existing") {
                ++existing;
                continue;
            }
            attempts += line.value("attempts", 0LL);
            (status == "accepted" ? accepted : skipped)++;
            if ((accepted + skipped) % 25 == 0)
                progress("progress: ");
        }
        progress("done: ");
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "nereus-dataset-render: " << error.what() << "\n";
        return 1;
    }
}
