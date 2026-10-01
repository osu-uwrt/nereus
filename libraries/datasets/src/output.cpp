#include <nereus/datasets/output.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace nereus::datasets {
void writeAtomic(const std::filesystem::path &path, const void *data, std::size_t size, bool sync_directory) {
    const auto temporary = path.string() + ".tmp." + std::to_string(::getpid());
    const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0)
        throw std::runtime_error("cannot create " + temporary + ": " + std::strerror(errno));
    const auto *bytes = static_cast<const char *>(data);
    std::size_t written = 0;
    bool ok = true;
    while (ok && written < size) {
        const auto n = ::write(fd, bytes + written, size - written);
        if (n < 0 && errno == EINTR)
            continue;
        ok = n > 0;
        if (ok)
            written += static_cast<std::size_t>(n);
    }
    ok = ok && ::fsync(fd) == 0; // contents durable before the name points at them
    const int error = errno;
    ok = ::close(fd) == 0 && ok;
    if (!ok) {
        ::unlink(temporary.c_str());
        throw std::runtime_error("cannot write " + temporary + ": " + std::strerror(error));
    }
    if (::rename(temporary.c_str(), path.c_str()) != 0) {
        const int renameError = errno;
        ::unlink(temporary.c_str());
        throw std::runtime_error("cannot rename " + temporary + " -> " + path.string() + ": " +
                                 std::strerror(renameError));
    }
    if (sync_directory) {
        const auto parent = path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path();
        const int dir = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
        if (dir < 0 || ::fsync(dir) != 0) {
            const int dirError = errno;
            if (dir >= 0)
                ::close(dir);
            throw std::runtime_error("cannot sync " + parent.string() + ": " + std::strerror(dirError));
        }
        ::close(dir);
    }
}

bool nonEmptyFile(const std::filesystem::path &path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error) && std::filesystem::file_size(path, error) > 0 && !error;
}

float linearDepth(float z, float near, float far) {
    if (!(z < 1.f))
        return std::numeric_limits<float>::infinity();
    return 2 * near * far / (far + near - (2 * z - 1) * (far - near));
}

double KeyStats::medianDepth() const {
    if (depth_m.empty())
        return std::numeric_limits<double>::quiet_NaN();
    auto copy = depth_m;
    const auto middle = copy.begin() + static_cast<std::ptrdiff_t>(copy.size() / 2);
    std::nth_element(copy.begin(), middle, copy.end());
    return *middle;
}

bool KeyStats::fragmented(std::int64_t min_visible_px, std::int64_t min_fragment_px) const {
    if (pixels < min_visible_px)
        return false;
    int large = 0;
    for (const auto size : components)
        large += size >= min_fragment_px;
    return large >= 2;
}

void measureComponents(const rendering::LabelCapture &capture, LabelStats &stats) {
    const int w = capture.width, h = capture.height;
    for (auto &[key, item] : stats.keys) {
        item.components.clear();
        if (item.pixels == 0)
            continue;
        // Mask of this key inside its (top-down) bounding box.
        const int bw = item.max_x - item.min_x + 1, bh = item.max_y - item.min_y + 1;
        cv::Mat mask(bh, bw, CV_8UC1);
        for (int y = 0; y < bh; ++y) {
            const auto *row = capture.ids.data() + static_cast<std::size_t>(h - 1 - (item.min_y + y)) * w;
            auto *out = mask.ptr<std::uint8_t>(y);
            for (int x = 0; x < bw; ++x)
                out[x] = row[item.min_x + x] == key ? 1 : 0;
        }
        cv::Mat labels, sizes, centroids;
        const int count = cv::connectedComponentsWithStats(mask, labels, sizes, centroids, 8, CV_32S);
        for (int c = 1; c < count; ++c)
            item.components.push_back(sizes.at<int>(c, cv::CC_STAT_AREA));
        std::sort(item.components.begin(), item.components.end(), std::greater<>());
    }
}

LabelStats analyzeLabels(const rendering::LabelCapture &capture, float near_plane, float far_plane, float near_m,
                         const std::vector<std::uint8_t> *exclude) {
    const int w = capture.width, h = capture.height;
    const auto pixels = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    if (capture.ids.size() != pixels || capture.depth.size() != pixels || (exclude && exclude->size() != pixels))
        throw std::invalid_argument("label capture buffers do not match its size");
    // Nonlinear depth of near_m: compare in nonlinear space (monotonic) and convert only labelled pixels.
    const float nearZ = ((far_plane + near_plane) - 2 * near_plane * far_plane / near_m) / (far_plane - near_plane);
    const float nearThreshold = (nearZ + 1) / 2;
    LabelStats stats;
    std::uint32_t lastKey = 0;
    KeyStats *last = nullptr;
    for (int row = 0; row < h; ++row) {
        const int y = h - 1 - row; // top-down
        for (int x = 0; x < w; ++x) {
            const auto i = static_cast<std::size_t>(row) * w + x;
            const float z = capture.depth[i];
            if (!exclude || !(*exclude)[i]) {
                ++stats.counted_pixels;
                if (z < nearThreshold)
                    ++stats.near_pixels;
            }
            const auto key = capture.ids[i];
            if ((key & 0xffu) == 0)
                continue;
            if (!last || key != lastKey) {
                last = &stats.keys[key];
                lastKey = key;
            }
            auto &k = *last;
            if (k.pixels++ == 0) {
                k.min_x = k.max_x = x;
                k.min_y = k.max_y = y;
            } else {
                k.min_x = std::min(k.min_x, x), k.max_x = std::max(k.max_x, x);
                k.min_y = std::min(k.min_y, y), k.max_y = std::max(k.max_y, y);
            }
            k.truncated = k.truncated || x == 0 || y == 0 || x == w - 1 || y == h - 1;
            k.depth_m.push_back(linearDepth(z, near_plane, far_plane));
        }
    }
    return stats;
}

std::vector<std::uint16_t> idMap(const rendering::LabelCapture &capture,
                                 const std::map<std::uint32_t, std::uint16_t> &table) {
    const int w = capture.width, h = capture.height;
    std::vector<std::uint16_t> out(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0);
    std::uint32_t lastKey = 0;
    std::uint16_t lastId = 0;
    for (int row = 0; row < h; ++row) {
        const auto *source = capture.ids.data() + static_cast<std::size_t>(row) * w;
        auto *target = out.data() + static_cast<std::size_t>(h - 1 - row) * w;
        for (int x = 0; x < w; ++x) {
            const auto key = source[x];
            if ((key & 0xffu) == 0)
                continue;
            if (key != lastKey) {
                const auto found = table.find(key);
                lastKey = key;
                lastId = found == table.end() ? 0 : found->second;
            }
            target[x] = lastId;
        }
    }
    return out;
}

void postProcess(std::vector<std::uint8_t> &rgb, int width, int height, double blur_sigma, double noise_sigma,
                 std::uint64_t noise_seed) {
    if (rgb.size() != static_cast<std::size_t>(width) * height * 3)
        throw std::invalid_argument("RGB buffer does not match its size");
    cv::Mat image(height, width, CV_8UC3, rgb.data());
    if (blur_sigma > 1e-3)
        cv::GaussianBlur(image, image, cv::Size(0, 0), blur_sigma, blur_sigma, cv::BORDER_REFLECT_101);
    if (noise_sigma > 1e-3) {
        cv::Mat noise(height, width, CV_16SC3);
        cv::RNG random(noise_seed);
        random.fill(noise, cv::RNG::NORMAL, cv::Scalar::all(0), cv::Scalar::all(noise_sigma));
        cv::Mat sum;
        cv::add(image, noise, sum, cv::noArray(), CV_16SC3);
        sum.convertTo(image, CV_8UC3); // saturates
    }
}

std::vector<std::uint8_t> encodePngRgb(const std::vector<std::uint8_t> &rgb, int width, int height) {
    const cv::Mat image(height, width, CV_8UC3, const_cast<std::uint8_t *>(rgb.data()));
    cv::Mat bgr;
    cv::cvtColor(image, bgr, cv::COLOR_RGB2BGR);
    std::vector<std::uint8_t> out;
    if (!cv::imencode(".png", bgr, out, {cv::IMWRITE_PNG_COMPRESSION, 1}))
        throw std::runtime_error("PNG encoding failed");
    return out;
}

std::vector<std::uint8_t> encodePng16(const std::vector<std::uint16_t> &gray, int width, int height) {
    const cv::Mat image(height, width, CV_16UC1, const_cast<std::uint16_t *>(gray.data()));
    std::vector<std::uint8_t> out;
    if (!cv::imencode(".png", image, out, {cv::IMWRITE_PNG_COMPRESSION, 1}))
        throw std::runtime_error("16-bit PNG encoding failed");
    return out;
}

std::vector<std::uint16_t> decodePng16(const std::filesystem::path &path, int &width, int &height) {
    const cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty() || image.type() != CV_16UC1)
        throw std::runtime_error("not a 16-bit grayscale PNG: " + path.string());
    width = image.cols;
    height = image.rows;
    std::vector<std::uint16_t> out(static_cast<std::size_t>(width) * height);
    for (int y = 0; y < height; ++y)
        std::copy_n(image.ptr<std::uint16_t>(y), width, out.data() + static_cast<std::size_t>(y) * width);
    return out;
}
} // namespace nereus::datasets
