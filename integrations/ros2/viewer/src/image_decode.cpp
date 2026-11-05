// Decoding of camera and image topic previews, and the single-slot worker thread that runs it off the UI thread.
#include "image_decode.hpp"
#include <algorithm>
#include <cmath>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <jpeglib.h>
#include <limits>
#include <png.h>

namespace nereus::ros_viewer::host {
namespace {
// libjpeg error manager extended with a jump target, so fatal decode errors return instead of exit().
struct ErrorManager {
    jpeg_error_mgr base;
    std::jmp_buf jump;
};

// error_exit handler: unwind back to the setjmp in decodeJpeg.
void fail(j_common_ptr info) {
    std::longjmp(reinterpret_cast<ErrorManager *>(info->err)->jump, 1);
}
} // namespace

bool decodeJpeg(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out) {
    if (!data || size < 4)
        return false;

    // Route libjpeg errors to the setjmp below (silently: no stderr messages).
    jpeg_decompress_struct info;
    ErrorManager errors;
    info.err = jpeg_std_error(&errors.base);
    errors.base.error_exit = fail;
    errors.base.output_message = [](j_common_ptr) {};
    if (setjmp(errors.jump)) {
        jpeg_destroy_decompress(&info);
        return false;
    }

    jpeg_create_decompress(&info);
    jpeg_mem_src(&info, data, static_cast<unsigned long>(size));
    if (jpeg_read_header(&info, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&info);
        return false;
    }

    // Pick the DCT downscale: halve while the result stays at least minWidth wide (libjpeg max is 1/8).
    info.out_color_space = JCS_RGB;
    unsigned denominator = 1;
    while (denominator < 8 && static_cast<int>(info.image_width / (denominator * 2)) >= minWidth)
        denominator *= 2;
    info.scale_num = 1;
    info.scale_denom = denominator;

    // Decode row by row straight into the output buffer.
    jpeg_start_decompress(&info);
    out.width = static_cast<int>(info.output_width);
    out.height = static_cast<int>(info.output_height);
    out.sourceWidth = static_cast<int>(info.image_width);
    out.sourceHeight = static_cast<int>(info.image_height);
    out.rgb.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3, 0);
    while (info.output_scanline < info.output_height) {
        JSAMPROW row =
            out.rgb.data() + static_cast<std::size_t>(info.output_scanline) * static_cast<std::size_t>(out.width) * 3;
        jpeg_read_scanlines(&info, &row, 1);
    }
    jpeg_finish_decompress(&info);
    jpeg_destroy_decompress(&info);
    return true;
}

bool decodeCompressed(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out) {
    static const std::uint8_t kJpeg[] = {0xff, 0xd8}, kPng[] = {0x89, 'P', 'N', 'G'};
    if (data && size > sizeof(kJpeg) && std::memcmp(data, kJpeg, sizeof(kJpeg)) == 0) {
        if (!decodeJpeg(data, size, minWidth, out)) {
            out.error = "Malformed JPEG";
            return false;
        }
        return true;
    }
    if (!data || size <= sizeof(kPng) || std::memcmp(data, kPng, sizeof(kPng)) != 0) {
        out.error = "Unsupported compressed format";
        return false;
    }

    png_image png;
    std::memset(&png, 0, sizeof(png));
    png.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_memory(&png, data, size)) {
        out.error = "Malformed PNG";
        return false;
    }
    png.format = PNG_FORMAT_RGB;
    out.width = out.sourceWidth = int(png.width);
    out.height = out.sourceHeight = int(png.height);
    out.rgb.resize(PNG_IMAGE_SIZE(png));
    if (!png_image_finish_read(&png, nullptr, out.rgb.data(), 0, nullptr)) {
        png_image_free(&png);
        out.error = "Malformed PNG";
        return false;
    }
    return true;
}

bool decodeRaw(const RawImage &image, int minWidth, DecodedImage &out) {
    // Color and 8 bit mono: bytes per pixel and channel, and which channels are red, green and blue.
    struct Color {
        const char *encoding;
        int channels, channelBytes, r, g, b;
    };
    static const Color kColors[] = {{"rgb8", 3, 1, 0, 1, 2},   {"bgr8", 3, 1, 2, 1, 0},  {"rgba8", 4, 1, 0, 1, 2},
                                    {"bgra8", 4, 1, 2, 1, 0},  {"8UC3", 3, 1, 2, 1, 0},  {"8UC4", 4, 1, 2, 1, 0},
                                    {"rgb16", 3, 2, 0, 1, 2},  {"bgr16", 3, 2, 2, 1, 0}, {"rgba16", 4, 2, 0, 1, 2},
                                    {"bgra16", 4, 2, 2, 1, 0}, {"mono8", 1, 1, 0, 0, 0}, {"8UC1", 1, 1, 0, 0, 0}};
    const Color *color = nullptr;
    for (const auto &candidate : kColors)
        if (image.encoding == candidate.encoding)
            color = &candidate;
    const bool mono16 = image.encoding == "mono16", depth16 = image.encoding == "16UC1",
               float32 = image.encoding == "32FC1";
    if (!color && !mono16 && !depth16 && !float32) {
        out.error = "Unsupported encoding: " + image.encoding;
        return false;
    }

    const int bytes = color ? color->channels * color->channelBytes : float32 ? 4 : 2;
    if (image.width <= 0 || image.height <= 0 || !image.data ||
        image.step < std::size_t(image.width) * std::size_t(bytes) ||
        image.size < image.step * std::size_t(image.height)) {
        out.error = "Malformed image";
        return false;
    }

    const int stride = std::max(1, image.width / std::max(1, minWidth));
    const int w = image.width / stride, h = std::max(1, image.height / stride);
    out.width = w;
    out.height = h;
    out.sourceWidth = image.width;
    out.sourceHeight = image.height;
    out.rgb.resize(std::size_t(w) * std::size_t(h) * 3);
    const auto pixel = [&](int x, int y) {
        return image.data + std::size_t(y * stride) * image.step + std::size_t(x * stride) * std::size_t(bytes);
    };

    if (color) {
        // of a 16 bit channel, its high byte
        const int high = color->channelBytes == 2 && !image.bigEndian ? 1 : 0;
        const auto byteOf = [&](int channel) { return channel * color->channelBytes + high; };
        const int r = byteOf(color->r), g = byteOf(color->g), b = byteOf(color->b);
        std::uint8_t *to = out.rgb.data();
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x, to += 3) {
                const std::uint8_t *from = pixel(x, y);
                to[0] = from[r];
                to[1] = from[g];
                to[2] = from[b];
            }
        return true;
    }

    // One value per pixel: over this frame's valid range (not finite and <= 0 are no data, as in depth images).
    const auto value = [&](int x, int y) {
        const std::uint8_t *from = pixel(x, y);
        if (!float32)
            return float(image.bigEndian ? (from[0] << 8) | from[1] : (from[1] << 8) | from[0]);
        std::uint8_t ordered[4] = {from[0], from[1], from[2], from[3]};
        if (image.bigEndian)
            std::reverse(ordered, ordered + 4);
        float v;
        std::memcpy(&v, ordered, sizeof(v));
        return v;
    };
    float lo = std::numeric_limits<float>::max(), hi = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            const float v = value(x, y);
            if (std::isfinite(v) && v > 0) {
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
        }
    std::uint8_t *to = out.rgb.data();
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x, to += 3) {
            const float v = value(x, y);
            if (mono16)
                to[0] = to[1] = to[2] =
                    v > 0 ? std::uint8_t(255 * std::clamp((v - lo) / std::max(1.f, hi - lo), 0.f, 1.f)) : 0;
            else
                colorize(v, lo, hi, to);
        }
    return true;
}

void colorize(float value, float lo, float hi, std::uint8_t *out) {
    if (!std::isfinite(value) || value <= 0) {
        out[0] = out[1] = out[2] = 8;
        return;
    }
    const float t = std::clamp((value - lo) / std::max(1e-3f, hi - lo), 0.f, 1.f);
    static const float stops[5][3] = {{255, 214, 84}, {96, 214, 110}, {48, 200, 210}, {60, 100, 220}, {60, 40, 110}};
    const float x = t * 4;
    const int i = std::min(3, int(x));
    const float f = x - float(i);
    for (int c = 0; c < 3; ++c)
        out[c] = std::uint8_t(stops[i][c] + (stops[i + 1][c] - stops[i][c]) * f);
}

AsyncImageDecoder::AsyncImageDecoder(int minWidth) : minWidth_(minWidth), worker_([this] { run(); }) {}

// Wake the worker, let it see `stopping_`, and wait for it to exit.
AsyncImageDecoder::~AsyncImageDecoder() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    condition_.notify_all();
    worker_.join();
}

void AsyncImageDecoder::submit(std::vector<std::uint8_t> jpeg) {
    submit([this, jpeg = std::move(jpeg)](DecodedImage &out) {
        return decodeJpeg(jpeg.data(), jpeg.size(), minWidth_, out);
    });
}

// Replace any frame the worker has not started yet (counted as dropped).
void AsyncImageDecoder::submit(Job job) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pending_)
            ++dropped_;
        pending_ = std::move(job);
    }
    condition_.notify_one();
}

bool AsyncImageDecoder::take(DecodedImage &out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!haveResult_)
        return false;
    out = std::move(result_);
    haveResult_ = false;
    return true;
}

std::uint64_t AsyncImageDecoder::decoded() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return decoded_;
}

std::uint64_t AsyncImageDecoder::dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
}

std::uint64_t AsyncImageDecoder::failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
}

// Worker loop: wait for a pending frame, decode it outside the lock, publish the result (overwriting any
// result the consumer has not taken).
void AsyncImageDecoder::run() {
    Job job;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [&] { return stopping_ || pending_; });
            if (stopping_)
                return;
            job = std::move(pending_);
            pending_ = nullptr;
        }

        DecodedImage image;
        const bool ok = job(image);
        job = nullptr; // frees the frame outside the lock
        std::lock_guard<std::mutex> lock(mutex_);
        if (ok)
            ++decoded_;
        else
            ++failed_;
        if (!ok && image.error.empty())
            continue;
        result_ = std::move(image);
        haveResult_ = true;
    }
}
} // namespace nereus::ros_viewer::host
