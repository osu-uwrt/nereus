// libjpeg decoding of camera previews, and the single-slot worker thread that runs it off the UI thread.
#include "jpeg_decode.hpp"
#include <csetjmp>
#include <cstdio>
#include <jpeglib.h>

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

AsyncJpegDecoder::AsyncJpegDecoder(int minWidth) : minWidth_(minWidth), worker_([this] { run(); }) {}

// Wake the worker, let it see `stopping_`, and wait for it to exit.
AsyncJpegDecoder::~AsyncJpegDecoder() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    condition_.notify_all();
    worker_.join();
}

// Replace any frame the worker has not started yet (counted as dropped).
void AsyncJpegDecoder::submit(std::vector<std::uint8_t> jpeg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (havePending_)
            ++dropped_;
        pending_ = std::move(jpeg);
        havePending_ = true;
    }
    condition_.notify_one();
}

bool AsyncJpegDecoder::take(DecodedImage &out) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!haveResult_)
        return false;
    out = std::move(result_);
    haveResult_ = false;
    return true;
}

std::uint64_t AsyncJpegDecoder::decoded() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return decoded_;
}

std::uint64_t AsyncJpegDecoder::dropped() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dropped_;
}

std::uint64_t AsyncJpegDecoder::failed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
}

// Worker loop: wait for a pending frame, decode it outside the lock, publish the result (overwriting any
// result the consumer has not taken).
void AsyncJpegDecoder::run() {
    std::vector<std::uint8_t> data;
    DecodedImage image;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [&] { return stopping_ || havePending_; });
            if (stopping_)
                return;
            data = std::move(pending_);
            havePending_ = false;
        }

        const bool ok = decodeJpeg(data.data(), data.size(), minWidth_, image);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ok) {
            ++failed_;
            continue;
        }
        ++decoded_;
        result_ = std::move(image);
        haveResult_ = true;
    }
}
} // namespace nereus::ros_viewer::host
