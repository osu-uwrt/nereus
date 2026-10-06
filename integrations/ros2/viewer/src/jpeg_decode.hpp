// Minimal libjpeg decoder for bridge camera previews (sensor_msgs/CompressedImage, JPEG).
#pragma once
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

namespace nereus::ros_viewer::host {
// A decoded frame (pixels).
struct DecodedImage {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb; // tightly packed RGB8, top row first
};

// Decodes with the largest DCT scaling (1/1, 1/2, 1/4, 1/8) whose width stays at least `minWidth`,
// which makes card previews of 1920 px frames cheap. Returns false on malformed data.
bool decodeJpeg(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out);

// Decodes on a worker thread so the UI/ROS-spin thread never pays for a 1920 px frame. Keeps only the
// newest submitted frame (an older undecoded frame is dropped, never queued); take() hands the newest
// decoded image to the consumer once. submit() and take() only lock a mutex briefly.
class AsyncJpegDecoder {
  public:
    explicit AsyncJpegDecoder(int minWidth);
    ~AsyncJpegDecoder();
    AsyncJpegDecoder(const AsyncJpegDecoder &) = delete;
    AsyncJpegDecoder &operator=(const AsyncJpegDecoder &) = delete;

    void submit(std::vector<std::uint8_t> jpeg);
    bool take(DecodedImage &out); // true when a new image was moved into `out`

    // Counters since construction.
    std::uint64_t decoded() const;
    std::uint64_t dropped() const; // frames replaced before the worker reached them
    std::uint64_t failed() const;

  private:
    void run();

    const int minWidth_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;

    // Guarded by mutex_: the newest undecoded frame, the newest decoded one, and counters.
    std::vector<std::uint8_t> pending_;
    bool havePending_ = false, haveResult_ = false, stopping_ = false;
    DecodedImage result_;
    std::uint64_t decoded_ = 0, dropped_ = 0, failed_ = 0;
    std::thread worker_; // last: starts after every other member exists
};
} // namespace nereus::ros_viewer::host
