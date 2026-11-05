// Image decoding for the camera and image topic cards: JPEG and PNG (sensor_msgs/CompressedImage) and raw pixels
// (sensor_msgs/Image), with a worker thread to run it off the UI thread.
#pragma once
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace nereus::ros_viewer::host {
// A decoded frame (pixels).
struct DecodedImage {
    int width = 0, height = 0;
    int sourceWidth = 0, sourceHeight = 0; // of the image before any downscaling
    std::vector<std::uint8_t> rgb;         // tightly packed RGB8, top row first
    std::string error;                     // why a frame could not be decoded, for the card to show
};

// The pixels of a sensor_msgs/Image.
struct RawImage {
    const std::uint8_t *data = nullptr;
    std::size_t size = 0, step = 0;
    int width = 0, height = 0;
    std::string encoding;
    bool bigEndian = false;
};

// Decodes with the largest DCT scaling (1/1, 1/2, 1/4, 1/8) whose width stays at least `minWidth`,
// which makes card previews of 1920 px frames cheap. Returns false on malformed data.
bool decodeJpeg(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out);

// A CompressedImage by its signature: JPEG (as decodeJpeg) or PNG (full size). False with out.error otherwise.
bool decodeCompressed(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out);

// Raw pixels subsampled toward minWidth. Color and mono encodings (8 bit, and 16 bit at 8 bit precision); 16UC1
// and 32FC1 go through colorize() over the frame's own valid range, mono16 to gray over that range. False with
// out.error on an unknown encoding or too little data.
bool decodeRaw(const RawImage &image, int minWidth, DecodedImage &out);

// Turbo-like ramp for depth: near = warm, far = cool, invalid (not finite or <= 0) = dark. Writes one RGB8 pixel.
void colorize(float value, float lo, float hi, std::uint8_t *out);

// Decodes on a worker thread so the UI/ROS-spin thread never pays for a 1920 px frame. Keeps only the
// newest submitted frame (an older undecoded frame is dropped, never queued); take() hands the newest
// decoded image to the consumer once. submit() and take() only lock a mutex briefly.
class AsyncImageDecoder {
  public:
    explicit AsyncImageDecoder(int minWidth);
    ~AsyncImageDecoder();
    AsyncImageDecoder(const AsyncImageDecoder &) = delete;
    AsyncImageDecoder &operator=(const AsyncImageDecoder &) = delete;

    using Job = std::function<bool(DecodedImage &)>;

    void submit(std::vector<std::uint8_t> jpeg);
    // Any decode, run on the worker. A failed one that set `error` is still handed to take(), so it can be shown.
    void submit(Job job);
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
    Job pending_;
    bool haveResult_ = false, stopping_ = false;
    DecodedImage result_;
    std::uint64_t decoded_ = 0, dropped_ = 0, failed_ = 0;
    std::thread worker_; // last: starts after every other member exists
};
} // namespace nereus::ros_viewer::host
