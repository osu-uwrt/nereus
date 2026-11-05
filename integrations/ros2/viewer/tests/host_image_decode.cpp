// Card previews: compressed camera images decode with DCT scaling, image topics from any encoding.
#include "image_decode.hpp"
#include <chrono>
#include <cstring>
#include <gtest/gtest.h>
#include <jpeglib.h>
#include <limits>
#include <png.h>
#include <thread>

using namespace nereus::ros_viewer::host;
namespace {
// Encodes a packed RGB8 image (w x h) as a quality-95 JPEG in memory with libjpeg.
std::vector<std::uint8_t> encode(int w, int h, const std::vector<std::uint8_t> &rgb) {
    jpeg_compress_struct info;
    jpeg_error_mgr error;
    info.err = jpeg_std_error(&error);
    jpeg_create_compress(&info);
    unsigned char *buffer = nullptr;
    unsigned long size = 0;
    jpeg_mem_dest(&info, &buffer, &size);

    info.image_width = unsigned(w);
    info.image_height = unsigned(h);
    info.input_components = 3;
    info.in_color_space = JCS_RGB;
    jpeg_set_defaults(&info);
    jpeg_set_quality(&info, 95, TRUE);

    jpeg_start_compress(&info, TRUE);
    while (info.next_scanline < info.image_height) {
        JSAMPROW row = const_cast<JSAMPROW>(rgb.data() + std::size_t(info.next_scanline) * std::size_t(w) * 3);
        jpeg_write_scanlines(&info, &row, 1);
    }
    jpeg_finish_compress(&info);

    // libjpeg malloc'd the output buffer; copy it out and free it.
    std::vector<std::uint8_t> out(buffer, buffer + size);
    jpeg_destroy_compress(&info);
    free(buffer);
    return out;
}
} // namespace

// Decoding keeps colors in place, DCT-scales down to (at least) the requested width, and rejects bad data.
TEST(HostJpeg, DecodesColorsAndScalesToTheRequestedWidth) {
    const int w = 256, h = 128;
    std::vector<std::uint8_t> rgb(std::size_t(w) * std::size_t(h) * 3);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            auto *p = &rgb[(std::size_t(y) * std::size_t(w) + std::size_t(x)) * 3];
            p[0] = x < w / 2 ? 230 : 20; // red left, blue right
            p[1] = 20;
            p[2] = x < w / 2 ? 20 : 230;
        }
    const auto data = encode(w, h, rgb);

    DecodedImage full, small;
    ASSERT_TRUE(decodeJpeg(data.data(), data.size(), 256, full));
    ASSERT_EQ(full.width, 256);
    ASSERT_EQ(full.height, 128);
    EXPECT_GT(full.rgb[(10 * 256 + 10) * 3], 200);      // red on the left
    EXPECT_GT(full.rgb[(10 * 256 + 200) * 3 + 2], 200); // blue on the right

    ASSERT_TRUE(decodeJpeg(data.data(), data.size(), 64, small));
    EXPECT_EQ(small.width, 64); // 1/4 scale keeps at least the requested width
    EXPECT_EQ(small.height, 32);
    EXPECT_GT(small.rgb[(5 * 64 + 5) * 3], 200);

    EXPECT_FALSE(decodeJpeg(data.data(), 10, 64, small)); // truncated header
    const std::uint8_t garbage[] = {1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT_FALSE(decodeJpeg(garbage, sizeof(garbage), 64, small));
}

// The worker-thread decoder has a single slot: a burst of submits replaces older frames (counted as dropped),
// the newest one is eventually decoded, and every submit ends up decoded, dropped or failed.
TEST(HostJpeg, AsyncDecoderKeepsOnlyTheNewestFrameAndNeverBlocksTheCaller) {
    const int w = 128, h = 64;
    std::vector<std::uint8_t> red(std::size_t(w) * std::size_t(h) * 3, 20), blue = red;
    for (std::size_t i = 0; i < red.size(); i += 3) {
        red[i] = 230;
        blue[i + 2] = 230;
    }
    const auto redJpeg = encode(w, h, red), blueJpeg = encode(w, h, blue);

    AsyncImageDecoder decoder(64);
    DecodedImage image;
    EXPECT_FALSE(decoder.take(image)); // nothing before the first submit
    for (int i = 0; i < 50; ++i) {     // a burst: the caller only ever copies bytes into the slot
        decoder.submit(redJpeg);
    }
    decoder.submit(blueJpeg);
    for (int i = 0; i < 400 && decoder.decoded() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

    // Wait for the worker to reach the newest frame, whatever it decoded in between.
    bool sawBlue = false;
    for (int i = 0; i < 400 && !sawBlue; ++i) {
        if (decoder.take(image))
            sawBlue = image.rgb[2] > 200;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_TRUE(sawBlue);
    EXPECT_EQ(image.width, 64);
    EXPECT_FALSE(decoder.take(image)); // an image is handed out once

    // A bad frame counts as failed; 50 red + 1 blue + 1 bad = 52 submits in all.
    decoder.submit({1, 2, 3, 4, 5, 6});
    for (int i = 0; i < 400 && decoder.failed() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_EQ(decoder.failed(), 1u);
    EXPECT_EQ(decoder.decoded() + decoder.dropped() + decoder.failed(), 52u);
}

namespace {
RawImage raw(const std::vector<std::uint8_t> &data, int w, int h, std::size_t step, const std::string &encoding) {
    RawImage image;
    image.data = data.data();
    image.size = data.size();
    image.width = w;
    image.height = h;
    image.step = step;
    image.encoding = encoding;
    return image;
}
} // namespace

TEST(RawImage, ColorEncodingsComeOutRgb) {
    DecodedImage out;
    const std::vector<std::uint8_t> bgra = {10, 20, 30, 255};
    ASSERT_TRUE(decodeRaw(raw(bgra, 1, 1, 4, "bgra8"), 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{30, 20, 10}));
    ASSERT_TRUE(decodeRaw(raw({10, 20, 30}, 1, 1, 3, "rgb8"), 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{10, 20, 30}));
    ASSERT_TRUE(decodeRaw(raw({77}, 1, 1, 1, "mono8"), 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{77, 77, 77}));
    // 16 bit color keeps the high byte, in either byte order
    ASSERT_TRUE(decodeRaw(raw({0, 10, 0, 20, 0, 30}, 1, 1, 6, "bgr16"), 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{30, 20, 10}));
    auto big = raw({10, 0, 20, 0, 30, 0}, 1, 1, 6, "rgb16");
    const std::vector<std::uint8_t> bytes = {10, 0, 20, 0, 30, 0};
    big.data = bytes.data();
    big.bigEndian = true;
    ASSERT_TRUE(decodeRaw(big, 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{10, 20, 30}));
}

TEST(RawImage, SubsamplesTowardTheWidthAndHonorsStep) {
    // 8 x 4 mono with two bytes of padding per row
    std::vector<std::uint8_t> pixels(10 * 4, 0);
    pixels[4] = 40;
    DecodedImage out;
    ASSERT_TRUE(decodeRaw(raw(pixels, 8, 4, 10, "mono8"), 2, out));
    EXPECT_EQ(out.width, 2);
    EXPECT_EQ(out.height, 1);
    EXPECT_EQ(out.sourceWidth, 8);
    EXPECT_EQ(out.sourceHeight, 4);
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{0, 0, 0, 40, 40, 40}));
}

TEST(RawImage, DepthIsColoredOverItsValidRange) {
    const float values[3] = {1.f, 3.f, std::numeric_limits<float>::quiet_NaN()};
    std::vector<std::uint8_t> bytes(sizeof(values));
    std::memcpy(bytes.data(), values, sizeof(values));
    DecodedImage out;
    ASSERT_TRUE(decodeRaw(raw(bytes, 3, 1, 12, "32FC1"), 64, out));
    std::uint8_t near[3], far[3], invalid[3];
    colorize(1, 1, 3, near);
    colorize(3, 1, 3, far);
    colorize(std::numeric_limits<float>::quiet_NaN(), 1, 3, invalid);
    EXPECT_EQ(0, std::memcmp(out.rgb.data(), near, 3));
    EXPECT_EQ(0, std::memcmp(out.rgb.data() + 3, far, 3));
    EXPECT_EQ(0, std::memcmp(out.rgb.data() + 6, invalid, 3));
}

TEST(RawImage, Mono16StretchesToItsRange) {
    DecodedImage out;
    ASSERT_TRUE(decodeRaw(raw({100, 0, 200, 0}, 2, 1, 4, "mono16"), 64, out));
    EXPECT_EQ(out.rgb, (std::vector<std::uint8_t>{0, 0, 0, 255, 255, 255}));
}

TEST(RawImage, ReportsWhatItCannotShow) {
    DecodedImage out;
    EXPECT_FALSE(decodeRaw(raw({1, 2, 3, 4}, 2, 1, 4, "yuv422"), 64, out));
    EXPECT_EQ(out.error, "Unsupported encoding: yuv422");
    DecodedImage shortData;
    EXPECT_FALSE(decodeRaw(raw({1, 2}, 2, 2, 2, "mono8"), 64, shortData)); // 2 x 2 needs 4 bytes
    EXPECT_FALSE(shortData.error.empty());
    DecodedImage shortStep;
    EXPECT_FALSE(decodeRaw(raw({1, 2, 3, 4}, 2, 1, 2, "rgb8"), 64, shortStep)); // a row of 2 needs 6 bytes
    EXPECT_FALSE(shortStep.error.empty());
}

TEST(CompressedImage, ByItsSignature) {
    std::vector<std::uint8_t> rgb(std::size_t(64) * 32 * 3, 200);
    const auto jpeg = encode(64, 32, rgb);
    DecodedImage out;
    ASSERT_TRUE(decodeCompressed(jpeg.data(), jpeg.size(), 16, out));
    EXPECT_EQ(out.width, 16);
    EXPECT_EQ(out.sourceWidth, 64);

    png_image png;
    std::memset(&png, 0, sizeof(png));
    png.version = PNG_IMAGE_VERSION;
    png.width = 2;
    png.height = 1;
    png.format = PNG_FORMAT_RGB;
    const std::vector<std::uint8_t> pixels = {1, 2, 3, 4, 5, 6};
    std::vector<std::uint8_t> file(1024);
    png_alloc_size_t size = file.size();
    ASSERT_TRUE(png_image_write_to_memory(&png, file.data(), &size, 0, pixels.data(), 0, nullptr));
    DecodedImage decoded;
    ASSERT_TRUE(decodeCompressed(file.data(), size, 16, decoded));
    EXPECT_EQ(decoded.width, 2);
    EXPECT_EQ(decoded.rgb, pixels);

    const std::uint8_t junk[] = {1, 2, 3, 4, 5, 6, 7, 8};
    DecodedImage failed;
    EXPECT_FALSE(decodeCompressed(junk, sizeof(junk), 16, failed));
    EXPECT_FALSE(failed.error.empty());
}

TEST(AsyncImageDecoder, HandsOverAFailureThatSaysWhy) {
    AsyncImageDecoder decoder(64);
    decoder.submit([](DecodedImage &out) {
        out.error = "Unsupported encoding: x";
        return false;
    });
    DecodedImage image;
    for (int i = 0; i < 400 && !decoder.take(image); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_EQ(image.error, "Unsupported encoding: x");
    EXPECT_EQ(decoder.failed(), 1u);
}
