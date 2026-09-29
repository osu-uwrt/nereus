// Card previews of the bridge's compressed camera images decode with DCT scaling.
#include "jpeg_decode.hpp"
#include <gtest/gtest.h>
#include <jpeglib.h>
#include <chrono>
#include <thread>

using namespace robotics::ros_viewer::host;
namespace {
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
    std::vector<std::uint8_t> out(buffer, buffer + size);
    jpeg_destroy_compress(&info);
    free(buffer);
    return out;
}
} // namespace

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
    EXPECT_GT(full.rgb[(10 * 256 + 10) * 3], 200);            // red on the left
    EXPECT_GT(full.rgb[(10 * 256 + 200) * 3 + 2], 200);       // blue on the right
    ASSERT_TRUE(decodeJpeg(data.data(), data.size(), 64, small));
    EXPECT_EQ(small.width, 64); // 1/4 scale keeps at least the requested width
    EXPECT_EQ(small.height, 32);
    EXPECT_GT(small.rgb[(5 * 64 + 5) * 3], 200);
    EXPECT_FALSE(decodeJpeg(data.data(), 10, 64, small)); // truncated header
    const std::uint8_t garbage[] = {1, 2, 3, 4, 5, 6, 7, 8};
    EXPECT_FALSE(decodeJpeg(garbage, sizeof(garbage), 64, small));
}

TEST(HostJpeg, AsyncDecoderKeepsOnlyTheNewestFrameAndNeverBlocksTheCaller) {
    const int w = 128, h = 64;
    std::vector<std::uint8_t> red(std::size_t(w) * std::size_t(h) * 3, 20), blue = red;
    for (std::size_t i = 0; i < red.size(); i += 3) {
        red[i] = 230;
        blue[i + 2] = 230;
    }
    const auto redJpeg = encode(w, h, red), blueJpeg = encode(w, h, blue);
    AsyncJpegDecoder decoder(64);
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
    decoder.submit({1, 2, 3, 4, 5, 6});
    for (int i = 0; i < 400 && decoder.failed() == 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_EQ(decoder.failed(), 1u);
    EXPECT_EQ(decoder.decoded() + decoder.dropped() + decoder.failed(), 52u);
}
