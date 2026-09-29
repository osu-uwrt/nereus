#include "jpeg_decode.hpp"
#include <csetjmp>
#include <cstdio>
#include <jpeglib.h>

namespace robotics::ros_viewer::host {
namespace {
struct ErrorManager {
    jpeg_error_mgr base;
    std::jmp_buf jump;
};
void fail(j_common_ptr info) {
    std::longjmp(reinterpret_cast<ErrorManager *>(info->err)->jump, 1);
}
} // namespace

bool decodeJpeg(const std::uint8_t *data, std::size_t size, int minWidth, DecodedImage &out) {
    if (!data || size < 4)
        return false;
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
    info.out_color_space = JCS_RGB;
    unsigned denominator = 1;
    while (denominator < 8 && static_cast<int>(info.image_width / (denominator * 2)) >= minWidth)
        denominator *= 2;
    info.scale_num = 1;
    info.scale_denom = denominator;
    jpeg_start_decompress(&info);
    out.width = static_cast<int>(info.output_width);
    out.height = static_cast<int>(info.output_height);
    out.rgb.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(out.height) * 3, 0);
    while (info.output_scanline < info.output_height) {
        JSAMPROW row = out.rgb.data() + static_cast<std::size_t>(info.output_scanline) *
                                            static_cast<std::size_t>(out.width) * 3;
        jpeg_read_scanlines(&info, &row, 1);
    }
    jpeg_finish_decompress(&info);
    jpeg_destroy_decompress(&info);
    return true;
}
} // namespace robotics::ros_viewer::host
