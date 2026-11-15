// The Dock icon on macOS: a plain executable gets the generic "exec" tile, and glfwSetWindowIcon does nothing
// there, so set NSApp's image (GLFW has already created NSApp).
#include "dock_icon.hpp"
#import <AppKit/AppKit.h>

namespace nereus::ros_viewer::host {
void setDockIcon(const std::filesystem::path &png) {
    @autoreleasepool {
        NSImage *image = [[NSImage alloc] initWithContentsOfFile:[NSString stringWithUTF8String:png.c_str()]];
        if (!image)
            return;
        [NSApp setApplicationIconImage:image];
        [image release];
    }
}
} // namespace nereus::ros_viewer::host
