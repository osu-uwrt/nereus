#include "camera_sink.hpp"

namespace nereus::ros_bridge {
// Builds without NEREUS_BUILD_SESSION_CAMERAS have no acquisition; session_camera_sink.cpp provides
// the real factory otherwise.
#ifndef NEREUS_BRIDGE_CAMERAS
std::unique_ptr<CameraSink> createCameraSink(const session::ResolvedScenario &, const std::vector<std::string> &,
                                             SessionPort &, const CameraSinkOptions &) {
    return nullptr;
}
#endif
} // namespace nereus::ros_bridge
