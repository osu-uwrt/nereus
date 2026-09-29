#include "camera_sink.hpp"

namespace robotics::ros_bridge {
// Camera acquisition is built by the cpp_cameras runtime; until it is wired in here the bridge
// only runs with --no-cameras. Replace this body with the factory of robotics/session_cameras.
std::unique_ptr<CameraSink> createCameraSink(const session::ResolvedScenario &,
                                             const std::vector<std::string> &) {
    return nullptr;
}
} // namespace robotics::ros_bridge
