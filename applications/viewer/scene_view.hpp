#pragma once
#include "interface.hpp"

namespace robotics::viewer {
// Empty shader path selects the installed executable's sibling data directory.
SceneDraw sceneDrawer(std::filesystem::path shaders = {});
} // namespace robotics::viewer
