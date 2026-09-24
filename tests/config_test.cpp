#include "robotics/config/scenario.hpp"
#include <gtest/gtest.h>

TEST(Configuration, MissingFileIncludesSourcePath) {
    try {
        robotics::config::loadScenario("/does/not/exist/scenario.yaml");
        FAIL() << "Expected load failure";
    } catch (const std::invalid_argument &error) {
        EXPECT_NE(std::string(error.what()).find("/does/not/exist/scenario.yaml"),
                  std::string::npos);
    }
}
