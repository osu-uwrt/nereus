# ROS pool viewer: operator panels and live pool view over the simulator bridge and robot stack.
# Optional outer integration; nothing in libraries/ depends on it.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(riptide_msgs2 QUIET)
find_package(chameleon_tf_msgs QUIET)
option(RP_ROS_VIEWER_UWRT "Build the UWRT (riptide_msgs2) panel providers" ${riptide_msgs2_FOUND})

add_library(rp_glm INTERFACE)
target_include_directories(rp_glm SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/third_party/glm)

# Panels library (composition, providers, panels, overlays): panels/CMakeLists.txt.
add_subdirectory(panels)
# Viewer host application (window, scene, overlays, camera cards): src/.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/CMakeLists.txt)
  add_subdirectory(src)
endif()
