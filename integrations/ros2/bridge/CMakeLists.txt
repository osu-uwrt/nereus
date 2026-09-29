# rclcpp simulator bridge (robotics-sim-ros) over rp_session; see integrations/ros2/python for the
# reference Python bridge. rp_bridge_core is middleware free (rosidl introspection only) and is
# tested against a fake session; the executable adds rclcpp, TF and the real Session adapter.
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(robot_localization REQUIRED)

add_library(rp_bridge_core STATIC
  src/native.cpp src/ros_types.cpp src/mapping.cpp src/visual.cpp src/core.cpp
  src/session_adapter.cpp src/records.cpp)
target_include_directories(rp_bridge_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(rp_bridge_core PUBLIC rp_session rclcpp::rclcpp rcpputils::rcpputils
  ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${builtin_interfaces_TARGETS} ${robot_localization_TARGETS} ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS})
rp_project_options(rp_bridge_core)

add_executable(robotics-sim-ros src/main.cpp src/node.cpp src/camera_sink.cpp)
target_link_libraries(robotics-sim-ros PRIVATE rp_bridge_core rp_rules tf2_ros::tf2_ros
  ${tf2_msgs_TARGETS} ${rosgraph_msgs_TARGETS})
if(TARGET rp_session_cameras)
  target_sources(robotics-sim-ros PRIVATE src/session_camera_sink.cpp)
  target_compile_definitions(robotics-sim-ros PRIVATE RP_BRIDGE_CAMERAS)
  target_link_libraries(robotics-sim-ros PRIVATE rp_session_cameras)
endif()
rp_project_options(robotics-sim-ros)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  # Resolved Talos scenario written by the session library's fixture test (session_resolve_talos).
  set(rp_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  file(GLOB rp_bridge_tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp)
  foreach(test_source ${rp_bridge_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(bridge_${test_name} ${test_source})
    target_compile_definitions(bridge_${test_name} PRIVATE
      RP_SOURCE_DIR="${PROJECT_SOURCE_DIR}" RP_RESOLVED_TALOS="${rp_resolved_talos}")
    target_link_libraries(bridge_${test_name} PRIVATE rp_bridge_core rp_rules GTest::gtest_main)
    rp_project_options(bridge_${test_name})
    gtest_discover_tests(bridge_${test_name} DISCOVERY_MODE PRE_TEST
      PROPERTIES FIXTURES_REQUIRED session_resolved)
  endforeach()
endif()
