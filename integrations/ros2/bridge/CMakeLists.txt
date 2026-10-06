# rclcpp simulator bridge (nereus-sim) over nereus_session. nereus_bridge_core is middleware free (rosidl introspection only) and is
# tested against a fake session; the executable adds rclcpp, TF and the real Session adapter.

# ROS packages of the message types the bridge maps, plus TF and the estimator's SetPose service.
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

# Bridge core: everything but the rclcpp node and the camera sink, shared by nereus-sim and the tests.
add_library(nereus_bridge_core STATIC
  src/native.cpp src/ros_types.cpp src/mapping.cpp src/visual.cpp src/core.cpp
  src/session_adapter.cpp src/records.cpp)
target_include_directories(nereus_bridge_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(nereus_bridge_core PUBLIC nereus_session rclcpp::rclcpp rcpputils::rcpputils
  ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${builtin_interfaces_TARGETS} ${robot_localization_TARGETS} ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS})
nereus_project_options(nereus_bridge_core)

# The nereus-sim node; camera acquisition is compiled in only when the session cameras library exists.
add_executable(nereus-sim src/main.cpp src/node.cpp src/camera_sink.cpp)
target_link_libraries(nereus-sim PRIVATE nereus_bridge_core nereus_rules tf2_ros::tf2_ros
  ${tf2_msgs_TARGETS} ${rosgraph_msgs_TARGETS})
if(TARGET nereus_session_cameras)
  target_sources(nereus-sim PRIVATE src/session_camera_sink.cpp)
  target_compile_definitions(nereus-sim PRIVATE NEREUS_BRIDGE_CAMERAS)
  target_link_libraries(nereus-sim PRIVATE nereus_session_cameras)
endif()
nereus_project_options(nereus-sim)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  # The session library's frozen resolved Talos scenario (configured from its tests/fixtures).
  set(nereus_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  # One gtest executable per tests/*_test.cpp.
  file(GLOB nereus_bridge_tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp)
  foreach(test_source ${nereus_bridge_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(bridge_${test_name} ${test_source})
    target_compile_definitions(bridge_${test_name} PRIVATE
      NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}" NEREUS_RESOLVED_TALOS="${nereus_resolved_talos}")
    target_link_libraries(bridge_${test_name} PRIVATE nereus_bridge_core nereus_rules GTest::gtest_main)
    nereus_project_options(bridge_${test_name})
    gtest_discover_tests(bridge_${test_name} DISCOVERY_MODE PRE_TEST)
  endforeach()
endif()
