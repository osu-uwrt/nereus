# robotics-pool-viewer: the ROS 2 pool viewer host (window, scene, overlays, camera cards, panels).
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)

# Logic that needs no GL context (tested by tests/host_*).
add_library(rp_viewer_host_core STATIC scenario.cpp jpeg_decode.cpp)
target_include_directories(rp_viewer_host_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/integrations/ros2/viewer/include)
target_link_libraries(rp_viewer_host_core PUBLIC rp_glm rp_mesh_assets Eigen3::Eigen yaml-cpp
  rclcpp::rclcpp tf2_ros::tf2_ros ${geometry_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${std_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS}
  PRIVATE JPEG::JPEG)
target_compile_features(rp_viewer_host_core PUBLIC cxx_std_17)
target_compile_definitions(rp_viewer_host_core PRIVATE RP_PACK_CONTENT="${PROJECT_SOURCE_DIR}/content/packs")

add_library(rp_viewer_host STATIC ros_side.cpp scene_model.cpp overlay_draw.cpp window.cpp app.cpp)
target_include_directories(rp_viewer_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rp_viewer_host PUBLIC rp_viewer_host_core rp_scene_rendering rp_viewer_panels rp_imgui
  ament_index_cpp::ament_index_cpp PRIVATE glfw GLEW::GLEW OpenGL::GL PNG::PNG)
target_compile_definitions(rp_viewer_host PRIVATE
  RP_VIEWER_CONTENT="${PROJECT_SOURCE_DIR}/content/viewer"
  RP_RENDERING_SHADERS="${PROJECT_SOURCE_DIR}/libraries/rendering/shaders")

add_executable(robotics-pool-viewer main.cpp)
target_link_libraries(robotics-pool-viewer PRIVATE rp_viewer_host)
set_target_properties(robotics-pool-viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
install(TARGETS robotics-pool-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/content/viewer/ DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/viewer)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(_host_tests viewer_input tf_tree detection_pose status_lights thruster_visuals camera_geometry jpeg scenario)
  foreach(name ${_host_tests})
    add_executable(host_${name}_test ../tests/host_${name}.cpp)
    target_link_libraries(host_${name}_test PRIVATE rp_viewer_host_core GTest::gtest_main)
    target_compile_definitions(host_${name}_test PRIVATE
      RP_VIEWER_CONTENT="${PROJECT_SOURCE_DIR}/content/viewer")
    target_compile_options(host_${name}_test PRIVATE -UNDEBUG)
    add_test(NAME host_${name} COMMAND host_${name}_test)
  endforeach()
  target_link_libraries(host_jpeg_test PRIVATE JPEG::JPEG)
  # Window/render smoke: needs a display; validates every ImGui frame and writes a screenshot.
  if(DEFINED ENV{DISPLAY} OR DEFINED ENV{WAYLAND_DISPLAY})
    add_test(NAME host_demo_capture COMMAND robotics-pool-viewer --demo
      --scenario ${CMAKE_CURRENT_SOURCE_DIR}/../tests/host_smoke_scenario.json
      --hidden --frames 6 --screenshot ${PROJECT_BINARY_DIR}/host_demo_capture.png)
    set_tests_properties(host_demo_capture PROPERTIES TIMEOUT 60)
  endif()
endif()
