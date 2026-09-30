# nereus-viewer: the ROS 2 pool viewer host (window, scene, overlays, camera cards, panels).
find_package(ament_index_cpp REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(Threads REQUIRED)

# Logic that needs no GL context (tested by tests/host_*).
add_library(nereus_viewer_host_core STATIC scenario.cpp jpeg_decode.cpp)
target_include_directories(nereus_viewer_host_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/integrations/ros2/viewer/include)
target_link_libraries(nereus_viewer_host_core PUBLIC nereus_glm nereus_mesh_assets nereus_pack_scene Eigen3::Eigen yaml-cpp
  rclcpp::rclcpp tf2_ros::tf2_ros ${geometry_msgs_TARGETS} ${visualization_msgs_TARGETS}
  ${std_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS}
  PRIVATE JPEG::JPEG Threads::Threads)
target_compile_features(nereus_viewer_host_core PUBLIC cxx_std_17)
target_compile_definitions(nereus_viewer_host_core PRIVATE NEREUS_PACK_CONTENT="${PROJECT_SOURCE_DIR}/content/packs")

add_library(nereus_viewer_host STATIC ros_side.cpp scene_model.cpp overlay_draw.cpp window.cpp app.cpp)
target_include_directories(nereus_viewer_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(nereus_viewer_host PUBLIC nereus_viewer_host_core nereus_scene_rendering nereus_viewer_panels nereus_imgui
  ament_index_cpp::ament_index_cpp PRIVATE glfw GLEW::GLEW OpenGL::GL PNG::PNG)
target_compile_definitions(nereus_viewer_host PRIVATE
  NEREUS_VIEWER_CONTENT="${PROJECT_SOURCE_DIR}/content/viewer"
  NEREUS_RENDERING_SHADERS="${PROJECT_SOURCE_DIR}/libraries/rendering/shaders")

add_executable(nereus-viewer main.cpp)
target_link_libraries(nereus-viewer PRIVATE nereus_viewer_host)
set_target_properties(nereus-viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
install(TARGETS nereus-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/content/viewer/ DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/viewer)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(_host_tests viewer_input tf_tree detection_pose status_lights thruster_visuals camera_geometry jpeg scenario
      display_clock frame_profiler point_cloud mapping_markers)
  foreach(name ${_host_tests})
    add_executable(host_${name}_test ../tests/host_${name}.cpp)
    target_link_libraries(host_${name}_test PRIVATE nereus_viewer_host_core GTest::gtest_main)
    target_compile_definitions(host_${name}_test PRIVATE
      NEREUS_VIEWER_CONTENT="${PROJECT_SOURCE_DIR}/content/viewer")
    target_compile_options(host_${name}_test PRIVATE -UNDEBUG)
    add_test(NAME host_${name} COMMAND host_${name}_test)
  endforeach()
  target_link_libraries(host_jpeg_test PRIVATE JPEG::JPEG Threads::Threads)
  # Window/render smoke: needs a display; validates every ImGui frame and writes a screenshot.
  if(DEFINED ENV{DISPLAY} OR DEFINED ENV{WAYLAND_DISPLAY})
    add_test(NAME host_demo_capture COMMAND nereus-viewer --demo
      --scenario ${CMAKE_CURRENT_SOURCE_DIR}/../tests/host_smoke_scenario.json
      --hidden --frames 6 --screenshot ${PROJECT_BINARY_DIR}/host_demo_capture.png)
    set_tests_properties(host_demo_capture PROPERTIES TIMEOUT 60)
  endif()
endif()
