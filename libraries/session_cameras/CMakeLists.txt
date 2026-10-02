# nereus_session_cameras: scheduled, on-demand stereo camera acquisition for a running session (EGL offscreen
# rendering of nereus_pack_scene scenes, nereus_cameras processing, bounded per-camera workers). No ROS.
find_package(Threads REQUIRED)
add_library(nereus_session_cameras src/session_cameras.cpp)
add_library(Nereus::session_cameras ALIAS nereus_session_cameras)
set_target_properties(nereus_session_cameras PROPERTIES EXPORT_NAME session_cameras)
target_include_directories(nereus_session_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_session_cameras PUBLIC nereus_pack_scene nereus_cameras nereus_offscreen_rendering nereus_session
                      PRIVATE Threads::Threads)
target_compile_definitions(nereus_session_cameras PRIVATE
  NEREUS_RENDERING_SHADERS="${PROJECT_SOURCE_DIR}/libraries/rendering/shaders")
nereus_project_options(nereus_session_cameras)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(nereus_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_executable(session_cameras_tests tests/session_cameras_test.cpp)
  target_link_libraries(session_cameras_tests PRIVATE nereus_session_cameras GTest::gtest_main)
  target_compile_definitions(session_cameras_tests PRIVATE NEREUS_RESOLVED_TALOS="${nereus_resolved_talos}"
    NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  nereus_project_options(session_cameras_tests)
  add_executable(session_cameras_capture tests/capture_tool.cpp)
  target_link_libraries(session_cameras_capture PRIVATE nereus_session_cameras)
  nereus_project_options(session_cameras_capture)
  gtest_discover_tests(session_cameras_tests DISCOVERY_MODE PRE_TEST)
endif()
