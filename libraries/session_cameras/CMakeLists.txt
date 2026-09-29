# rp_session_cameras: scheduled, on-demand stereo camera acquisition for a running session (EGL offscreen
# rendering of rp_pack_scene scenes, rp_cameras processing, bounded per-camera workers). No ROS.
find_package(Threads REQUIRED)
add_library(rp_session_cameras src/session_cameras.cpp)
add_library(RoboticsPlatform::session_cameras ALIAS rp_session_cameras)
set_target_properties(rp_session_cameras PROPERTIES EXPORT_NAME session_cameras)
target_include_directories(rp_session_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_session_cameras PUBLIC rp_pack_scene rp_cameras rp_offscreen_rendering rp_session
                      PRIVATE Threads::Threads)
target_compile_definitions(rp_session_cameras PRIVATE
  RP_RENDERING_SHADERS="${PROJECT_SOURCE_DIR}/libraries/rendering/shaders")
rp_project_options(rp_session_cameras)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(rp_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_executable(session_cameras_tests tests/session_cameras_test.cpp)
  target_link_libraries(session_cameras_tests PRIVATE rp_session_cameras GTest::gtest_main)
  target_compile_definitions(session_cameras_tests PRIVATE RP_RESOLVED_TALOS="${rp_resolved_talos}"
    RP_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  rp_project_options(session_cameras_tests)
  gtest_discover_tests(session_cameras_tests DISCOVERY_MODE PRE_TEST PROPERTIES FIXTURES_REQUIRED session_resolved)
endif()
