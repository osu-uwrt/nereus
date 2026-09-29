# rp_pack_scene: renderer scene composition from a resolved scenario (pool, task visuals with cutouts
# and texture overrides, robot visuals, caller-posed dynamic instances). Shared by the ROS viewer
# and rp_session_cameras; mirrors python/src/robotics_platform/pack_cameras.py.
add_library(rp_pack_scene src/pack_scene.cpp)
add_library(RoboticsPlatform::pack_scene ALIAS rp_pack_scene)
set_target_properties(rp_pack_scene PROPERTIES EXPORT_NAME pack_scene)
target_include_directories(rp_pack_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_pack_scene PUBLIC rp_session rp_scene_rendering rp_mesh_assets rp_spatial)
rp_project_options(rp_pack_scene)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(rp_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_executable(pack_scene_tests tests/pack_scene_test.cpp)
  target_link_libraries(pack_scene_tests PRIVATE rp_pack_scene GTest::gtest_main)
  target_compile_definitions(pack_scene_tests PRIVATE RP_RESOLVED_TALOS="${rp_resolved_talos}"
    RP_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  rp_project_options(pack_scene_tests)
  gtest_discover_tests(pack_scene_tests DISCOVERY_MODE PRE_TEST PROPERTIES FIXTURES_REQUIRED session_resolved)
endif()
