# nereus_pack_scene: renderer scene composition from a resolved scenario (pool, task visuals with cutouts
# and texture overrides, robot visuals, caller-posed dynamic instances). Shared by the ROS viewer
# and nereus_session_cameras.
add_library(nereus_pack_scene src/pack_scene.cpp)
add_library(Nereus::pack_scene ALIAS nereus_pack_scene)
set_target_properties(nereus_pack_scene PROPERTIES EXPORT_NAME pack_scene)
target_include_directories(nereus_pack_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_pack_scene PUBLIC nereus_session nereus_scene_rendering nereus_mesh_assets nereus_spatial)
nereus_project_options(nereus_pack_scene)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  set(nereus_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_executable(pack_scene_tests tests/pack_scene_test.cpp)
  target_link_libraries(pack_scene_tests PRIVATE nereus_pack_scene GTest::gtest_main)
  target_compile_definitions(pack_scene_tests PRIVATE NEREUS_RESOLVED_TALOS="${nereus_resolved_talos}"
    NEREUS_RESOLVED_RPAC="${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_rpac_resolved.json"
    NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  nereus_project_options(pack_scene_tests)
  gtest_discover_tests(pack_scene_tests DISCOVERY_MODE PRE_TEST)
endif()
