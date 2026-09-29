# CPU assets are independently usable without visualization, OpenGL or simulation.
find_package(assimp 5 REQUIRED CONFIG)
add_library(rp_mesh_assets src/assets.cpp)
add_library(RoboticsPlatform::mesh_assets ALIAS rp_mesh_assets)
set_target_properties(rp_mesh_assets PROPERTIES EXPORT_NAME mesh_assets)
target_include_directories(rp_mesh_assets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_mesh_assets PUBLIC Eigen3::Eigen PRIVATE assimp::assimp)
rp_project_options(rp_mesh_assets)
install(TARGETS rp_mesh_assets EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/robotics/rendering/assets.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(asset_tests tests/assets_test.cpp)
  target_link_libraries(asset_tests PRIVATE rp_mesh_assets GTest::gtest_main)
  target_compile_definitions(asset_tests PRIVATE
    RP_ASSET_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures"
    RP_VISUAL_CONTENT="${PROJECT_SOURCE_DIR}/content/visuals")
  rp_project_options(asset_tests)
  gtest_discover_tests(asset_tests DISCOVERY_MODE PRE_TEST)
endif()
