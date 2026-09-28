add_library(rp_spatial src/frames.cpp)
add_library(RoboticsPlatform::spatial ALIAS rp_spatial)
set_target_properties(rp_spatial PROPERTIES EXPORT_NAME spatial)
target_include_directories(rp_spatial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_spatial PUBLIC Eigen3::Eigen)
rp_project_options(rp_spatial)
install(TARGETS rp_spatial EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(spatial_tests tests/frames_test.cpp)
  target_link_libraries(spatial_tests PRIVATE rp_spatial GTest::gtest_main)
  rp_project_options(spatial_tests)
  gtest_discover_tests(spatial_tests DISCOVERY_MODE PRE_TEST)
endif()
