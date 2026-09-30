add_library(nereus_spatial src/frames.cpp)
add_library(Nereus::spatial ALIAS nereus_spatial)
set_target_properties(nereus_spatial PROPERTIES EXPORT_NAME spatial)
target_include_directories(nereus_spatial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_spatial PUBLIC Eigen3::Eigen)
nereus_project_options(nereus_spatial)
install(TARGETS nereus_spatial EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(spatial_tests tests/frames_test.cpp)
  target_link_libraries(spatial_tests PRIVATE nereus_spatial GTest::gtest_main)
  nereus_project_options(spatial_tests)
  gtest_discover_tests(spatial_tests DISCOVERY_MODE PRE_TEST)
endif()
