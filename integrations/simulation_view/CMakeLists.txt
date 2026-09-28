add_library(rp_simulation_view src/simulation_view.cpp)
add_library(RoboticsPlatform::simulation_view ALIAS rp_simulation_view)
set_target_properties(rp_simulation_view PROPERTIES EXPORT_NAME simulation_view)
target_include_directories(rp_simulation_view PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_simulation_view PUBLIC rp_simulation rp_visualization)
rp_project_options(rp_simulation_view)
install(TARGETS rp_simulation_view EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(simulation_view_tests tests/simulation_view_test.cpp)
  target_link_libraries(simulation_view_tests PRIVATE rp_simulation_view rp_sensors GTest::gtest_main)
  rp_project_options(simulation_view_tests)
  gtest_discover_tests(simulation_view_tests DISCOVERY_MODE PRE_TEST)
endif()
