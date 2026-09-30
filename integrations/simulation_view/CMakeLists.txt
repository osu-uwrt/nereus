add_library(nereus_simulation_view src/simulation_view.cpp)
add_library(Nereus::simulation_view ALIAS nereus_simulation_view)
set_target_properties(nereus_simulation_view PROPERTIES EXPORT_NAME simulation_view)
target_include_directories(nereus_simulation_view PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_simulation_view PUBLIC nereus_simulation nereus_visualization)
nereus_project_options(nereus_simulation_view)
install(TARGETS nereus_simulation_view EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(simulation_view_tests tests/simulation_view_test.cpp)
  target_link_libraries(simulation_view_tests PRIVATE nereus_simulation_view nereus_sensors GTest::gtest_main)
  nereus_project_options(simulation_view_tests)
  gtest_discover_tests(simulation_view_tests DISCOVERY_MODE PRE_TEST)
endif()
