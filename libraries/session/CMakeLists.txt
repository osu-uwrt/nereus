# rp_session: the C++ simulation runtime over resolved packs (plant, sensors, mechanisms,
# payloads, prop worlds, task observers, run control). Sources are globbed per component folder
# so components can be developed independently: src/*.cpp, src/tasks/*.cpp.
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Bullet REQUIRED)
file(GLOB rp_session_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/*.cpp)
add_library(rp_session ${rp_session_sources})
add_library(RoboticsPlatform::session ALIAS rp_session)
set_target_properties(rp_session PROPERTIES EXPORT_NAME session)
target_include_directories(rp_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(rp_session SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(rp_session PUBLIC rp_sensors rp_spatial nlohmann_json::nlohmann_json
                      PRIVATE ${BULLET_LIBRARIES})
rp_project_options(rp_session)

# Compiled competition rules, selected by name from task packs (extensions/rules/<name>/*.cpp).
file(GLOB rp_rules_sources CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/extensions/rules/*.cpp
     ${PROJECT_SOURCE_DIR}/extensions/rules/*/*.cpp)
add_library(rp_rules ${rp_rules_sources} ${PROJECT_SOURCE_DIR}/extensions/rules/registry.hpp)
target_include_directories(rp_rules PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/extensions>)
target_link_libraries(rp_rules PUBLIC rp_session)
rp_project_options(rp_rules)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  # Resolved Talos/UWRT scenario for tests, written by the pack tool (asset paths are absolute).
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)
  set(rp_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_test(NAME session_resolve_talos
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${PROJECT_SOURCE_DIR}/python/src
      ${Python_EXECUTABLE} -m robotics_platform.packs resolve
      ${PROJECT_SOURCE_DIR}/content/packs/scenarios/talos_uwrt -o ${rp_resolved_talos})
  set_tests_properties(session_resolve_talos PROPERTIES FIXTURES_SETUP session_resolved)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/session_fixtures)
  file(GLOB rp_session_tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp)
  foreach(test_source ${rp_session_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(session_${test_name} ${test_source})
    target_compile_definitions(session_${test_name} PRIVATE
      RP_SOURCE_DIR="${PROJECT_SOURCE_DIR}" RP_SESSION_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
      RP_RESOLVED_TALOS="${rp_resolved_talos}")
    target_link_libraries(session_${test_name} PRIVATE rp_rules GTest::gtest_main)
    rp_project_options(session_${test_name})
    gtest_discover_tests(session_${test_name} DISCOVERY_MODE PRE_TEST
      PROPERTIES FIXTURES_REQUIRED session_resolved)
  endforeach()
endif()
