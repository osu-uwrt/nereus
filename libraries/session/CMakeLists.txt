# nereus_session: the C++ simulation runtime over resolved packs (plant, sensors, mechanisms,
# payloads, prop worlds, task observers, run control). Sources are globbed per component folder
# so components can be developed independently: src/*.cpp, src/tasks/*.cpp.
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Bullet REQUIRED)
file(GLOB nereus_session_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/*.cpp)
add_library(nereus_session ${nereus_session_sources})
add_library(Nereus::session ALIAS nereus_session)
set_target_properties(nereus_session PROPERTIES EXPORT_NAME session)
target_include_directories(nereus_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(nereus_session SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})
# PyBullet is a double-precision Bullet build; the prop world matches it (Debian ships both).
target_compile_definitions(nereus_session PRIVATE BT_USE_DOUBLE_PRECISION)
target_link_libraries(nereus_session PUBLIC nereus_sensors nereus_spatial nlohmann_json::nlohmann_json
                      PRIVATE BulletDynamics-float64 BulletCollision-float64 LinearMath-float64)
nereus_project_options(nereus_session)

# Compiled competition rules, selected by name from task packs (extensions/rules/<name>/*.cpp).
file(GLOB nereus_rules_sources CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/extensions/rules/*.cpp
     ${PROJECT_SOURCE_DIR}/extensions/rules/*/*.cpp)
add_library(nereus_rules ${nereus_rules_sources} ${PROJECT_SOURCE_DIR}/extensions/rules/registry.hpp)
target_include_directories(nereus_rules PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/extensions>)
target_link_libraries(nereus_rules PUBLIC nereus_session)
nereus_project_options(nereus_rules)

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  # Resolved Talos/UWRT scenario for tests, written by the pack tool (asset paths are absolute).
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter)
  set(nereus_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  add_test(NAME session_resolve_talos
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${PROJECT_SOURCE_DIR}/python/src
      ${Python_EXECUTABLE} -m nereus.packs resolve
      ${PROJECT_SOURCE_DIR}/content/packs/scenarios/talos_uwrt -o ${nereus_resolved_talos})
  set_tests_properties(session_resolve_talos PROPERTIES FIXTURES_SETUP session_resolved)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/session_fixtures)
  file(GLOB nereus_session_tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp)
  foreach(test_source ${nereus_session_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(session_${test_name} ${test_source})
    target_compile_definitions(session_${test_name} PRIVATE
      NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}" NEREUS_SESSION_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
      NEREUS_RESOLVED_TALOS="${nereus_resolved_talos}")
    target_link_libraries(session_${test_name} PRIVATE nereus_rules GTest::gtest_main)
    nereus_project_options(session_${test_name})
    gtest_discover_tests(session_${test_name} DISCOVERY_MODE PRE_TEST
      PROPERTIES FIXTURES_REQUIRED session_resolved)
  endforeach()
endif()
