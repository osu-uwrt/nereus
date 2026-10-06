# nereus_session: the C++ simulation runtime over resolved packs (plant, sensors, mechanisms,
# payloads, prop worlds, task observers, run control). Sources are globbed per component folder
# so components can be developed independently: src/*.cpp, src/tasks/*.cpp.
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Bullet REQUIRED)

# The session library itself.
file(GLOB nereus_session_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/*.cpp)
add_library(nereus_session ${nereus_session_sources})
add_library(Nereus::session ALIAS nereus_session)
set_target_properties(nereus_session PROPERTIES EXPORT_NAME session)

# Public headers live in include/; src/ holds private headers shared between components.
target_include_directories(nereus_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(nereus_session SYSTEM PRIVATE ${BULLET_INCLUDE_DIRS})

# The prop world needs the double-precision Bullet build (Debian ships both).
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

  # Frozen resolved Talos/UWRT scenarios (tests/fixtures/*_resolved.json.in), not the live packs: the recorded
  # references pin the plant to the last bit, and the team retunes the Talos pack. configure_file fills in the
  # absolute asset paths. Re-freeze with tools/freeze_session_scenarios.py only when re-recording the references.
  set(NEREUS_SOURCE_DIR ${PROJECT_SOURCE_DIR})
  set(nereus_resolved_talos ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/talos_uwrt_resolved.json.in ${nereus_resolved_talos} @ONLY)
  # The same robot in the RPAC dive well: a sloped (profiled) floor.
  set(nereus_resolved_rpac ${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_rpac_resolved.json)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/talos_uwrt_rpac_resolved.json.in ${nereus_resolved_rpac}
    @ONLY)

  # One gtest executable per tests/*_test.cpp, named session_<file stem>.
  file(GLOB nereus_session_tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*_test.cpp)
  foreach(test_source ${nereus_session_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(session_${test_name} ${test_source})
    target_compile_definitions(session_${test_name} PRIVATE
      NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}" NEREUS_SESSION_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures"
      NEREUS_RESOLVED_TALOS="${nereus_resolved_talos}" NEREUS_RESOLVED_RPAC="${nereus_resolved_rpac}")
    target_link_libraries(session_${test_name} PRIVATE nereus_rules GTest::gtest_main)
    nereus_project_options(session_${test_name})
    gtest_discover_tests(session_${test_name} DISCOVERY_MODE PRE_TEST)
  endforeach()
endif()
