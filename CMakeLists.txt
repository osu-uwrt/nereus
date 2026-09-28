cmake_minimum_required(VERSION 3.22)
project(RoboticsPlatform VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)
option(RP_BUILD_SIMULATION "Build simulation libraries" ON)
option(RP_BUILD_VISUALIZATION "Build neutral visualization and viewer IO libraries" OFF)
option(RP_BUILD_SIMULATION_VIEW "Build read-only simulation visualization adapter" OFF)
option(RP_BUILD_VIEWER "Build the standalone desktop viewer" OFF)
option(RP_BUILD_CLI "Build the YAML scenario loader and standalone runner" ON)
option(RP_BUILD_PYTHON "Build the optional Python extension and package" OFF)
if(RP_BUILD_PYTHON)
  if(BUILD_SHARED_LIBS)
    message(FATAL_ERROR "The Python package currently requires BUILD_SHARED_LIBS=OFF")
  endif()
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()
option(RP_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

if(NOT RP_BUILD_SIMULATION AND (RP_BUILD_CLI OR RP_BUILD_PYTHON))
  message(FATAL_ERROR "CLI/Python require RP_BUILD_SIMULATION; disable them for viewer-only builds")
endif()
set(RP_NEEDS_YAML OFF)
if(RP_BUILD_CLI OR RP_BUILD_PYTHON OR RP_BUILD_VISUALIZATION OR RP_BUILD_VIEWER OR RP_BUILD_SIMULATION_VIEW)
  set(RP_NEEDS_YAML ON)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

function(rp_project_options target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    if(RP_ENABLE_SANITIZERS)
      target_compile_options(${target} PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
      target_link_options(${target} PUBLIC -fsanitize=address,undefined)
    endif()
  endif()
endfunction()

if(RP_BUILD_SIMULATION)
add_library(rp_simulation
  libraries/simulation/src/marine_dynamics.cpp
  libraries/simulation/src/thruster_dynamics.cpp
  libraries/simulation/src/plant.cpp)
add_library(RoboticsPlatform::simulation ALIAS rp_simulation)
set_target_properties(rp_simulation PROPERTIES EXPORT_NAME simulation)
target_include_directories(rp_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libraries/simulation/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE libraries/simulation/src)
target_link_libraries(rp_simulation PUBLIC Eigen3::Eigen)
rp_project_options(rp_simulation)

add_library(rp_sensors libraries/sensors/src/runtime.cpp libraries/sensors/src/models.cpp)
add_library(RoboticsPlatform::sensors ALIAS rp_sensors)
set_target_properties(rp_sensors PROPERTIES EXPORT_NAME sensors)
target_include_directories(rp_sensors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libraries/sensors/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_sensors PUBLIC rp_simulation)
rp_project_options(rp_sensors)

if(RP_BUILD_CLI OR RP_BUILD_PYTHON)
  find_package(yaml-cpp REQUIRED CONFIG)
  add_library(rp_config libraries/config/src/scenario.cpp libraries/config/src/sensor_profiles.cpp)
  add_library(RoboticsPlatform::config ALIAS rp_config)
  set_target_properties(rp_config PROPERTIES EXPORT_NAME config)
  target_include_directories(rp_config PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libraries/config/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(rp_config PUBLIC rp_sensors PRIVATE yaml-cpp)
  rp_project_options(rp_config)
  if(RP_BUILD_CLI)
    add_executable(robotics-sim applications/simulator/main.cpp applications/simulator/telemetry.cpp)
    target_link_libraries(robotics-sim PRIVATE rp_config)
    rp_project_options(robotics-sim)
    install(TARGETS robotics-sim RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
  install(TARGETS rp_config EXPORT RoboticsPlatformTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY libraries/config/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

if(RP_BUILD_PYTHON)
  find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 2.13 REQUIRED CONFIG)
  pybind11_add_module(_native NO_EXTRAS bindings/python/module.cpp
    bindings/python/plant.cpp bindings/python/sensors.cpp bindings/python/runtime.cpp)
  target_link_libraries(_native PRIVATE rp_config)
  rp_project_options(_native)
  install(TARGETS _native LIBRARY DESTINATION robotics_platform COMPONENT python)
  install(DIRECTORY python/src/robotics_platform/ DESTINATION robotics_platform
    COMPONENT python PATTERN "__pycache__" EXCLUDE)
  install(DIRECTORY content/ DESTINATION robotics_platform/content COMPONENT python)
  install(FILES LICENSE.md docs/PROVENANCE.md DESTINATION robotics_platform/legal COMPONENT python)
endif()

install(TARGETS rp_simulation rp_sensors EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY libraries/simulation/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY libraries/sensors/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY content/ DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform)
endif()

if(RP_BUILD_VISUALIZATION OR RP_BUILD_VIEWER OR RP_BUILD_SIMULATION_VIEW)
  add_subdirectory(libraries/visualization)
endif()

if(RP_BUILD_SIMULATION_VIEW)
  if(NOT RP_BUILD_SIMULATION)
    message(FATAL_ERROR "RP_BUILD_SIMULATION_VIEW requires RP_BUILD_SIMULATION")
  endif()
  add_subdirectory(integrations/simulation_view)
endif()

install(FILES LICENSE.md docs/PROVENANCE.md DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/legal)
install(EXPORT RoboticsPlatformTargets NAMESPACE RoboticsPlatform::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboticsPlatform)
configure_package_config_file(cmake/RoboticsPlatformConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RoboticsPlatformConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboticsPlatform)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/RoboticsPlatformConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/RoboticsPlatformConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/RoboticsPlatformConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoboticsPlatform)

if(BUILD_TESTING AND RP_BUILD_SIMULATION)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(plant_tests libraries/simulation/tests/plant_test.cpp
    libraries/simulation/tests/model_reference_test.cpp)
  target_include_directories(plant_tests PRIVATE libraries/simulation/src)
  target_link_libraries(plant_tests PRIVATE rp_simulation GTest::gtest_main)
  rp_project_options(plant_tests)
  gtest_discover_tests(plant_tests DISCOVERY_MODE PRE_TEST)
  add_executable(sensor_tests libraries/sensors/tests/sensors_test.cpp)
  target_link_libraries(sensor_tests PRIVATE rp_sensors GTest::gtest_main)
  rp_project_options(sensor_tests)
  gtest_discover_tests(sensor_tests DISCOVERY_MODE PRE_TEST)
  if(RP_BUILD_CLI)
    add_executable(config_tests tests/config_test.cpp)
    target_compile_definitions(config_tests PRIVATE RP_TEST_CONTENT="${CMAKE_CURRENT_SOURCE_DIR}/content")
    target_link_libraries(config_tests PRIVATE rp_config GTest::gtest_main)
    rp_project_options(config_tests)
    gtest_discover_tests(config_tests DISCOVERY_MODE PRE_TEST)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME profile_cli_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/profile_cli_test.py
      $<TARGET_FILE:robotics-sim> ${CMAKE_CURRENT_SOURCE_DIR}/content)
    add_test(NAME cli_contract COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_test.py
      $<TARGET_FILE:robotics-sim> ${CMAKE_CURRENT_SOURCE_DIR}/content/examples/empty_pool.yaml)
  endif()
endif()
