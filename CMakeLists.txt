# Nereus top-level build: the always-on simulation/sensors libraries plus optional components (renderer,
# session runtime, cameras, datasets, ROS 2 bridge and viewer) switched on by the options below.
cmake_minimum_required(VERSION 3.22)
project(Nereus VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CTest)

# Optional components. CMakePresets.json turns on the combinations that make sense together; the checks
# further down reject inconsistent ones.
option(NEREUS_BUILD_SCENE_RENDERER "Build context-owned OpenGL scene rendering" OFF)
option(NEREUS_BUILD_OFFSCREEN_RENDERER "Build the optional display-free EGL capture host" OFF)
option(NEREUS_BUILD_MESH_ASSETS "Build neutral CPU mesh loading" OFF)
option(NEREUS_BUILD_CAMERAS "Build optional calibrated camera image processing" OFF)
option(NEREUS_BUILD_SIMULATION "Build simulation libraries" ON)
option(NEREUS_BUILD_SESSION "Build the C++ scenario runtime (needs nlohmann-json, Bullet)" OFF)
option(NEREUS_BUILD_PACK_SCENE "Build renderer scene composition from resolved scenarios" OFF)
option(NEREUS_BUILD_SESSION_CAMERAS "Build the scheduled, on-demand simulator camera acquisition" OFF)
option(NEREUS_BUILD_DATASETS "Build the headless labeled-dataset renderer (nereus-dataset-render)" OFF)
option(NEREUS_BUILD_ROS_BRIDGE "Build the rclcpp simulator bridge (needs a sourced ROS 2 workspace)" OFF)
option(NEREUS_BUILD_ROS_VIEWER "Build the ROS pool viewer (needs a sourced ROS 2 workspace)" OFF)
if(NEREUS_BUILD_OFFSCREEN_RENDERER AND NOT NEREUS_BUILD_SCENE_RENDERER)
  message(FATAL_ERROR "NEREUS_BUILD_OFFSCREEN_RENDERER requires NEREUS_BUILD_SCENE_RENDERER")
endif()
option(NEREUS_ENABLE_SANITIZERS "Enable address and undefined-behavior sanitizers" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Common settings for every Nereus target: C++17 without extensions, warnings as errors on GCC/Clang, and
# ASan/UBSan when NEREUS_ENABLE_SANITIZERS is on (PUBLIC so dependents link the runtime too).
function(nereus_project_options target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    if(NEREUS_ENABLE_SANITIZERS)
      target_compile_options(${target} PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
      target_link_options(${target} PUBLIC -fsanitize=address,undefined)
    endif()
  endif()
endfunction()

add_subdirectory(libraries/spatial)
if(NEREUS_BUILD_CAMERAS)
  add_subdirectory(libraries/cameras)
endif()

# Core plant (rigid-body marine dynamics, thrusters, contacts) and the sensor models built on it.
if(NEREUS_BUILD_SIMULATION)
add_library(nereus_simulation
  libraries/simulation/src/marine_dynamics.cpp
  libraries/simulation/src/thruster_dynamics.cpp
  libraries/simulation/src/box_contacts.cpp
  libraries/simulation/src/payload.cpp
  libraries/simulation/src/floor_profile.cpp
  libraries/simulation/src/plant.cpp)
add_library(Nereus::simulation ALIAS nereus_simulation)
set_target_properties(nereus_simulation PROPERTIES EXPORT_NAME simulation)
target_include_directories(nereus_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libraries/simulation/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE libraries/simulation/src)
target_link_libraries(nereus_simulation PUBLIC Eigen3::Eigen)
nereus_project_options(nereus_simulation)

add_library(nereus_sensors libraries/sensors/src/runtime.cpp libraries/sensors/src/models.cpp)
add_library(Nereus::sensors ALIAS nereus_sensors)
set_target_properties(nereus_sensors PROPERTIES EXPORT_NAME sensors)
target_include_directories(nereus_sensors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libraries/sensors/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_sensors PUBLIC nereus_simulation)
nereus_project_options(nereus_sensors)

install(TARGETS nereus_simulation nereus_sensors EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY libraries/simulation/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY libraries/sensors/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY content/ DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus)
endif()

# Optional components, each guarded by the components it depends on.
if(NEREUS_BUILD_MESH_ASSETS OR NEREUS_BUILD_SCENE_RENDERER)
  add_subdirectory(libraries/rendering)
endif()
if(NEREUS_BUILD_SESSION)
  if(NOT NEREUS_BUILD_SIMULATION)
    message(FATAL_ERROR "NEREUS_BUILD_SESSION requires NEREUS_BUILD_SIMULATION")
  endif()
  add_subdirectory(libraries/session)
endif()

if(NEREUS_BUILD_PACK_SCENE OR NEREUS_BUILD_SESSION_CAMERAS)
  if(NOT (NEREUS_BUILD_SESSION AND NEREUS_BUILD_SCENE_RENDERER AND NEREUS_BUILD_MESH_ASSETS))
    message(FATAL_ERROR "NEREUS_BUILD_PACK_SCENE/NEREUS_BUILD_SESSION_CAMERAS require NEREUS_BUILD_SESSION, "
                        "NEREUS_BUILD_SCENE_RENDERER and NEREUS_BUILD_MESH_ASSETS")
  endif()
  add_subdirectory(libraries/pack_scene)
endif()
if(NEREUS_BUILD_SESSION_CAMERAS)
  if(NOT (NEREUS_BUILD_CAMERAS AND NEREUS_BUILD_OFFSCREEN_RENDERER))
    message(FATAL_ERROR "NEREUS_BUILD_SESSION_CAMERAS requires NEREUS_BUILD_CAMERAS and NEREUS_BUILD_OFFSCREEN_RENDERER")
  endif()
  add_subdirectory(libraries/session_cameras)
endif()
if(NEREUS_BUILD_DATASETS)
  if(NOT (NEREUS_BUILD_SESSION AND NEREUS_BUILD_PACK_SCENE AND NEREUS_BUILD_OFFSCREEN_RENDERER AND NEREUS_BUILD_CAMERAS
          AND NEREUS_BUILD_MESH_ASSETS))
    message(FATAL_ERROR "NEREUS_BUILD_DATASETS requires NEREUS_BUILD_SESSION, NEREUS_BUILD_PACK_SCENE, "
                        "NEREUS_BUILD_OFFSCREEN_RENDERER, NEREUS_BUILD_CAMERAS and NEREUS_BUILD_MESH_ASSETS")
  endif()
  add_subdirectory(libraries/datasets)
endif()

# ROS 2 integrations (configure only inside a sourced ROS 2 workspace).
if(NEREUS_BUILD_ROS_BRIDGE)
  if(NOT NEREUS_BUILD_SESSION)
    message(FATAL_ERROR "NEREUS_BUILD_ROS_BRIDGE requires NEREUS_BUILD_SESSION")
  endif()
  add_subdirectory(integrations/ros2/bridge)
endif()

if(NEREUS_BUILD_ROS_VIEWER)
  if(NOT NEREUS_BUILD_SCENE_RENDERER OR NOT NEREUS_BUILD_MESH_ASSETS)
    message(FATAL_ERROR "NEREUS_BUILD_ROS_VIEWER requires NEREUS_BUILD_SCENE_RENDERER and NEREUS_BUILD_MESH_ASSETS")
  endif()
  if(NOT (NEREUS_BUILD_PACK_SCENE AND NEREUS_BUILD_SESSION_CAMERAS))
    message(FATAL_ERROR "NEREUS_BUILD_ROS_VIEWER requires NEREUS_BUILD_PACK_SCENE and NEREUS_BUILD_SESSION_CAMERAS")
  endif()
  add_subdirectory(integrations/ros2/viewer)
endif()

# Install the exported targets and a find_package(Nereus) config (cmake/NereusConfig.cmake.in).
install(FILES LICENSE NOTICE DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/legal)
install(EXPORT NereusTargets NAMESPACE Nereus::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Nereus)
configure_package_config_file(cmake/NereusConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/NereusConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Nereus)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/NereusConfigVersion.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/NereusConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/NereusConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Nereus)

# Unit tests for the simulation and sensor libraries; the other components register their own tests.
if(BUILD_TESTING AND NEREUS_BUILD_SIMULATION)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)

  # Plant, reference-model, contact and floor-profile tests (read the shared fixtures under tests/fixtures).
  add_executable(plant_tests libraries/simulation/tests/plant_test.cpp
    libraries/simulation/tests/model_reference_test.cpp libraries/simulation/tests/stage_reference_test.cpp
    libraries/simulation/tests/contact_reference_test.cpp libraries/simulation/tests/floor_profile_test.cpp)
  target_compile_definitions(plant_tests PRIVATE NEREUS_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")
  target_include_directories(plant_tests PRIVATE libraries/simulation/src)
  target_link_libraries(plant_tests PRIVATE nereus_simulation GTest::gtest_main)
  nereus_project_options(plant_tests)
  gtest_discover_tests(plant_tests DISCOVERY_MODE PRE_TEST)

  add_executable(payload_tests libraries/simulation/tests/payload_test.cpp)
  target_compile_definitions(payload_tests PRIVATE NEREUS_PAYLOAD_FIXTURE="${PROJECT_SOURCE_DIR}/libraries/simulation/tests/fixtures/payload_reference.csv")
  target_link_libraries(payload_tests PRIVATE nereus_simulation GTest::gtest_main)
  nereus_project_options(payload_tests)
  gtest_discover_tests(payload_tests DISCOVERY_MODE PRE_TEST)

  add_executable(sensor_tests libraries/sensors/tests/sensors_test.cpp)
  target_link_libraries(sensor_tests PRIVATE nereus_sensors GTest::gtest_main)
  nereus_project_options(sensor_tests)
  gtest_discover_tests(sensor_tests DISCOVERY_MODE PRE_TEST)
endif()
