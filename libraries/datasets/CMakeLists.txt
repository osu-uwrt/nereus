# nereus_datasets: headless labeled-dataset rendering from a planner job (nereus.dataset_job.v1): per-task robot
# pose sampling, label-pass acceptance, randomized RGB through the robot pack's camera model, 16-bit id maps and
# per-sample records. No ROS, no physics, no window. The executable is nereus-dataset-render.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)

# Library: job parsing, sampling, part resolution, outputs and the generator.
add_library(nereus_datasets src/job.cpp src/sampling.cpp src/environment.cpp src/mesh_parts.cpp src/output.cpp
  src/generator.cpp)
add_library(Nereus::datasets ALIAS nereus_datasets)
set_target_properties(nereus_datasets PROPERTIES EXPORT_NAME datasets)
target_include_directories(nereus_datasets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(nereus_datasets SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nereus_datasets PUBLIC nereus_pack_scene nereus_cameras nereus_offscreen_rendering nereus_session
                      PRIVATE ${OpenCV_LIBS})
# Fallback shader directory when neither --shaders nor $NEREUS_SHADER_DIR is given.
target_compile_definitions(nereus_datasets PRIVATE
  NEREUS_RENDERING_SHADERS="${PROJECT_SOURCE_DIR}/libraries/rendering/shaders")
nereus_project_options(nereus_datasets)

# Command-line renderer (one shard of a job per process).
add_executable(nereus-dataset-render src/main.cpp)
target_link_libraries(nereus-dataset-render PRIVATE nereus_datasets)
nereus_project_options(nereus-dataset-render)
install(TARGETS nereus-dataset-render RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  # End-to-end tests read the resolved Talos fixture from the build tree and task assets from the source tree.
  add_executable(datasets_tests tests/datasets_test.cpp)
  target_link_libraries(datasets_tests PRIVATE nereus_datasets GTest::gtest_main ${OpenCV_LIBS})
  target_include_directories(datasets_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_compile_definitions(datasets_tests PRIVATE
    NEREUS_RESOLVED_TALOS="${CMAKE_BINARY_DIR}/session_fixtures/talos_uwrt_resolved.json"
    NEREUS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  nereus_project_options(datasets_tests)
  gtest_discover_tests(datasets_tests DISCOVERY_MODE PRE_TEST)
endif()
