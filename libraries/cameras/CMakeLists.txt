# Optional camera geometry and owned RGB/depth processing, usable without GL or simulation.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)

# Library target; OpenCV stays private so consumers only see Eigen/spatial types.
add_library(nereus_cameras src/camera.cpp src/depth_noise.cpp)
add_library(Nereus::cameras ALIAS nereus_cameras)
set_target_properties(nereus_cameras PROPERTIES EXPORT_NAME cameras)
target_include_directories(nereus_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(nereus_cameras SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(nereus_cameras PUBLIC nereus_spatial PRIVATE ${OpenCV_LIBS})
nereus_project_options(nereus_cameras)

# Install the library and its public headers into the Nereus export set.
install(TARGETS nereus_cameras EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Unit tests (the tests decode JPEGs, so they link OpenCV directly).
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(camera_tests tests/camera_test.cpp)
  target_link_libraries(camera_tests PRIVATE nereus_cameras GTest::gtest_main ${OpenCV_LIBS})
  target_include_directories(camera_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  nereus_project_options(camera_tests)
  gtest_discover_tests(camera_tests DISCOVERY_MODE PRE_TEST)
endif()
