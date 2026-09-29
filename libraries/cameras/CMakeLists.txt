# Optional camera geometry and owned RGB/depth processing, usable without GL or simulation.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
add_library(rp_cameras src/camera.cpp src/depth_noise.cpp)
add_library(RoboticsPlatform::cameras ALIAS rp_cameras)
set_target_properties(rp_cameras PROPERTIES EXPORT_NAME cameras)
target_include_directories(rp_cameras PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(rp_cameras SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rp_cameras PUBLIC rp_spatial PRIVATE ${OpenCV_LIBS})
rp_project_options(rp_cameras)
install(TARGETS rp_cameras EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(camera_tests tests/camera_test.cpp)
  target_link_libraries(camera_tests PRIVATE rp_cameras GTest::gtest_main ${OpenCV_LIBS})
  target_include_directories(camera_tests SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
  rp_project_options(camera_tests)
  gtest_discover_tests(camera_tests DISCOVERY_MODE PRE_TEST)
endif()
