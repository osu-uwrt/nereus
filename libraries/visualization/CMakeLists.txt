add_library(rp_visualization src/frames.cpp src/source.cpp src/live_source.cpp src/display.cpp src/camera.cpp src/animation.cpp)
add_library(RoboticsPlatform::visualization ALIAS rp_visualization)
set_target_properties(rp_visualization PROPERTIES EXPORT_NAME visualization)
target_include_directories(rp_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
find_package(Threads REQUIRED)
target_link_libraries(rp_visualization PUBLIC rp_spatial Threads::Threads)
rp_project_options(rp_visualization)

find_package(yaml-cpp REQUIRED CONFIG)
add_library(rp_viewer_io ../viewer_io/src/workspace.cpp ../viewer_io/src/session.cpp ../viewer_io/src/save.cpp)
add_library(RoboticsPlatform::viewer_io ALIAS rp_viewer_io)
set_target_properties(rp_viewer_io PROPERTIES EXPORT_NAME viewer_io)
target_include_directories(rp_viewer_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../viewer_io/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_viewer_io PUBLIC rp_visualization PRIVATE yaml-cpp)
rp_project_options(rp_viewer_io)
install(TARGETS rp_visualization rp_viewer_io EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ ../viewer_io/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/workspaces/ DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/workspaces)

if(RP_BUILD_VIEWER)
  find_package(glfw3 3.3 REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  add_library(rp_rendering ../rendering/src/lines.cpp)
  add_library(RoboticsPlatform::rendering ALIAS rp_rendering)
  set_target_properties(rp_rendering PROPERTIES EXPORT_NAME rendering)
  target_include_directories(rp_rendering PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../rendering/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(rp_rendering PUBLIC rp_visualization PRIVATE OpenGL::GL GLEW::GLEW)
  rp_project_options(rp_rendering)
  install(TARGETS rp_rendering EXPORT RoboticsPlatformTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(FILES ../rendering/include/robotics/rendering/lines.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
  set(imgui ${PROJECT_SOURCE_DIR}/third_party/imgui)
  add_library(rp_imgui STATIC ${imgui}/imgui.cpp ${imgui}/imgui_draw.cpp
    ${imgui}/imgui_tables.cpp ${imgui}/imgui_widgets.cpp
    ${imgui}/backends/imgui_impl_glfw.cpp ${imgui}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(rp_imgui SYSTEM PUBLIC ${imgui})
  target_link_libraries(rp_imgui PRIVATE glfw ${CMAKE_DL_LIBS})
  add_library(rp_viewer_desktop STATIC ${PROJECT_SOURCE_DIR}/applications/viewer/desktop.cpp
    ${PROJECT_SOURCE_DIR}/applications/viewer/interface.cpp)
  target_include_directories(rp_viewer_desktop PUBLIC ${PROJECT_SOURCE_DIR}/applications/viewer)
  target_link_libraries(rp_viewer_desktop PUBLIC rp_viewer_io rp_rendering
    PRIVATE rp_imgui glfw OpenGL::GL GLEW::GLEW)
  if(RP_BUILD_SCENE_RENDERER)
    target_sources(rp_viewer_desktop PRIVATE ${PROJECT_SOURCE_DIR}/applications/viewer/scene_view.cpp)
    target_link_libraries(rp_viewer_desktop PRIVATE rp_scene_view)
    target_compile_definitions(rp_viewer_desktop PUBLIC RP_VIEWER_SCENES=1)
    if(BUILD_TESTING)
      add_executable(scene_viewport_contract ${PROJECT_SOURCE_DIR}/applications/viewer/scene_contract.cpp)
      target_link_libraries(scene_viewport_contract PRIVATE rp_viewer_desktop rp_scene_rendering OpenGL::GL GLEW::GLEW)
      set_target_properties(scene_viewport_contract PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
      rp_project_options(scene_viewport_contract)
    endif()
  endif()
  rp_project_options(rp_viewer_desktop)
  add_executable(robotics-viewer ${PROJECT_SOURCE_DIR}/applications/viewer/main.cpp)
  target_link_libraries(robotics-viewer PRIVATE rp_viewer_desktop)
  set_target_properties(robotics-viewer PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  rp_project_options(robotics-viewer)
  install(TARGETS robotics-viewer RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(FILES ${imgui}/LICENSE.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/legal RENAME imgui-MIT.txt)
endif()
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(visualization_tests tests/visualization_test.cpp tests/live_source_test.cpp tests/animation_test.cpp)
  target_link_libraries(visualization_tests PRIVATE rp_viewer_io GTest::gtest_main)
  target_compile_definitions(visualization_tests PRIVATE RP_WORKSPACES="${PROJECT_SOURCE_DIR}/workspaces"
    RP_ANIMATION_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")
  rp_project_options(visualization_tests)
  gtest_discover_tests(visualization_tests DISCOVERY_MODE PRE_TEST)
endif()
