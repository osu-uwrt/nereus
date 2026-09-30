add_library(nereus_visualization src/frames.cpp src/source.cpp src/live_source.cpp src/display.cpp src/camera.cpp src/animation.cpp)
add_library(Nereus::visualization ALIAS nereus_visualization)
set_target_properties(nereus_visualization PROPERTIES EXPORT_NAME visualization)
target_include_directories(nereus_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
find_package(Threads REQUIRED)
target_link_libraries(nereus_visualization PUBLIC nereus_spatial Threads::Threads)
nereus_project_options(nereus_visualization)

find_package(yaml-cpp REQUIRED CONFIG)
add_library(nereus_viewer_io ../viewer_io/src/workspace.cpp ../viewer_io/src/session.cpp ../viewer_io/src/save.cpp)
add_library(Nereus::viewer_io ALIAS nereus_viewer_io)
set_target_properties(nereus_viewer_io PROPERTIES EXPORT_NAME viewer_io)
target_include_directories(nereus_viewer_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../viewer_io/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_viewer_io PUBLIC nereus_visualization PRIVATE yaml-cpp)
nereus_project_options(nereus_viewer_io)
install(TARGETS nereus_visualization nereus_viewer_io EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ ../viewer_io/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/workspaces/ DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/workspaces)

if(NEREUS_BUILD_VIEWER)
  find_package(glfw3 3.3 REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  add_library(nereus_rendering ../rendering/src/lines.cpp)
  add_library(Nereus::rendering ALIAS nereus_rendering)
  set_target_properties(nereus_rendering PROPERTIES EXPORT_NAME rendering)
  target_include_directories(nereus_rendering PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../rendering/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(nereus_rendering PUBLIC nereus_visualization PRIVATE OpenGL::GL GLEW::GLEW)
  nereus_project_options(nereus_rendering)
  install(TARGETS nereus_rendering EXPORT NereusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(FILES ../rendering/include/robotics/rendering/lines.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
  set(imgui ${PROJECT_SOURCE_DIR}/third_party/imgui)
  add_library(nereus_imgui STATIC ${imgui}/imgui.cpp ${imgui}/imgui_draw.cpp
    ${imgui}/imgui_tables.cpp ${imgui}/imgui_widgets.cpp
    ${imgui}/backends/imgui_impl_glfw.cpp ${imgui}/backends/imgui_impl_opengl3.cpp)
  target_include_directories(nereus_imgui SYSTEM PUBLIC ${imgui})
  target_link_libraries(nereus_imgui PRIVATE glfw ${CMAKE_DL_LIBS})
  add_library(nereus_viewer_desktop STATIC ${PROJECT_SOURCE_DIR}/applications/viewer/desktop.cpp
    ${PROJECT_SOURCE_DIR}/applications/viewer/interface.cpp)
  target_include_directories(nereus_viewer_desktop PUBLIC ${PROJECT_SOURCE_DIR}/applications/viewer)
  target_link_libraries(nereus_viewer_desktop PUBLIC nereus_viewer_io nereus_rendering
    PRIVATE nereus_imgui glfw OpenGL::GL GLEW::GLEW)
  if(NEREUS_BUILD_SCENE_RENDERER)
    target_sources(nereus_viewer_desktop PRIVATE ${PROJECT_SOURCE_DIR}/applications/viewer/scene_view.cpp)
    target_link_libraries(nereus_viewer_desktop PRIVATE nereus_scene_view)
    target_compile_definitions(nereus_viewer_desktop PUBLIC NEREUS_VIEWER_SCENES=1)
    if(BUILD_TESTING)
      add_executable(scene_viewport_contract ${PROJECT_SOURCE_DIR}/applications/viewer/scene_contract.cpp)
      target_link_libraries(scene_viewport_contract PRIVATE nereus_viewer_desktop nereus_scene_rendering OpenGL::GL GLEW::GLEW)
      set_target_properties(scene_viewport_contract PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
      nereus_project_options(scene_viewport_contract)
    endif()
  endif()
  nereus_project_options(nereus_viewer_desktop)
  add_executable(nereus-desktop ${PROJECT_SOURCE_DIR}/applications/viewer/main.cpp)
  target_link_libraries(nereus-desktop PRIVATE nereus_viewer_desktop)
  set_target_properties(nereus-desktop PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
  nereus_project_options(nereus-desktop)
  install(TARGETS nereus-desktop RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(FILES ${imgui}/LICENSE.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/legal RENAME imgui-MIT.txt)
endif()
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(visualization_tests tests/visualization_test.cpp tests/live_source_test.cpp tests/animation_test.cpp)
  target_link_libraries(visualization_tests PRIVATE nereus_viewer_io GTest::gtest_main)
  target_compile_definitions(visualization_tests PRIVATE NEREUS_WORKSPACES="${PROJECT_SOURCE_DIR}/workspaces"
    NEREUS_ANIMATION_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")
  nereus_project_options(visualization_tests)
  gtest_discover_tests(visualization_tests DISCOVERY_MODE PRE_TEST)
endif()
