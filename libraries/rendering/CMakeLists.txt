# CPU assets are independently usable without visualization, OpenGL or simulation.
find_package(assimp 5 REQUIRED CONFIG)
add_library(rp_mesh_assets src/assets.cpp)
add_library(RoboticsPlatform::mesh_assets ALIAS rp_mesh_assets)
set_target_properties(rp_mesh_assets PROPERTIES EXPORT_NAME mesh_assets)
target_include_directories(rp_mesh_assets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(rp_mesh_assets PUBLIC Eigen3::Eigen PRIVATE assimp::assimp)
rp_project_options(rp_mesh_assets)
install(TARGETS rp_mesh_assets EXPORT RoboticsPlatformTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/robotics/rendering/assets.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(asset_tests tests/assets_test.cpp)
  target_link_libraries(asset_tests PRIVATE rp_mesh_assets GTest::gtest_main)
  target_compile_definitions(asset_tests PRIVATE
    RP_ASSET_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures"
    RP_VISUAL_CONTENT="${PROJECT_SOURCE_DIR}/content/visuals")
  rp_project_options(asset_tests)
  gtest_discover_tests(asset_tests DISCOVERY_MODE PRE_TEST)
endif()

if(RP_BUILD_SCENE_RENDERER)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(PNG REQUIRED)
  add_library(rp_scene_rendering src/scene.cpp src/renderer.cpp)
  add_library(RoboticsPlatform::scene_rendering ALIAS rp_scene_rendering)
  set_target_properties(rp_scene_rendering PROPERTIES EXPORT_NAME scene_rendering)
  target_link_libraries(rp_scene_rendering PUBLIC rp_mesh_assets PRIVATE OpenGL::GL GLEW::GLEW PNG::PNG)
  target_include_directories(rp_scene_rendering SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/glm)
  rp_project_options(rp_scene_rendering)
  install(TARGETS rp_scene_rendering EXPORT RoboticsPlatformTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(FILES include/robotics/rendering/scene.hpp include/robotics/rendering/renderer.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
  install(DIRECTORY shaders/ DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/shaders)
  install(FILES ${PROJECT_SOURCE_DIR}/third_party/glm/glm/copying.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/robotics_platform/legal RENAME glm-license.txt)
  if(RP_BUILD_OFFSCREEN_RENDERER)
    find_package(OpenGL REQUIRED COMPONENTS EGL)
    find_package(Threads REQUIRED)
    add_library(rp_offscreen_rendering src/offscreen.cpp)
    add_library(RoboticsPlatform::offscreen_rendering ALIAS rp_offscreen_rendering)
    set_target_properties(rp_offscreen_rendering PROPERTIES EXPORT_NAME offscreen_rendering)
    target_link_libraries(rp_offscreen_rendering PUBLIC rp_scene_rendering
      PRIVATE OpenGL::EGL GLEW::GLEW Threads::Threads)
    rp_project_options(rp_offscreen_rendering)
    install(TARGETS rp_offscreen_rendering EXPORT RoboticsPlatformTargets
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES include/robotics/rendering/offscreen.hpp
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robotics/rendering)
    if(BUILD_TESTING)
      add_executable(offscreen_tests tests/offscreen_test.cpp)
      target_link_libraries(offscreen_tests PRIVATE rp_offscreen_rendering GTest::gtest_main
        OpenGL::EGL Threads::Threads)
      target_compile_definitions(offscreen_tests PRIVATE
        RP_RENDERING_SHADERS="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
      rp_project_options(offscreen_tests)
      gtest_discover_tests(offscreen_tests DISCOVERY_MODE PRE_TEST)
    endif()
  endif()
  if(BUILD_TESTING)
    find_package(glfw3 REQUIRED CONFIG)
    add_executable(renderer_image_tests tests/renderer_image_test.cpp)
    target_link_libraries(renderer_image_tests PRIVATE rp_scene_rendering glfw
      GLEW::GLEW OpenGL::GL PNG::PNG GTest::gtest_main)
    target_compile_definitions(renderer_image_tests PRIVATE
      RP_RENDERING_SHADERS="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
      RP_PACK_CONTENT="${PROJECT_SOURCE_DIR}/content/packs")
    rp_project_options(renderer_image_tests)
    gtest_discover_tests(renderer_image_tests DISCOVERY_MODE PRE_TEST)
  endif()
endif()
