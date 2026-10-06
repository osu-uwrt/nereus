# CPU assets are independently usable without OpenGL or simulation.
find_package(assimp 5 REQUIRED CONFIG)

# nereus_mesh_assets: Assimp mesh loading (assets.cpp); Assimp stays private.
add_library(nereus_mesh_assets src/assets.cpp)
add_library(Nereus::mesh_assets ALIAS nereus_mesh_assets)
set_target_properties(nereus_mesh_assets PROPERTIES EXPORT_NAME mesh_assets)
target_include_directories(nereus_mesh_assets PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(nereus_mesh_assets PUBLIC Eigen3::Eigen PRIVATE assimp::assimp)
nereus_project_options(nereus_mesh_assets)

# Install the library and only its own header (the GL headers belong to the optional targets).
install(TARGETS nereus_mesh_assets EXPORT NereusTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/nereus/rendering/assets.hpp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/nereus/rendering)

# Asset tests read fixture meshes and the shipped content packs from the source tree.
if(BUILD_TESTING)
  find_package(GTest REQUIRED CONFIG)
  include(GoogleTest)
  add_executable(asset_tests tests/assets_test.cpp)
  target_link_libraries(asset_tests PRIVATE nereus_mesh_assets GTest::gtest_main)
  target_compile_definitions(asset_tests PRIVATE
    NEREUS_ASSET_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures"
    NEREUS_PACK_CONTENT="${PROJECT_SOURCE_DIR}/content/packs")
  nereus_project_options(asset_tests)
  gtest_discover_tests(asset_tests DISCOVERY_MODE PRE_TEST)
endif()

# Optional OpenGL scene renderer (scene.cpp, renderer.cpp); needs GLEW, libpng and the vendored glm.
if(NEREUS_BUILD_SCENE_RENDERER)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)
  find_package(PNG REQUIRED)
  add_library(nereus_scene_rendering src/scene.cpp src/renderer.cpp)
  add_library(Nereus::scene_rendering ALIAS nereus_scene_rendering)
  set_target_properties(nereus_scene_rendering PROPERTIES EXPORT_NAME scene_rendering)
  target_link_libraries(nereus_scene_rendering PUBLIC nereus_mesh_assets PRIVATE OpenGL::GL GLEW::GLEW PNG::PNG)
  target_include_directories(nereus_scene_rendering SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/third_party/glm)
  nereus_project_options(nereus_scene_rendering)
  install(TARGETS nereus_scene_rendering EXPORT NereusTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(FILES include/nereus/rendering/scene.hpp include/nereus/rendering/renderer.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/nereus/rendering)
  # Shaders are loaded at runtime from a directory the caller passes in, so they install as data.
  install(DIRECTORY shaders/ DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/shaders)
  install(FILES ${PROJECT_SOURCE_DIR}/third_party/glm/glm/copying.txt
    DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/legal RENAME glm-license.txt)

  # Optional headless EGL host for the renderer (offscreen.cpp).
  if(NEREUS_BUILD_OFFSCREEN_RENDERER)
    find_package(OpenGL REQUIRED COMPONENTS EGL)
    find_package(Threads REQUIRED)
    add_library(nereus_offscreen_rendering src/offscreen.cpp)
    add_library(Nereus::offscreen_rendering ALIAS nereus_offscreen_rendering)
    set_target_properties(nereus_offscreen_rendering PROPERTIES EXPORT_NAME offscreen_rendering)
    target_link_libraries(nereus_offscreen_rendering PUBLIC nereus_scene_rendering
      PRIVATE OpenGL::EGL GLEW::GLEW Threads::Threads)
    nereus_project_options(nereus_offscreen_rendering)
    install(TARGETS nereus_offscreen_rendering EXPORT NereusTargets
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
    install(FILES include/nereus/rendering/offscreen.hpp
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/nereus/rendering)
    if(BUILD_TESTING)
      add_executable(offscreen_tests tests/offscreen_test.cpp)
      target_link_libraries(offscreen_tests PRIVATE nereus_offscreen_rendering GTest::gtest_main
        OpenGL::EGL Threads::Threads PNG::PNG)
      target_compile_definitions(offscreen_tests PRIVATE
        NEREUS_RENDERING_SHADERS="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
      nereus_project_options(offscreen_tests)
      gtest_discover_tests(offscreen_tests DISCOVERY_MODE PRE_TEST)
    endif()
  endif()

  # Image tests render through a GLFW window context, using the shaders straight from the source tree.
  if(BUILD_TESTING)
    find_package(glfw3 REQUIRED CONFIG)
    add_executable(renderer_image_tests tests/renderer_image_test.cpp)
    target_link_libraries(renderer_image_tests PRIVATE nereus_scene_rendering glfw
      GLEW::GLEW OpenGL::GL PNG::PNG GTest::gtest_main)
    target_compile_definitions(renderer_image_tests PRIVATE
      NEREUS_RENDERING_SHADERS="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
      NEREUS_PACK_CONTENT="${PROJECT_SOURCE_DIR}/content/packs")
    nereus_project_options(renderer_image_tests)
    gtest_discover_tests(renderer_image_tests DISCOVERY_MODE PRE_TEST)
  endif()
endif()
