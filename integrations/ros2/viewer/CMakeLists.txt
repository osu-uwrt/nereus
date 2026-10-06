# ROS pool viewer: operator panels and live pool view over the simulator bridge and robot stack.
# Optional outer integration; nothing in libraries/ depends on it.

# ROS 2 dependencies. The robot-specific message packages are optional: the UWRT providers are built only
# when riptide_msgs2 is found (or NEREUS_ROS_VIEWER_UWRT is forced).
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(riptide_msgs2 QUIET)
find_package(chameleon_tf_msgs QUIET)
find_package(zed_msgs QUIET)
option(NEREUS_ROS_VIEWER_UWRT "Build the UWRT (riptide_msgs2) panel providers" ${riptide_msgs2_FOUND})
if(NEREUS_ROS_VIEWER_UWRT)
  find_package(diagnostic_msgs REQUIRED) # telemetry readings
endif()

# Windowing and OpenGL for the viewer host.
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Vendored Dear ImGui (GLFW + OpenGL 3 backends) as a static library; SYSTEM includes keep -Werror off its
# headers.
set(imgui ${PROJECT_SOURCE_DIR}/third_party/imgui)
add_library(nereus_imgui STATIC ${imgui}/imgui.cpp ${imgui}/imgui_draw.cpp
  ${imgui}/imgui_tables.cpp ${imgui}/imgui_widgets.cpp
  ${imgui}/backends/imgui_impl_glfw.cpp ${imgui}/backends/imgui_impl_opengl3.cpp)
target_include_directories(nereus_imgui SYSTEM PUBLIC ${imgui})
target_link_libraries(nereus_imgui PRIVATE glfw ${CMAKE_DL_LIBS})
install(FILES ${imgui}/LICENSE.txt DESTINATION ${CMAKE_INSTALL_DATADIR}/nereus/legal RENAME imgui-MIT.txt)

# Vendored header-only GLM.
add_library(nereus_glm INTERFACE)
target_include_directories(nereus_glm SYSTEM INTERFACE ${PROJECT_SOURCE_DIR}/third_party/glm)

# Panels library (composition, providers, panels, overlays): panels/CMakeLists.txt.
add_subdirectory(panels)

# Viewer host application (window, scene, overlays, camera cards): src/.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/CMakeLists.txt)
  add_subdirectory(src)
endif()
