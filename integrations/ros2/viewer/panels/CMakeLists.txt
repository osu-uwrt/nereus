# Target nereus_viewer_panels: panel composition, providers and panels of the pool viewer.
# Depends on ROS messages, ImGui, glm and yaml-cpp only; no simulator types.
set(_panel_sources
  panel_registry.cpp composition.cpp motion_panel.cpp mapping_panel.cpp actuator_panel.cpp
  run_panel.cpp simulation_panel.cpp autonomy_panel.cpp pose_gizmo.cpp
  ros_runtime.cpp standard_motion.cpp sim_run.cpp simulation_rate.cpp)
if(NEREUS_ROS_VIEWER_UWRT)
  list(APPEND _panel_sources uwrt_motion.cpp uwrt_autonomy.cpp uwrt_mapping.cpp uwrt_actuators.cpp)
endif()

add_library(nereus_viewer_panels STATIC ${_panel_sources})
target_compile_features(nereus_viewer_panels PUBLIC cxx_std_17)
target_include_directories(nereus_viewer_panels
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(nereus_viewer_panels PUBLIC nereus_imgui nereus_glm yaml-cpp
  rclcpp::rclcpp rclcpp_action::rclcpp_action tf2_ros::tf2_ros
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${visualization_msgs_TARGETS} ${action_msgs_TARGETS} ${rcl_interfaces_TARGETS})
if(NEREUS_ROS_VIEWER_UWRT)
  target_compile_definitions(nereus_viewer_panels PUBLIC NEREUS_VIEWER_UWRT=1)
  target_link_libraries(nereus_viewer_panels PUBLIC ${riptide_msgs2_TARGETS})
  if(chameleon_tf_msgs_FOUND)
    target_compile_definitions(nereus_viewer_panels PUBLIC NEREUS_VIEWER_HAVE_CHAMELEON=1)
    target_link_libraries(nereus_viewer_panels PUBLIC ${chameleon_tf_msgs_TARGETS})
  else()
    message(STATUS "chameleon_tf_msgs not found: mapping calibration action compiled out")
  endif()
endif()

if(BUILD_TESTING)
  set(_viewer_config ${PROJECT_SOURCE_DIR}/content/viewer/talos_uwrt_panels.yaml)
  add_executable(panels_gizmo_input_test ../tests/panels_gizmo_input.cpp)
  add_executable(panels_composition_test ../tests/panels_panel_composition.cpp)
  add_executable(panels_panel_ros_test ../tests/panels_panel_ros.cpp)
  add_executable(panels_operator_test ../tests/panels_operator_panels.cpp)
  foreach(t panels_gizmo_input_test panels_composition_test panels_panel_ros_test panels_operator_test)
    target_link_libraries(${t} PRIVATE nereus_viewer_panels)
    target_compile_options(${t} PRIVATE -UNDEBUG)
  endforeach()
  add_test(NAME panels_gizmo_input COMMAND panels_gizmo_input_test)
  add_test(NAME panels_composition COMMAND panels_composition_test)
  if(NEREUS_ROS_VIEWER_UWRT)
    add_test(NAME panels_ros COMMAND panels_panel_ros_test ${_viewer_config}
      ${CMAKE_CURRENT_SOURCE_DIR}/../tests/panels_standard_example.yaml)
    add_test(NAME panels_operator COMMAND panels_operator_test ${_viewer_config} ${_viewer_config})
    set_tests_properties(panels_ros PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
    set_tests_properties(panels_operator PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
  endif()
endif()
