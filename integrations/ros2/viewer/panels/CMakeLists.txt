# Target nereus_viewer_panels: panel composition, providers and panels of the pool viewer.
# Depends on ROS messages, ImGui, glm and yaml-cpp only; no simulator types.

# Generic panels, providers and the ROS runtime they share.
set(_panel_sources
  panel_registry.cpp composition.cpp motion_panel.cpp mapping_panel.cpp actuator_panel.cpp
  run_panel.cpp simulation_panel.cpp autonomy_panel.cpp pose_gizmo.cpp telemetry_panel.cpp recording_panel.cpp
  dock_layout.cpp theme.cpp pins.cpp
  electrical_panel.cpp bagging_panel.cpp bag_recorder.cpp
  ros_runtime.cpp ros_bagging.cpp standard_motion.cpp sim_run.cpp simulation_rate.cpp
  plots_model.cpp plots_fields.cpp plots_hub.cpp plots_ui.cpp)

# UWRT-specific providers (riptide_msgs2 topics/services) are only built for the UWRT viewer.
if(NEREUS_ROS_VIEWER_UWRT)
  list(APPEND _panel_sources uwrt_motion.cpp uwrt_autonomy.cpp uwrt_mapping.cpp uwrt_actuators.cpp
    uwrt_telemetry.cpp uwrt_recording.cpp uwrt_electrical.cpp)
endif()

add_library(nereus_viewer_panels STATIC ${_panel_sources})
target_compile_features(nereus_viewer_panels PUBLIC cxx_std_17)
target_include_directories(nereus_viewer_panels
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(nereus_viewer_panels PUBLIC nereus_imgui nereus_glm yaml-cpp
  rclcpp::rclcpp rcpputils::rcpputils rclcpp_action::rclcpp_action tf2_ros::tf2_ros
  ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS} ${visualization_msgs_TARGETS} ${action_msgs_TARGETS} ${rcl_interfaces_TARGETS})

# UWRT build: riptide messages plus optional chameleon (mapping calibration) and ZED (SVO recording) support.
if(NEREUS_ROS_VIEWER_UWRT)
  target_compile_definitions(nereus_viewer_panels PUBLIC NEREUS_VIEWER_UWRT=1)
  target_link_libraries(nereus_viewer_panels PUBLIC ${riptide_msgs2_TARGETS} ${diagnostic_msgs_TARGETS})
  if(chameleon_tf_msgs_FOUND)
    target_compile_definitions(nereus_viewer_panels PUBLIC NEREUS_VIEWER_HAVE_CHAMELEON=1)
    target_link_libraries(nereus_viewer_panels PUBLIC ${chameleon_tf_msgs_TARGETS})
  else()
    message(STATUS "chameleon_tf_msgs not found: mapping calibration action compiled out")
  endif()
  if(zed_msgs_FOUND)
    target_compile_definitions(nereus_viewer_panels PUBLIC NEREUS_VIEWER_HAVE_ZED=1)
    target_link_libraries(nereus_viewer_panels PUBLIC ${zed_msgs_TARGETS})
  else()
    message(STATUS "zed_msgs not found: starting SVO recordings compiled out (stop and capture remain)")
  endif()
endif()

if(BUILD_TESTING)
  set(_viewer_config ${PROJECT_SOURCE_DIR}/content/viewer/talos_uwrt_panels.yaml)

  # Test executables; all link the panels library and keep asserts on (-UNDEBUG).
  set(_panel_tests panels_gizmo_input_test panels_composition_test panels_dock_layout_test
    panels_pins_test panels_theme_classic_test panels_theme_heat_sheet_test panels_bagging_test
    panels_plots_test panels_plots_ros_test)
  add_executable(panels_gizmo_input_test ../tests/panels_gizmo_input.cpp)
  add_executable(panels_composition_test ../tests/panels_panel_composition.cpp)
  add_executable(panels_dock_layout_test ../tests/panels_dock_layout.cpp)
  add_executable(panels_pins_test ../tests/panels_pins.cpp)
  add_executable(panels_theme_classic_test ../tests/panels_theme_classic.cpp)
  add_executable(panels_theme_heat_sheet_test ../tests/panels_theme_heat_sheet.cpp)
  add_executable(panels_bagging_test ../tests/panels_bagging.cpp)
  add_executable(panels_plots_test ../tests/panels_plots.cpp)
  add_executable(panels_plots_ros_test ../tests/panels_plots_ros.cpp)
  if(NEREUS_ROS_VIEWER_UWRT) # these two use riptide_msgs2
    add_executable(panels_panel_ros_test ../tests/panels_panel_ros.cpp)
    add_executable(panels_operator_test ../tests/panels_operator_panels.cpp)
    list(APPEND _panel_tests panels_panel_ros_test panels_operator_test)
  endif()

  foreach(t ${_panel_tests})
    target_link_libraries(${t} PRIVATE nereus_viewer_panels)
    target_compile_options(${t} PRIVATE -UNDEBUG)
    target_compile_definitions(${t} PRIVATE NEREUS_VIEWER_THEMES="${PROJECT_SOURCE_DIR}/content/viewer/themes")
  endforeach()

  # Test registrations; the two ROS-backed UWRT tests run serially.
  add_test(NAME panels_gizmo_input COMMAND panels_gizmo_input_test)
  add_test(NAME panels_composition COMMAND panels_composition_test)
  add_test(NAME panels_dock_layout COMMAND panels_dock_layout_test)
  add_test(NAME panels_pins COMMAND panels_pins_test)
  add_test(NAME panels_theme_classic COMMAND panels_theme_classic_test)
  add_test(NAME panels_theme_heat_sheet COMMAND panels_theme_heat_sheet_test ${PROJECT_SOURCE_DIR}/content/viewer/fonts)
  add_test(NAME panels_bagging COMMAND panels_bagging_test)
  set_tests_properties(panels_bagging PROPERTIES TIMEOUT 120)
  add_test(NAME panels_plots COMMAND panels_plots_test)
  add_test(NAME panels_plots_ros COMMAND panels_plots_ros_test)
  set_tests_properties(panels_plots_ros PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
  if(NEREUS_ROS_VIEWER_UWRT)
    add_test(NAME panels_ros COMMAND panels_panel_ros_test ${_viewer_config}
      ${CMAKE_CURRENT_SOURCE_DIR}/../tests/panels_standard_example.yaml)
    add_test(NAME panels_operator COMMAND panels_operator_test ${_viewer_config} ${_viewer_config})
    set_tests_properties(panels_ros PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
    set_tests_properties(panels_operator PROPERTIES TIMEOUT 45 RUN_SERIAL TRUE)
  endif()
endif()
