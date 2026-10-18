# `ros2 launch nereus <file>` without making Nereus an ament package: a minimal ament prefix in the build tree
# (build/<preset>/ros) that registers a `nereus` package, whose share directory holds symlinks to the launch files
# registered with nereus_register_launch(), plus a setup.bash that puts the prefix on AMENT_PREFIX_PATH.
# Symlinks, not copies: the launch files find the repository from their resolved path.
set(NEREUS_ROS_PREFIX ${CMAKE_BINARY_DIR}/ros)
set(NEREUS_ROS_LAUNCH_DIR ${NEREUS_ROS_PREFIX}/share/nereus/launch)

# Start from an empty launch directory so a launch file no longer registered disappears on reconfigure.
file(REMOVE_RECURSE ${NEREUS_ROS_LAUNCH_DIR})
file(MAKE_DIRECTORY ${NEREUS_ROS_LAUNCH_DIR})
file(WRITE ${NEREUS_ROS_PREFIX}/share/ament_index/resource_index/packages/nereus "")
configure_file(${CMAKE_CURRENT_LIST_DIR}/ros_setup.bash.in ${NEREUS_ROS_PREFIX}/setup.bash @ONLY)

# Exposes launch files (relative to the calling CMakeLists.txt) as `ros2 launch nereus <file name>`.
function(nereus_register_launch)
  foreach(file IN LISTS ARGN)
    get_filename_component(path ${file} ABSOLUTE)
    get_filename_component(name ${file} NAME)
    if(NOT EXISTS ${path})
      message(FATAL_ERROR "nereus_register_launch: ${path} does not exist")
    endif()
    file(CREATE_LINK ${path} ${NEREUS_ROS_LAUNCH_DIR}/${name} SYMBOLIC)
  endforeach()
endfunction()
