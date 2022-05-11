include(${PROJECT_SOURCE_DIR}/cmake/fwslam_thirdpart_libraries.cmake)

fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Eigen eigen VERSION 3.4.0)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)
fwslam_find_thirdparty(Gflags gflags VERSION 2.2.2)
fwslam_find_thirdparty(Glog glog VERSION 0.4.0)
fwslam_find_thirdparty(Libpng libpng VERSION 1.6.34)
fwslam_find_thirdparty(Lua lua VERSION 5.2.0)
fwslam_find_thirdparty(Ncurses ncurses VERSION 6.1)
fwslam_find_thirdparty(Orocos-kdl orocos-kdl VERSION 1.4.0)
fwslam_find_thirdparty(Pixman pixman VERSION 0.34.0)
fwslam_find_thirdparty(Tinyxml tinyxml VERSION 1.0)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)
fwslam_find_thirdparty(Roscpp-core roscpp-core VERSION 1.0.0)

set(NAVIGATION_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(PACKAGE_OUTPUT_DIR               "${CMAKE_INSTALL_PREFIX}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/bin")
set(CMAKE_INCLUDE_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/include")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/lib")

set(COSTMAP_2D_INCLUDE              ${NAVIGATION_DIR}/costmap_2d/include)
set(GLOBAL_PLANNER_INCLUDE          ${NAVIGATION_DIR}/global_planner/include)
set(NAV_CORE_INCLUDE                ${NAVIGATION_DIR}/nav_core/include)

if (COMPILE_TRANSFORMER)
    set(TF2_ROBOT_STATE_PUBLISHER       ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/robot_state_publisher/include)
    set(TF2_INCLUDE                     ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/tf2/include)
    set(TF2_ROS_INCLUDE                 ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/tf2_ros/include)
    set(TF2_BRIDGE_INCLUDE              ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/tf2_bridge/include)
else()
    set(TF2_ROBOT_STATE_PUBLISHER       )
    set(TF2_INCLUDE                     ${FW_SLAM_PROJECT_TOP_DIR}/src/tf2/include)
    set(TF2_ROS_INCLUDE                 ${FW_SLAM_PROJECT_TOP_DIR}/src/tf2_ros/include)
    set(TF2_BRIDGE_INCLUDE              )
endif()

set(LASER_GEOMETRY_INCLUDE          ${NAVIGATION_DIR}/laser_geometry/include)
set(BASE_LOCAL_PLANNER_INCLUDE      ${NAVIGATION_DIR}/base_local_planner/include)
set(DWA_LOCAL_PLANNER_INCLUDE       ${NAVIGATION_DIR}/dwa_local_planner/include)
set(SENSOR_BRIDGE_INCLUDE           ${NAVIGATION_DIR}/sensor_bridge/include)
set(CLEAR_COSTMAP_RECOVERY_INCLUDE  ${NAVIGATION_DIR}/clear_costmap_recovery/include)
set(MOVE_BASE_INCLUDE               ${NAVIGATION_DIR}/move_base/include)
set(FULL_COVERAGE_PATH_PLANNER      ${NAVIGATION_DIR}/full_coverage_path_planner/include)
set(UTILS_INCLUDE                   ${FW_SLAM_PROJECT_TOP_DIR}/utils)
set(DDS_WRAPPER_INCLUDE_DIR         ${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
set(DDS_WRAPPER_LIBRARY_DIR         ${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)
set(IDL_OUTPUT_INCLUDE_DIR          ${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
set(TRACKING_PID_INCLUDE_DIR        ${NAVIGATION_DIR}/tracking_pid/include)

include_directories(
    ${UTILS_INCLUDE}
    ${COSTMAP_2D_INCLUDE}
    ${GLOBAL_PLANNER_INCLUDE}
    ${NAV_CORE_INCLUDE}
    ${TF2_ROBOT_STATE_PUBLISHER}
    ${TF2_ROS_INCLUDE}
    ${TF2_INCLUDE}
    ${TF2_BRIDGE_INCLUDE}
    ${LASER_GEOMETRY_INCLUDE}
    ${BASE_LOCAL_PLANNER_INCLUDE}
    ${DWA_LOCAL_PLANNER_INCLUDE}
    ${SENSOR_BRIDGE_INCLUDE}
    ${ROBOT_STATE_PUBLISHER_INCLUDE}
    ${CLEAR_COSTMAP_RECOVERY_INCLUDE}
    ${MOVE_BASE_INCLUDE}
    ${FULL_COVERAGE_PATH_PLANNER}
    ${ASYNC_SOCKETS_INCLUDE_DIR}
    ${DDS_WRAPPER_INCLUDE_DIR}
    ${DDS_WRAPPER_LIBRARY_DIR}
    ${IDL_OUTPUT_INCLUDE_DIR}
    ${TRACKING_PID_INCLUDE_DIR}
)

add_subdirectory(base_local_planner)
add_subdirectory(clear_costmap_recovery)
add_subdirectory(costmap_2d)
add_subdirectory(dwa_local_planner)
add_subdirectory(global_planner)
add_subdirectory(laser_geometry)
add_subdirectory(move_base)
add_subdirectory(sensor_bridge)
add_subdirectory(full_coverage_path_planner)
add_subdirectory(tracking_pid)