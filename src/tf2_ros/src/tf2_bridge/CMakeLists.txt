cmake_minimum_required(VERSION 3.10)

project(tf2_bridge)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/robot_state_publisher/include)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/utils)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

file(GLOB TF2COMMUNICATION_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/TF2Communication*")

set(SYSTEM_LINK_LIBS pthread)
set(DDS_LINK_LIBS dds_wrapper)
set(TF2_LINK_LIBS tf2_ros tf2 tf2_robot_state_publisher)
set(OTHERS_LINK_LIBS utils)

add_library(${PROJECT_NAME} SHARED tf2_bridge_client.cc tf2_bridge_cast.cc ${TF2COMMUNICATION_SOURCES})
target_link_libraries(${PROJECT_NAME} ${DDS_LINK_LIBS} ${OTHERS_LINK_LIBS})

# tf2_bridge
# add_executable(tf2_bridge_test tf2_bridge_test.cc)
# target_link_libraries(tf2_bridge_test ${PROJECT_NAME} ${SYSTEM_LINK_LIBS})

add_executable(tf2_node tf2_node.cc tf2_bridge_server.cc tf2_bridge_cast.cc ${TF2COMMUNICATION_SOURCES})
target_link_libraries(tf2_node ${SYSTEM_LINK_LIBS} ${ROS_LINK_LIBS} ${DDS_LINK_LIBS} ${TF2_LINK_LIBS} ${OTHERS_LINK_LIBS})
