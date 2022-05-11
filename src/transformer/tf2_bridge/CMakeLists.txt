cmake_minimum_required(VERSION 3.10)

project(tf2_bridge)

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

set(SYSTEM_LINK_LIBS pthread)
set(DDS_LINK_LIBS dds_wrapper)
set(TF2_LINK_LIBS tf2_ros tf2 tf2_robot_state_publisher)
set(OTHERS_LINK_LIBS utils)

file(GLOB TF2COMMUNICATION_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/TF2Communication*")

add_library(${PROJECT_NAME} SHARED src/tf2_bridge_client.cc src/tf2_bridge_cast.cc ${TF2COMMUNICATION_SOURCES})
target_link_libraries(${PROJECT_NAME} ${DDS_LINK_LIBS} ${ROS_LINK_LIBS} ${OTHERS_LINK_LIBS})

add_executable(tf2_node src/tf2_node.cc src/tf2_bridge_server.cc src/tf2_bridge_cast.cc ${TF2COMMUNICATION_SOURCES})
target_link_libraries(tf2_node ${SYSTEM_LINK_LIBS} ${ROS_LINK_LIBS} ${DDS_LINK_LIBS} ${TF2_LINK_LIBS} ${OTHERS_LINK_LIBS})

install(TARGETS
    ${PROJECT_NAME} tf2_node
    ARCHIVE DESTINATION ./lib
    LIBRARY DESTINATION ./lib
    RUNTIME DESTINATION ./bin
)