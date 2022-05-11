cmake_minimum_required(VERSION 3.10)
project(transformer)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)

fwslam_find_thirdparty(Eigen eigen VERSION 3.4.0)
fwslam_find_thirdparty(Tinyxml tinyxml VERSION 1.0)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)
fwslam_find_thirdparty(Roscpp-core roscpp-core VERSION 1.0.0)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Orocos-kdl orocos-kdl VERSION 1.4.0)

set(TRANSFORMER_DIR                         ${CMAKE_CURRENT_SOURCE_DIR})
set(ROBOT_STATE_PUBLISHER_INCLUDE           ${TRANSFORMER_DIR}/robot_state_publisher/include)
set(TF2_BRIDGE_INCLUDE                      ${TRANSFORMER_DIR}/tf2_bridge/include)
set(TF2_ROS_INCLUDE                         ${TRANSFORMER_DIR}/tf2_ros/include)
set(TF2_INCLUDE                             ${TRANSFORMER_DIR}/tf2/include)
set(UTILS_INCLUDE                           ${FW_SLAM_PROJECT_TOP_DIR}/utils)

include_directories(
    ${ROBOT_STATE_PUBLISHER_INCLUDE}
    ${TF2_BRIDGE_INCLUDE}
    ${TF2_ROS_INCLUDE}
    ${TF2_INCLUDE}
    ${UTILS_INCLUDE}
)

add_subdirectory(robot_state_publisher)
if (TF2_BRIDGE)
add_subdirectory(tf2_bridge)
endif()
add_subdirectory(tf2_ros)
add_subdirectory(tf2)

if (TF2_BRIDGE)

else()
    add_library(${PROJECT_NAME} SHARED
        tf2/src/cache.cpp
        tf2/src/buffer_core.cpp
        tf2/src/static_cache.cpp
    )
    set(ARCHIVE_LIBRARIES -Wl,--whole-archive tf2_ros tf2_robot_state_publisher -Wl,--no-whole-archive)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${ARCHIVE_LIBRARIES})

    install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ./lib
        LIBRARY DESTINATION ./lib
        RUNTIME DESTINATION ./bin
    )
endif()