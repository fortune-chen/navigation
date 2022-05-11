cmake_minimum_required(VERSION 3.10)  # Ships with Ubuntu 14.04 (Trusty)
project(ros_sensor_bridge)


#####################################################################
## build sensor bridge at ubuntu x86 enviroment
## run it as follow, parameter is "publisher local ip" and "subscriber remote ip"
## ./mapping/bin/sensor_bridge_server_dds 10.10.19.223 10.10.62.173
#####################################################################
set(ROS_INSTALL_PATH /opt/ros/melodic)
include_directories(${ROS_INSTALL_PATH}/include ${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${ROS_INSTALL_PATH}/lib)

include_directories(${ASYNC_SOCKETS_INCLUDE_DIR})

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

include_directories(include)

# add_executable(sensor_bridge_server
#             src/sensor_bridge_ros_server.cc
#             src/ros_sensor_bridge.cc)
# target_link_libraries(sensor_bridge_server
#                 rosconsole
#                 roscpp
#                 roscpp_serialization
#                 rostime
#                 pthread)

# target_link_libraries(sensor_bridge_server ${Boost_LIBRARIES})

#####################################################################

add_executable(${PROJECT_NAME}
            src/sensor_bridge_ros_server_dds.cc
            src/ros_sensor_bridge.cc)
target_link_libraries(${PROJECT_NAME}
                rosconsole
                roscpp
                roscpp_serialization
                rostime
                pthread)

target_link_libraries(${PROJECT_NAME} dds_wrapper ${Boost_LIBRARIES})

#####################################################################

add_library(sensor_bridge_client SHARED
            src/sensor_bridge_client.cc)

# add_executable(sensor_bridge
#             src/sensor_bridge_client_main.cc)
# target_link_libraries(sensor_bridge
#                 sensor_bridge_client
#                 roscpp_serialization
#                 pthread)

# target_link_libraries(sensor_bridge ${Boost_LIBRARIES})

install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ./lib
        LIBRARY DESTINATION ./lib
        RUNTIME DESTINATION ./bin
)