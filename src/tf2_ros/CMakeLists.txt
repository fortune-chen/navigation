cmake_minimum_required(VERSION 3.10)
project(tf2_ros)

if(NOT ANDROID)
set(TF2_PY tf2_py)
endif()

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Roscpp-core roscpp-core VERSION 1.0.0)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Orocos-kdl orocos-kdl VERSION 1.4.0)
fwslam_find_thirdparty(Tinyxml tinyxml VERSION 1.0)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)

include_directories(include)
include_directories(../tf2/include)
include_directories(src/robot_state_publisher/include)

link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

# tf2_bridge
add_subdirectory(src/tf2_bridge)
# robot_state_publisher
add_subdirectory(src/robot_state_publisher)

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/transform_listener.cpp
  # src/buffer_client.cpp
  # src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)

# set(ROS_INSTALL_PATH "/opt/ros/melodic")
# target_link_libraries(${PROJECT_NAME} ${ROS_INSTALL_PATH}/lib/librostime.so)
# target_link_libraries(${PROJECT_NAME} ${ROS_INSTALL_PATH}/lib/libcpp_common.so)

# buffer_server executable
add_executable(${PROJECT_NAME}_buffer_server src/buffer_server_main.cpp)
target_link_libraries(${PROJECT_NAME}_buffer_server
  tf2_ros
  tf2
  dds_wrapper
  boost_system
  boost_date_time
  boost_thread
  boost_chrono
  boost_atomic
)
set_target_properties(${PROJECT_NAME}_buffer_server
  PROPERTIES OUTPUT_NAME buffer_server
)

# static_transform_publisher
# add_executable(${PROJECT_NAME}_static_transform_publisher
#   src/static_transform_broadcaster_program.cpp
# )
# add_dependencies(${PROJECT_NAME}_static_transform_publisher ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_static_transform_publisher
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# set_target_properties(${PROJECT_NAME}_static_transform_publisher
#   PROPERTIES OUTPUT_NAME static_transform_publisher
# )

# Install library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ./lib
  LIBRARY DESTINATION ./lib
  RUNTIME DESTINATION ./bin
)
# # Install executable
# install(TARGETS
#   ${PROJECT_NAME}_buffer_server ${PROJECT_NAME}_static_transform_publisher
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )


# # Tests
# if(CATKIN_ENABLE_TESTING)

# # new requirements for testing
# find_package(catkin REQUIRED COMPONENTS
#   actionlib
#   actionlib_msgs
#   geometry_msgs
#   message_filters
#   roscpp
#   rosgraph
#   rospy
#   rostest
#   tf2
#   tf2_msgs
#   ${TF2_PY}
# )



# # tf2_ros_test_listener
# add_executable(${PROJECT_NAME}_test_listener EXCLUDE_FROM_ALL test/listener_unittest.cpp)
# add_dependencies(${PROJECT_NAME}_test_listener ${catkin_EXPORTED_TARGETS})
# add_executable(${PROJECT_NAME}_test_time_reset EXCLUDE_FROM_ALL test/time_reset_test.cpp)
# add_dependencies(${PROJECT_NAME}_test_time_reset ${catkin_EXPORTED_TARGETS})
# add_executable(${PROJECT_NAME}_test_message_filter EXCLUDE_FROM_ALL test/message_filter_test.cpp)
# add_dependencies(${PROJECT_NAME}_test_message_filter ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_test_listener
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${GTEST_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_test_time_reset
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${GTEST_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_test_message_filter
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${GTEST_LIBRARIES}
# )

# add_dependencies(tests ${PROJECT_NAME}_test_listener)
# add_dependencies(tests ${PROJECT_NAME}_test_time_reset)
# add_dependencies(tests ${PROJECT_NAME}_test_message_filter)

# add_rostest(test/transform_listener_unittest.launch)
# add_rostest(test/transform_listener_time_reset_test.launch)
# add_rostest(test/message_filter_test.launch)

# endif()
