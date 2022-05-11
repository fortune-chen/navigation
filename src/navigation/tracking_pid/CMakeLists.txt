cmake_minimum_required(VERSION 3.0.2)

project(tracking_pid)

add_compile_options(-fPIC)

aux_source_directory(./src SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})

set(SYSTEM_LINK_LIBS orocos-kdl boost_system pthread)
set(ROS_LINK_LIBS costmap_2d base_local_planner transformer ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test test/tracking_pid_test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${SYSTEM_LINK_LIBS} ${ROS_LINK_LIBS})
