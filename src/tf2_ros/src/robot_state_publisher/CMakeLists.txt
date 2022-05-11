cmake_minimum_required(VERSION 3.0.2)
project(tf2_robot_state_publisher)

fwslam_find_thirdparty(Eigen eigen VERSION 3.4.0)
fwslam_find_thirdparty(Tinyxml tinyxml VERSION 1.0)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)

aux_source_directory(./src SRC)

add_library(${PROJECT_NAME} STATIC ${SRC})
target_link_libraries(${PROJECT_NAME} boost_system tinyxml tinyxml2 orocos-kdl)
