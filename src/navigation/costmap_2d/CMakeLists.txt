cmake_minimum_required(VERSION 3.0.2)
project(costmap_2d)

add_compile_options(-fPIC)

aux_source_directory(./src SRC)

add_library(${PROJECT_NAME} SHARED ${SRC})
target_link_libraries(${PROJECT_NAME} sensor_bridge_nav transformer boost_system boost_thread laser_geometry)
