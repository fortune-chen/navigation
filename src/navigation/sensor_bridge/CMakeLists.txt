cmake_minimum_required(VERSION 3.0.2)
project(sensor_bridge_nav)

add_compile_options(-fPIC)

include_directories(
    ${FW_SLAM_PROJECT_TOP_DIR}/src/async-sockets/include
)
aux_source_directory(./src SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})
target_link_libraries(${PROJECT_NAME} boost_system dds_wrapper pthread)


