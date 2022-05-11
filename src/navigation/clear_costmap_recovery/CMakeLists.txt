cmake_minimum_required(VERSION 3.0.2)
project(clear_costmap_recovery)

add_compile_options(-fPIC)

aux_source_directory(./src SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})
target_link_libraries(${PROJECT_NAME} boost_system)

