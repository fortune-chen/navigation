cmake_minimum_required(VERSION 3.10)
project(tf2_ros)

add_compile_options(-fPIC)

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

file(GLOB TF2MESSAGE_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/TF2Message*")

# tf2_ros library
add_library(${PROJECT_NAME} STATIC
  src/buffer.cpp
  src/transform_listener.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
  src/transform_helper.cpp
  ${TF2MESSAGE_SOURCES}
)
target_link_libraries(${PROJECT_NAME} dds_wrapper)

# add_executable(tf2_transform_test test/tf2_transform_test.cc)
# target_link_libraries(tf2_transform_test boost_system transformer dds_wrapper utils)
