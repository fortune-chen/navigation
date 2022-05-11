cmake_minimum_required(VERSION 3.10)

project(dds_wrapper_example)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

file(GLOB DDS_EXAMPLE_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/DDSExample*")

add_executable(${PROJECT_NAME} dds_example.cpp  ${DDS_EXAMPLE_SOURCES})

target_link_libraries(${PROJECT_NAME} dds_wrapper pthread)

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ./lib LIBRARY DESTINATION ./lib RUNTIME DESTINATION ./bin)

