cmake_minimum_required(VERSION 3.10)

project(collector)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)

include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/dds)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)

file(GLOB_RECURSE ALL_COLLECTOR_HDRS "src/*.h")
file(GLOB_RECURSE ALL_COLLECTOR_SRCS "src/*.cpp")

add_library(collector_lib STATIC ${ALL_COLLECTOR_SRCS})

add_executable(${PROJECT_NAME} "src/collector_main.cpp" )

target_link_libraries(${PROJECT_NAME} collector_lib dds_wrapper boost_thread)

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION ./lib LIBRARY DESTINATION ./lib RUNTIME DESTINATION ./bin)

