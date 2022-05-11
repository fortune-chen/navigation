cmake_minimum_required(VERSION 3.10)

project(app_server)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Eigen eigen VERSION 3.4.0)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/trimmer/src/include)
link_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)

add_library(app_server_lib STATIC "flslam_api.cpp" "mapping.cpp" "navigation.cpp" "trimmer.cpp")

add_executable(${PROJECT_NAME} "app_server_main.cpp" )

target_link_libraries(${PROJECT_NAME} app_server_lib dds_wrapper)

install(TARGETS ${PROJECT_NAME} app_server_lib ARCHIVE DESTINATION ./lib LIBRARY DESTINATION ./lib RUNTIME DESTINATION ./bin)
set(head_files 
    "mapping.h"
    "navigation.h"
    "trimmer.h"    
)
install(FILES  ${head_files} DESTINATION ./include)
