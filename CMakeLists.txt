###############################################################################
# CMake build rules for FL SLAM
###############################################################################
cmake_minimum_required(VERSION 3.10)
cmake_policy(VERSION 3.5...3.13)
project(flslam VERSION "1.0.0" LANGUAGES C CXX)

###############################################################################
# Set project Option                                                          #
###############################################################################
# Set CMAKE_BUILD_TYPE to Release by default.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
	"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
	FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "-s -O2")
endif()

if(NOT BUILD_TARGET_CPU_TYPE)
    message(STATUS "Setting build target type to 'x86-64' as none was specified.")
    set(BUILD_TARGET_CPU_TYPE x86-64 CACHE STRING
	"Choose the type of build target, options are: None Debug x86-64 aarch64." FORCE)
    set_property(CACHE BUILD_TARGET_CPU_TYPE PROPERTY STRINGS "x86-64" "aarch64")
endif()

set(FW_SLAM_PROJECT_TOP_DIR ${PROJECT_SOURCE_DIR})

option(GENERATE_IDL         "Generate idl message type" OFF)
option(COMPILE_EXAMPLES     "Build example"             ON)
option(COMPILE_TEST         "Build test"                OFF)
option(COMPILE_MAPPING      "Build mapping"             ON)
option(COMPILE_NAVIGATION   "Build navigation"          ON)
option(COMPILE_TRANSFORMER  "Build transformer"         ON)
option(TF2_BRIDGE           "Used tf2_bridge"           OFF)
option(MULTI_ECHO_SCAN      "Use multi echo scan"       OFF)
option(JUST_NAVIGATION      "Just used in navigation"   OFF)
option(GNSS_MAPPING         "Use Gnss mapping"          OFF)
option(COMPILE_IMSHOW       "Build cv::imshow()"        ON)

###############################################################################
# Enter to sub directry to build package
###############################################################################
if (GENERATE_IDL)
# Please install fastddsgen by "sudo apt install fastddsgen" in your ubuntu develop enviroment
EXEC_PROGRAM("mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/idl_output; fastddsgen ${FW_SLAM_PROJECT_TOP_DIR}/idl/*.idl -d ${CMAKE_CURRENT_BINARY_DIR}/idl_output")
else ()
EXEC_PROGRAM("mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/idl_output; cp -rf ${PROJECT_SOURCE_DIR}/idl/idl_cpp/* ${CMAKE_CURRENT_BINARY_DIR}/idl_output/")
endif ()

include_directories(include/ros_header)

if (GNSS_MAPPING)
    add_definitions(-DUSED_GNSS_MAPPING)
endif()

if (MULTI_ECHO_SCAN)
    add_definitions(-DUSED_MULTI_ECHO_SCAN)
endif()

if (TF2_BRIDGE)
    add_definitions(-DUSED_TF2_BRIDGE)
endif()

if (JUST_NAVIGATION)
    add_definitions(-DUSED_NAVIGATION_ONLY)
    add_definitions(-DUSED_ROS_TRANSFORM)
endif()

if (COMPILE_NAVIGATION)
    add_definitions(-DENABLE_FULL_COVERAGE_PATH_PLANNER)
endif()

if (COMPILE_IMSHOW)
    add_definitions(-DDISPLAY_IMSHOW)
endif()

add_subdirectory(utils)
add_subdirectory(src/async_sockets)
add_subdirectory(src/dds_wrapper)
add_subdirectory(src/app_server)
add_subdirectory(src/trimmer)

if (COMPILE_TRANSFORMER)
    add_subdirectory(src/transformer)
else()
    add_subdirectory(src/tf2)
    add_subdirectory(src/tf2_ros)
endif()

if (COMPILE_EXAMPLES)
add_subdirectory(examples/dds_wrapper)
endif ()

if (${BUILD_TARGET_CPU_TYPE} STREQUAL "x86-64") ## ros bridge only for x86 ros enviroment
    add_subdirectory(src/ros_sensor_bridge)
endif ()
if (COMPILE_MAPPING)
    add_subdirectory(src/mapping)
endif ()
if (COMPILE_NAVIGATION)
    add_subdirectory(src/navigation)
endif ()

include(cmake/flslam_output_pack.cmake)
