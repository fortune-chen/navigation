cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(dds_wrapper)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)
fwslam_find_thirdparty(Tinyxml2 tinyxml2 VERSION 6.0.0)
fwslam_find_thirdparty(Foonathan-memory foonathan-memory VERSION 1.2.0)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)
fwslam_find_thirdparty(Roscpp-core roscpp-core VERSION 1.0.0)

include_directories(include)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)

file(GLOB ROSADAPTERCONTENT_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/RosAdapterContent*")
file(GLOB NODE_CONTROL_COMMAND_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/NodeControlCommand*")
file(GLOB ROOM_PUBLISH_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/RoomPublish*")
file(GLOB TF2MESSAGE_SOURCES "${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output/TF2Message*")

add_library(${PROJECT_NAME} SHARED src/dds_publisher.cpp src/dds_subscriber.cpp ${ROSADAPTERCONTENT_SOURCES} ${NODE_CONTROL_COMMAND_SOURCES} ${TF2MESSAGE_SOURCES} ${ROOM_PUBLISH_SOURCES})

set(THIRD_LIBRARIES
"${tinyxml2_LIBRARY_DIR}/libtinyxml2.so" "${tinyxml2_LIBRARY_DIR}/libtinyxml2.so.6" "${tinyxml2_LIBRARY_DIR}/libtinyxml2.so.6.0.0"
)
set(ARCHIVE_LIBRARIES -Wl,--whole-archive fastrtps fastcdr foonathan_memory-0.7.1 roscore -Wl,--no-whole-archive)

target_link_libraries(${PROJECT_NAME} PRIVATE ${ARCHIVE_LIBRARIES})
target_link_libraries(${PROJECT_NAME} PUBLIC boost_system tinyxml2 ssl rt)
install(FILES ${THIRD_LIBRARIES} DESTINATION ./lib)
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ./lib
        LIBRARY DESTINATION ./lib
        RUNTIME DESTINATION ./bin
)
