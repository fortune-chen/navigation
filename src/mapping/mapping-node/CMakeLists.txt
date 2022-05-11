cmake_minimum_required(VERSION 3.10)  # Ships with Ubuntu 14.04 (Trusty)

set(PACKAGE_OUTPUT_DIR               "${CMAKE_INSTALL_PREFIX}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/bin")
set(CMAKE_INCLUDE_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/include")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY   "${PACKAGE_OUTPUT_DIR}/lib")

add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)

file(GLOB_RECURSE ALL_SRCS "src/*.cc" "src/*.cpp" "src/*.h")
file(GLOB_RECURSE ALL_TESTS "src/*_test.cc")
file(GLOB_RECURSE ASSETS_WRITER "src/assets_writer.cc")
file(GLOB_RECURSE OFFLINE_NODE "src/offline_node.cc")
file(GLOB_RECURSE PLAYABLE_BAG "src/playable_bag.cc")
file(GLOB_RECURSE ROS_LOG_SINK "src/ros_log_sink.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "src/mapping_grpc/*")
file(GLOB_RECURSE ALL_DEPRECATED "src/*_deprecated.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ASSETS_WRITER})
list(REMOVE_ITEM ALL_SRCS ${OFFLINE_NODE})
list(REMOVE_ITEM ALL_SRCS ${PLAYABLE_BAG})
list(REMOVE_ITEM ALL_SRCS ${ROS_LOG_SINK})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_SRCS ${ALL_DEPRECATED})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()

add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})

set(CARTOGRAPHER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../cartographer)
set(RTKFUSION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../rtkfusion)
set(CARTOGRAPHER_PB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pb)
set(TF2_INCLUDE_DIR ${FW_SLAM_PROJECT_TOP_DIR}/src/tf2/include)
set(TF2_ROS_INCLUDE_DIR ${FW_SLAM_PROJECT_TOP_DIR}/src/tf2_ros/include)

if (COMPILE_TRANSFORMER)
    set(TF2_BRIDGE_INCLUDE              ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/tf2_bridge/include)
    set(ROBOT_PUBLISHER_INCLUDE              ${FW_SLAM_PROJECT_TOP_DIR}/src/transformer/robot_state_publisher/include)
endif()

set(DDS_WRAPPER_INCLUDE_DIR ${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${CARTOGRAPHER_INCLUDE_DIR}
    ${RTKFUSION_INCLUDE_DIR}
    ${CARTOGRAPHER_PB_INCLUDE_DIR}
    ${ASYNC_SOCKETS_INCLUDE_DIR}
    ${TF2_INCLUDE_DIR}
    ${TF2_ROS_INCLUDE_DIR}
    ${TF2_BRIDGE_INCLUDE}
    ${ROBOT_PUBLISHER_INCLUDE}
    ${DDS_WRAPPER_INCLUDE_DIR}
)

set(CARTOGRAPHER_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/${BUILD_TARGET_CPU_TYPE})

set(DDS_WRAPPER_LIBRARY_DIR ${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper)
set(TRANSFORMER_LIBRARY_DIR ${FW_SLAM_PROJECT_TOP_DIR}/build/src/transformer)
link_directories(${CARTOGRAPHER_LIBRARY_DIR} ${DDS_WRAPPER_LIBRARY_DIR} ${TRANSFORMER_LIBRARY_DIR})

set(absl_DIR ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/abseil-20211102/lib/cmake/absl)
find_package(absl REQUIRED)

set(DDS_LINK_LIBS dds_wrapper)
set(GOOGLE_LINK_LIBS cartographer cairo ceres libpng16.a libpixman-1.a libz.so lua gflags glog protobuf)

set(RTK_FUSION_LINK_LIBS rtkfusion)

if (COMPILE_TRANSFORMER)
    set(ROS_LINK_LIBS transformer)
else()
    set(ROS_LINK_LIBS tf2 tf2_ros)
endif()
set(SYSTEM_LINK_LIBS dl pcl_common tinyxml tinyxml2 orocos-kdl boost_system boost_thread boost_iostreams boost_date_time boost_chrono boost_atomic)
set(ABSL_LINK_LIBS  absl::algorithm absl::base absl::debugging absl::flat_hash_map absl::memory absl::meta absl::numeric absl::str_format absl::strings absl::synchronization absl::time absl::utility)

add_executable(mapping_node src/node_main.cc)
target_link_libraries(mapping_node PRIVATE ${PROJECT_NAME} ${DDS_LINK_LIBS} ${GOOGLE_LINK_LIBS} ${RTK_FUSION_LINK_LIBS} ${ROS_LINK_LIBS} ${SYSTEM_LINK_LIBS} ${ABSL_LINK_LIBS})

set(THIRD_LIBRARIES
"${gflags_LIBRARY_DIR}/libgflags.so" "${gflags_LIBRARY_DIR}/libgflags.so.2.2" "${gflags_LIBRARY_DIR}/libgflags.so.2.2.2"
"${glog_LIBRARY_DIR}/libglog.so" "${glog_LIBRARY_DIR}/libglog.so.0" "${glog_LIBRARY_DIR}/libglog.so.0.4.0"
"${protobuf_LIBRARY_DIR}/libprotobuf.so" "${protobuf_LIBRARY_DIR}/libprotobuf.so.29" "${protobuf_LIBRARY_DIR}/libprotobuf.so.29.0.0"
"${tinyxml_LIBRARY_DIR}/libtinyxml.so"
"${tinyxml2_LIBRARY_DIR}/libtinyxml2.so" "${tinyxml2_LIBRARY_DIR}/libtinyxml2.so.6" "${tinyxml2_LIBRARY_DIR}/libtinyxml2.so.6.0.0"
"${orocos-kdl_LIBRARY_DIR}/liborocos-kdl.so" "${orocos-kdl_LIBRARY_DIR}/liborocos-kdl.so.1.4" "${orocos-kdl_LIBRARY_DIR}/liborocos-kdl.so.1.4.0"
"${boost_LIBRARY_DIR}/libboost_system.so" "${boost_LIBRARY_DIR}/libboost_system.so.1.65.1"
"${boost_LIBRARY_DIR}/libboost_iostreams.so" "${boost_LIBRARY_DIR}/libboost_iostreams.so.1.65.1"
"${boost_LIBRARY_DIR}/libboost_thread.so" "${boost_LIBRARY_DIR}/libboost_thread.so.1.65.1"
"${boost_LIBRARY_DIR}/libboost_date_time.so" "${boost_LIBRARY_DIR}/libboost_date_time.so.1.65.1"
"${boost_LIBRARY_DIR}/libboost_chrono.so" "${boost_LIBRARY_DIR}/libboost_chrono.so.1.65.1"
"${boost_LIBRARY_DIR}/libboost_atomic.so" "${boost_LIBRARY_DIR}/libboost_atomic.so.1.65.1"
)

install(FILES ${THIRD_LIBRARIES} DESTINATION ./lib)
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/config DESTINATION .)
install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/urdf DESTINATION .)
