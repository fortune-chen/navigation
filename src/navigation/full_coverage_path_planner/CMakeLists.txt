cmake_minimum_required(VERSION 3.0.2)

project(full_coverage_path_planner)

add_compile_options(-fPIC)

aux_source_directory(./src SRC)
add_library(${PROJECT_NAME} STATIC ${SRC})

set(SYSTEM_LINK_LIBS orocos-kdl boost_system pthread)
set(ROS_LINK_LIBS costmap_2d base_local_planner ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_test test/full_coverage_path_planner_test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${SYSTEM_LINK_LIBS} ${ROS_LINK_LIBS})

set(THIRD_LIBRARIES 
"${gflags_LIBRARY_DIR}/libgflags.so" "${gflags_LIBRARY_DIR}/libgflags.so.2.2" "${gflags_LIBRARY_DIR}/libgflags.so.2.2.2"
"${glog_LIBRARY_DIR}/libglog.so" "${glog_LIBRARY_DIR}/libglog.so.0" "${glog_LIBRARY_DIR}/libglog.so.0.4.0"
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

install(
  FILES ${THIRD_LIBRARIES}
  DESTINATION lib
)

# install(TARGETS
#     global_planner_node
#     ARCHIVE DESTINATION ./lib
#     LIBRARY DESTINATION ./lib
#     RUNTIME DESTINATION ./bin
# )