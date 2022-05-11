############################################
## CMakeLists.txt
#############################################
cmake_minimum_required(VERSION 2.8)

find_package(OpenCV 3)  

fwslam_find_thirdparty(Fast-dds fast-dds VERSION 2.5.0)
fwslam_find_thirdparty(Fast-cdr fast-cdr VERSION 1.0.23)

option(DISPLAY_IMSHOW "enable debug display image" ON)

if (DISPLAY_IMSHOW)
    add_definitions(-DDISPLAY_IMSHOW)
endif()

include_directories(${FW_SLAM_PROJECT_TOP_DIR}/build/idl_output)    
include_directories(${FW_SLAM_PROJECT_TOP_DIR}/src/dds_wrapper/include)

include_directories (src/include)
include_directories (src/ipa_room_segmentation/common/include)
include_directories (src/ipa_room_segmentation/common/src)
aux_source_directory (src/ipa_room_segmentation/common/src SRC_LIST)

add_executable(trimmer_node ${SRC_LIST} src/trimmer_main.cc src/map_mgr.cc src/trimmer_dds_client.cc )

add_executable(trimmer_test src/test/trimmer_test.cpp)
target_link_libraries(trimmer_test dds_wrapper pthread opencv_imgcodecs opencv_imgproc opencv_core opencv_highgui)

target_link_libraries(trimmer_node dds_wrapper pthread)
target_link_libraries(trimmer_node opencv_imgcodecs opencv_imgproc opencv_core opencv_highgui) 

set(THIRD_LIBRARIES 
"${fast-dds_LIBRARY_DIR}/libfastrtps.a" "${fast-cdr_LIBRARY_DIR}/libfastcdr.a" 
"${FW_SLAM_PROJECT_TOP_DIR}/build/src/dds_wrapper/libdds_wrapper.so")
install(FILES ${THIRD_LIBRARIES} DESTINATION ./lib)

install(TARGETS trimmer_node ARCHIVE DESTINATION ./lib LIBRARY DESTINATION ./lib RUNTIME DESTINATION ./bin)
install(TARGETS trimmer_test ARCHIVE DESTINATION ./lib LIBRARY DESTINATION ./lib RUNTIME DESTINATION ./bin)
