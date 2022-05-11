cmake_minimum_required(VERSION 3.10)
project(tf2)

include(${FW_SLAM_PROJECT_TOP_DIR}/cmake/fwslam_thirdpart_libraries.cmake)
fwslam_find_thirdparty(Boost boost VERSION 1.65.1)

include_directories(include)
#CPP Libraries
add_library(${PROJECT_NAME} SHARED
        src/cache.cpp
        src/buffer_core.cpp
        src/static_cache.cpp)
install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ./lib
        LIBRARY DESTINATION ./lib
        RUNTIME DESTINATION ./bin
)