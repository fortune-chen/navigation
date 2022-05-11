# Install script for directory: /home/chen/project/fl-slam/src/navigation/full_coverage_path_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE FILE FILES
    "/home/chen/project/fl-slam/thirdparty/x86-64/gflags-2.2.2/lib/libgflags.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/gflags-2.2.2/lib/libgflags.so.2.2"
    "/home/chen/project/fl-slam/thirdparty/x86-64/gflags-2.2.2/lib/libgflags.so.2.2.2"
    "/home/chen/project/fl-slam/thirdparty/x86-64/glog-0.4.0/lib/libglog.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/glog-0.4.0/lib/libglog.so.0"
    "/home/chen/project/fl-slam/thirdparty/x86-64/glog-0.4.0/lib/libglog.so.0.4.0"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml-1.0/lib/libtinyxml.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so.6"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so.6.0.0"
    "/home/chen/project/fl-slam/thirdparty/x86-64/orocos-kdl-1.4.0/lib/liborocos-kdl.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/orocos-kdl-1.4.0/lib/liborocos-kdl.so.1.4"
    "/home/chen/project/fl-slam/thirdparty/x86-64/orocos-kdl-1.4.0/lib/liborocos-kdl.so.1.4.0"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_system.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_system.so.1.65.1"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_iostreams.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_iostreams.so.1.65.1"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_thread.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_thread.so.1.65.1"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_date_time.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_date_time.so.1.65.1"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_chrono.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_chrono.so.1.65.1"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_atomic.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib/libboost_atomic.so.1.65.1"
    )
endif()

