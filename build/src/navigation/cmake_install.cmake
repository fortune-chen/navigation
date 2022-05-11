# Install script for directory: /home/chen/project/fl-slam/src/navigation

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/chen/project/fl-slam/build/src/navigation/base_local_planner/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/clear_costmap_recovery/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/costmap_2d/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/dwa_local_planner/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/global_planner/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/laser_geometry/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/move_base/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/sensor_bridge/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/full_coverage_path_planner/cmake_install.cmake")
  include("/home/chen/project/fl-slam/build/src/navigation/tracking_pid/cmake_install.cmake")

endif()

