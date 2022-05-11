# Install script for directory: /home/chen/project/fl-slam/src/dds_wrapper

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/./lib" TYPE FILE FILES
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so.6"
    "/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib/libtinyxml2.so.6.0.0"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/./lib" TYPE SHARED_LIBRARY FILES "/home/chen/project/fl-slam/build/src/dds_wrapper/libdds_wrapper.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so"
         OLD_RPATH "/home/chen/project/fl-slam/thirdparty/x86-64/fast-dds-2.5.0/lib:/home/chen/project/fl-slam/thirdparty/x86-64/fast-cdr-1.0.23/lib:/home/chen/project/fl-slam/thirdparty/x86-64/tinyxml2-6.0.0/lib:/home/chen/project/fl-slam/thirdparty/x86-64/foonathan-memory-1.2.0/lib:/home/chen/project/fl-slam/thirdparty/x86-64/boost-1.65.1/lib:/home/chen/project/fl-slam/thirdparty/x86-64/roscpp-core-1.0.0/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./lib/libdds_wrapper.so")
    endif()
  endif()
endif()

