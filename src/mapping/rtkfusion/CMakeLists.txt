cmake_minimum_required(VERSION 3.10)

project(rtkfusion)

set(GEOGRAPHIC_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/geographic/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${GEOGRAPHIC_INCLUDE_DIR})

file(GLOB GEOGRAPHIC_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/geographic/src/*.cpp")
file(GLOB RTKFUSION_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_library(${PROJECT_NAME} STATIC ${GEOGRAPHIC_SOURCES} ${RTKFUSION_SOURCES})

