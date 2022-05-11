cmake_minimum_required(VERSION 3.10)

project(cartographer)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})
option(BUILD_GRPC "build Cartographer gRPC support" false)
set(CARTOGRAPHER_HAS_GRPC ${BUILD_GRPC})
option(BUILD_PROMETHEUS "build Prometheus monitoring support" false)

add_compile_options(-fPIC)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
${PROJECT_SOURCE_DIR}/cmake/modules)

set(PROTOC_EXECUTABLE "${FW_SLAM_PROJECT_TOP_DIR}/tools/protoc/bin/protoc")
set(PROTOC_LIBRARY_DIR "${FW_SLAM_PROJECT_TOP_DIR}/tools/protoc/lib")

# Install catkin package.xml
#install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

# install(DIRECTORY configuration_files DESTINATION share/cartographer/)

# install(DIRECTORY cmake DESTINATION share/cartographer/)

file(GLOB_RECURSE ALL_LIBRARY_HDRS "cartographer/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "cartographer/*.cc")
file(GLOB_RECURSE TEST_LIBRARY_HDRS "cartographer/fake_*.h" "cartographer/*test_helpers*.h" "cartographer/mock_*.h")
file(GLOB_RECURSE TEST_LIBRARY_SRCS "cartographer/fake_*.cc" "cartographer/*test_helpers*.cc" "cartographer/mock_*.cc")
file(GLOB_RECURSE ALL_TESTS "cartographer/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer/*_main.cc")

# Remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_DOTFILES})
endif()
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_EXECUTABLES})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_LIBRARY_HDRS ${TEST_LIBRARY_HDRS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_LIBRARY_SRCS})
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer/cloud/*")
file(GLOB_RECURSE ALL_PROMETHEUS_FILES "cartographer/cloud/metrics/prometheus/*")
list(REMOVE_ITEM ALL_GRPC_FILES ${ALL_PROMETHEUS_FILES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
if (NOT ${BUILD_PROMETHEUS})
  list(REMOVE_ITEM ALL_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_HDRS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM TEST_LIBRARY_SRCS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_PROMETHEUS_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_PROMETHEUS_FILES})
endif()
set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS} ${TEST_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "cartographer/*/internal/*.h")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})

file(GLOB_RECURSE ALL_PROTOS "cartographer/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "cartographer/*_service.proto")
list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_PROTOS ${ALL_GRPC_FILES})
endif()

# TODO(cschuet): Move proto compilation to separate function.
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  set(GENERATE_PB "${PROTOC_EXECUTABLE} -I ${PROJECT_SOURCE_DIR} --cpp_out=${PROJECT_BINARY_DIR} ${ABS_FIL}")
  EXEC_PROGRAM("export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PROTOC_LIBRARY_DIR};cd ${PROJECT_BINARY_DIR};${GENERATE_PB}" OUTPUT_VARIABLE QUIT_PRINT)
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

if(${BUILD_GRPC})
  set(ALL_GRPC_SERVICE_SRCS)
  set(ALL_GRPC_SERVICE_HDRS)
  foreach(ABS_FIL ${ALL_GRPC_SERVICES})
    file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
    get_filename_component(DIR ${REL_FIL} DIRECTORY)
    get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

    list(APPEND ALL_GRPC_SERVICE_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
    list(APPEND ALL_GRPC_SERVICE_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
    set(GENERATE_PB "${PROTOC_EXECUTABLE} -I ${PROJECT_SOURCE_DIR} --cpp_out=${PROJECT_BINARY_DIR} ${ABS_FIL}")
    EXEC_PROGRAM("export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${PROTOC_LIBRARY_DIR};cd ${PROJECT_BINARY_DIR};${GENERATE_PB}" OUTPUT_VARIABLE QUIT_PRINT)
  endforeach()
  set_source_files_properties(${ALL_GRPC_SERVICE_SRCS} ${ALL_GRPC_SERVICE_HDRS} PROPERTIES GENERATED TRUE)
  list(APPEND ALL_LIBRARY_HDRS ${ALL_GRPC_SERVICE_HDRS})
  list(APPEND ALL_LIBRARY_SRCS ${ALL_GRPC_SERVICE_SRCS})
endif()
set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS} ${ALL_GRPC_SERVICE_HDRS})

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS})

configure_file(
  ${PROJECT_SOURCE_DIR}/cartographer/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/cartographer/common/config.h)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  # install(
  #   FILES ${HDR}
  #   DESTINATION include/${DIR}
  # )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  # install(
  #   FILES ${HDR}
  #   DESTINATION include/${DIR}
  # )
endforeach()

# set(CARTOGRAPHER_CMAKE_DIR share/cartographer/cmake)
# include(CMakePackageConfigHelpers)
# configure_package_config_file(
#   cartographer-config.cmake.in
#   ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
#   PATH_VARS CARTOGRAPHER_CMAKE_DIR
#   INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
# )

file(GLOB_RECURSE ALL_CONFIGURATION_FILES "${PROJECT_SOURCE_DIR}/configuration_files/*")
install(
  FILES ${ALL_CONFIGURATION_FILES}
  DESTINATION ./config
)

# install(
#   FILES ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
#   DESTINATION share/cartographer/
# )
