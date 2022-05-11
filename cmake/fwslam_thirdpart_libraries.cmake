# Macro to find all Fw SLAM thirdparty libraries expect for Fw SLAM.
#
# Arguments:
#   :package: The name of the package to find. Used for find_package(${package})
#   :thirdparty_name: The name of the package directory under thirdparty, i.e. thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}
#   :VERSION: [Optional] The minimum required version of the package.
#
# The macro's procedure is as follows:
#   1. Try to find the package with by check if the include/lib in thirdparty.
#   2. If the package is not found in 1)
#         2.1. If THIRDPARTY_UPDATE is set to ON, then update the corresponding git submodule.
#         2.2. Append the thirdparty source directory to CMAKE_PREFIX_PATH.
#         2.3. Try to find the package again.
#   3. If the package was not found anywhere, then print an FATAL_ERROR message.
macro(fwslam_find_thirdparty package thirdparty_name)
    set(oneValueArgs VERSION)
    cmake_parse_arguments(FIND "" "${oneValueArgs}" "" ${ARGN})
    set(ALLOWED_VALUES ON OFF FORCE)

    # set(THIRDPARTY OFF CACHE STRING "Activate use of internal submodules.")
    # set_property(CACHE THIRDPARTY PROPERTY STRINGS ON OFF FORCE)
    # if(NOT THIRDPARTY IN_LIST ALLOWED_VALUES)
    #     message(FATAL_ERROR, "Wrong configuration of THIRDPARTY. Allowed values: ${ALLOWED_VALUES}")
    # endif()

    # set(THIRDPARTY_${package} ${THIRDPARTY} CACHE STRING "Activate use of internal submodule ${package}.")
    # set_property(CACHE THIRDPARTY_${package} PROPERTY STRINGS ON OFF FORCE)
    # if(NOT THIRDPARTY_${package} IN_LIST ALLOWED_VALUES)
    #     message(FATAL_ERROR, "Wrong configuration of THIRDPARTY_${package}. Allowed values: ${ALLOWED_VALUES}")
    # endif()

    # Keep for future feature 
    # option(THIRDPARTY_UPDATE "Activate the auto update of internal thirdparties" ON)

    # Keep for future feature 
    # If THIRDPARTY_${package} is set to FORCE, don't try to find the library outside thirdparty.
    # if(NOT (THIRDPARTY_${package} STREQUAL "FORCE"))
    #     # Try to quietly find the package outside thridparty first.
    #     find_package(${package} QUIET)

    #     # Show message if package is found here.
    #     if(${package}_FOUND)
    #         # Cannot state where the package is. Asio sets Asio_DIR to Asio_DIR-NOTFOUND even when found.
    #         message(STATUS "Found ${package}")
    #     endif()
    # endif()

    # Use thirdparty if THIRDPARTY_${package} is set to FORCE, or if the package is not found elsewhere and
    # THIRDPARTY_${package} is set to ON.
    set(THIRDPARTY_${package} ON)
    if((THIRDPARTY_${package} STREQUAL "FORCE") OR ((NOT ${package}_FOUND) AND (THIRDPARTY_${package} STREQUAL "ON")))
        # Keep for future feature 
        # if(THIRDPARTY_UPDATE)
        #     message(STATUS "Updating submodule thirdparty/${thirdparty_name}")
        #     execute_process(
        #         COMMAND git submodule update --recursive --init "thirdparty/${thirdparty_name}"
        #         WORKING_DIRECTORY ${FW_SLAM_PROJECT_TOP_DIR}
        #         RESULT_VARIABLE EXECUTE_RESULT
        #         )
        #     if(NOT EXECUTE_RESULT EQUAL 0)
        #         message(FATAL_ERROR "Cannot configure Git submodule ${package}")
        #     endif()
        # endif
        # Check that the package is correctly initialized by looking for thirdpart libraries folder extracted.
        set(THIRDPARTY_SUBDIRECTORY_EXIST FALSE)
        if(EXISTS "${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}")
            set(THIRDPARTY_SUBDIRECTORY_EXIST TRUE)
        endif()

        # If the thirdpart subdirectory not exist, extract the ${BUILD_TARGET_CPU_TYPE}.tar.gz
        if(NOT ${THIRDPARTY_SUBDIRECTORY_EXIST})
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar -xvf "${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}.tar.gz"
                WORKING_DIRECTORY "${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/"
                RESULT_VARIABLE EXECUTE_RESULT
                )
            message("thirdpart library extract result:" ${EXECUTE_RESULT})
        endif()

        if (FIND_VERSION)
            set(VERSION_STRING "-${FIND_VERSION}")
        endif ()

        # First check include and lib is exist, if exist, just add the varible to the enviroment
        if(EXISTS "${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/include")
            set(${thirdparty_name}_INCLUDE_DIR ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/include)
            include_directories(${${thirdparty_name}_INCLUDE_DIR})
            set(THIRDPARTY_PACKAGE_FOUND TRUE)
        endif()
        if(EXISTS "${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/lib")
            set(${thirdparty_name}_LIBRARY_DIR ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/lib)
            link_directories(${${thirdparty_name}_LIBRARY_DIR})
            set(THIRDPARTY_PACKAGE_FOUND TRUE)
        endif()

        if (NOT ${THIRDPARTY_PACKAGE_FOUND})
            # Prepend CMAKE_PREFIX_PATH with the package thirdparty directory. The third path is needed for lib/cmake/${thirdparty_name}, 
            # lib/${thirdparty_name}/cmake, since the find package cmake maybe in different folder
            set(CMAKE_PREFIX_PATH ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING} ${CMAKE_PREFIX_PATH})
            set(CMAKE_PREFIX_PATH ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/lib/cmake ${CMAKE_PREFIX_PATH})
            set(CMAKE_PREFIX_PATH ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/lib/cmake/${thirdparty_name} ${CMAKE_PREFIX_PATH})
            set(CMAKE_PREFIX_PATH ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}${VERSION_STRING}/lib/${thirdparty_name}/cmake ${CMAKE_PREFIX_PATH})
            set(CMAKE_PREFIX_PATH ${FW_SLAM_PROJECT_TOP_DIR}/thirdparty/${BUILD_TARGET_CPU_TYPE}/${thirdparty_name}/${thirdparty_name} ${CMAKE_PREFIX_PATH})
            find_package(${package})
        endif ()
    endif()
    # If the package was not found throw an error.
    if((NOT ${THIRDPARTY_PACKAGE_FOUND}) AND (NOT ${package}_FOUND))
        message(FATAL_ERROR "Cannot find package ${package}")
    endif()
endmacro()