#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "foonathan_memory" for configuration "Release"
set_property(TARGET foonathan_memory APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(foonathan_memory PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfoonathan_memory-0.7.1.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS foonathan_memory )
list(APPEND _IMPORT_CHECK_FILES_FOR_foonathan_memory "${_IMPORT_PREFIX}/lib/libfoonathan_memory-0.7.1.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
