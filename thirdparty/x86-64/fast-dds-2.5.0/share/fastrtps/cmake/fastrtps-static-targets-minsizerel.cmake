#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fastrtps" for configuration "MinSizeRel"
set_property(TARGET fastrtps APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(fastrtps PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "C;CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/libfastrtps.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS fastrtps )
list(APPEND _IMPORT_CHECK_FILES_FOR_fastrtps "${_IMPORT_PREFIX}/lib/libfastrtps.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
