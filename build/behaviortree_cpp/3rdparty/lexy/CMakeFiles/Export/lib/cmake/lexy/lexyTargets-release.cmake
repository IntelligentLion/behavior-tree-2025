#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "foonathan::lexy::file" for configuration "Release"
set_property(TARGET foonathan::lexy::file APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(foonathan::lexy::file PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblexy_file.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS foonathan::lexy::file )
list(APPEND _IMPORT_CHECK_FILES_FOR_foonathan::lexy::file "${_IMPORT_PREFIX}/lib/liblexy_file.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
