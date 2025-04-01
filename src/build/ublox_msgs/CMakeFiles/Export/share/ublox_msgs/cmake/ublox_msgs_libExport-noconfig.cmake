#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ublox_msgs::ublox_msgs_lib" for configuration ""
set_property(TARGET ublox_msgs::ublox_msgs_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ublox_msgs::ublox_msgs_lib PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libublox_msgs_lib.so"
  IMPORTED_SONAME_NOCONFIG "libublox_msgs_lib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ublox_msgs::ublox_msgs_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_ublox_msgs::ublox_msgs_lib "${_IMPORT_PREFIX}/lib/libublox_msgs_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
