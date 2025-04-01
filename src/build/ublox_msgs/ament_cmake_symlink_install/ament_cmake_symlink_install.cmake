# generated from
# ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install.cmake.in

# create empty symlink install manifest before starting install step
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt")

#
# Reimplement CMake install(DIRECTORY) command to use symlinks instead of
# copying resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_directory cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "DIRECTORY;PATTERN;PATTERN_EXCLUDE" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_directory() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/jun/ros2_ws/src/install/ublox_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # default pattern to include
  if(NOT ARG_PATTERN)
    set(ARG_PATTERN "*")
  endif()

  # iterate over directories
  foreach(dir ${ARG_DIRECTORY})
    # make dir an absolute path
    if(NOT IS_ABSOLUTE "${dir}")
      set(dir "${cmake_current_source_dir}/${dir}")
    endif()

    if(EXISTS "${dir}")
      # if directory has no trailing slash
      # append folder name to destination
      set(destination "${ARG_DESTINATION}")
      string(LENGTH "${dir}" length)
      math(EXPR offset "${length} - 1")
      string(SUBSTRING "${dir}" ${offset} 1 dir_last_char)
      if(NOT dir_last_char STREQUAL "/")
        get_filename_component(destination_name "${dir}" NAME)
        set(destination "${destination}/${destination_name}")
      else()
        # remove trailing slash
        string(SUBSTRING "${dir}" 0 ${offset} dir)
      endif()

      # glob recursive files
      set(relative_files "")
      foreach(pattern ${ARG_PATTERN})
        file(
          GLOB_RECURSE
          include_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT include_files STREQUAL "")
          list(APPEND relative_files ${include_files})
        endif()
      endforeach()
      foreach(pattern ${ARG_PATTERN_EXCLUDE})
        file(
          GLOB_RECURSE
          exclude_files
          RELATIVE "${dir}"
          "${dir}/${pattern}"
        )
        if(NOT exclude_files STREQUAL "")
          list(REMOVE_ITEM relative_files ${exclude_files})
        endif()
      endforeach()
      list(SORT relative_files)

      foreach(relative_file ${relative_files})
        set(absolute_file "${dir}/${relative_file}")
        # determine link name for file including destination path
        set(symlink "${destination}/${relative_file}")

        # ensure that destination exists
        get_filename_component(symlink_dir "${symlink}" PATH)
        if(NOT EXISTS "${symlink_dir}")
          file(MAKE_DIRECTORY "${symlink_dir}")
        endif()

        _ament_cmake_symlink_install_create_symlink("${absolute_file}" "${symlink}")
      endforeach()
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_directory() can't find '${dir}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(FILES) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_files cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION;RENAME" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/jun/ros2_ws/src/install/ublox_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  if(ARG_RENAME)
    list(LENGTH ARG_FILES file_count)
    if(NOT file_count EQUAL 1)
    message(FATAL_ERROR "ament_cmake_symlink_install_files() called with "
      "RENAME argument but not with a single file")
    endif()
  endif()

  # iterate over files
  foreach(file ${ARG_FILES})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      if(NOT ARG_RENAME)
        set(symlink "${ARG_DESTINATION}/${filename}")
      else()
        set(symlink "${ARG_DESTINATION}/${ARG_RENAME}")
      endif()
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_files() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(PROGRAMS) command to use symlinks instead of copying
# resources.
#
# :param cmake_current_source_dir: The CMAKE_CURRENT_SOURCE_DIR when install
#   was invoked
# :type cmake_current_source_dir: string
# :param ARGN: the same arguments as the CMake install command.
# :type ARGN: various
#
function(ament_cmake_symlink_install_programs cmake_current_source_dir)
  cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "PROGRAMS" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_programs() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # make destination an absolute path and ensure that it exists
  if(NOT IS_ABSOLUTE "${ARG_DESTINATION}")
    set(ARG_DESTINATION "/home/jun/ros2_ws/src/install/ublox_msgs/${ARG_DESTINATION}")
  endif()
  if(NOT EXISTS "${ARG_DESTINATION}")
    file(MAKE_DIRECTORY "${ARG_DESTINATION}")
  endif()

  # iterate over programs
  foreach(file ${ARG_PROGRAMS})
    # make file an absolute path
    if(NOT IS_ABSOLUTE "${file}")
      set(file "${cmake_current_source_dir}/${file}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${ARG_DESTINATION}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_programs() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

#
# Reimplement CMake install(TARGETS) command to use symlinks instead of copying
# resources.
#
# :param TARGET_FILES: the absolute files, replacing the name of targets passed
#   in as TARGETS
# :type TARGET_FILES: list of files
# :param ARGN: the same arguments as the CMake install command except that
#   keywords identifying the kind of type and the DESTINATION keyword must be
#   joined with an underscore, e.g. ARCHIVE_DESTINATION.
# :type ARGN: various
#
function(ament_cmake_symlink_install_targets)
  cmake_parse_arguments(ARG "OPTIONAL" "ARCHIVE_DESTINATION;DESTINATION;LIBRARY_DESTINATION;RUNTIME_DESTINATION"
    "TARGETS;TARGET_FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "ament_cmake_symlink_install_targets() called with "
      "unused/unsupported arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  # iterate over target files
  foreach(file ${ARG_TARGET_FILES})
    if(NOT IS_ABSOLUTE "${file}")
      message(FATAL_ERROR "ament_cmake_symlink_install_targets() target file "
        "'${file}' must be an absolute path")
    endif()

    # determine destination of file based on extension
    set(destination "")
    get_filename_component(fileext "${file}" EXT)
    if(fileext STREQUAL ".a" OR fileext STREQUAL ".lib")
      set(destination "${ARG_ARCHIVE_DESTINATION}")
    elseif(fileext STREQUAL ".dylib" OR fileext MATCHES "\\.so(\\.[0-9]+)?(\\.[0-9]+)?(\\.[0-9]+)?$")
      set(destination "${ARG_LIBRARY_DESTINATION}")
    elseif(fileext STREQUAL "" OR fileext STREQUAL ".dll" OR fileext STREQUAL ".exe")
      set(destination "${ARG_RUNTIME_DESTINATION}")
    endif()
    if(destination STREQUAL "")
      set(destination "${ARG_DESTINATION}")
    endif()

    # make destination an absolute path and ensure that it exists
    if(NOT IS_ABSOLUTE "${destination}")
      set(destination "/home/jun/ros2_ws/src/install/ublox_msgs/${destination}")
    endif()
    if(NOT EXISTS "${destination}")
      file(MAKE_DIRECTORY "${destination}")
    endif()

    if(EXISTS "${file}")
      # determine link name for file including destination path
      get_filename_component(filename "${file}" NAME)
      set(symlink "${destination}/${filename}")
      _ament_cmake_symlink_install_create_symlink("${file}" "${symlink}")
    else()
      if(NOT ARG_OPTIONAL)
        message(FATAL_ERROR
          "ament_cmake_symlink_install_targets() can't find '${file}'")
      endif()
    endif()
  endforeach()
endfunction()

function(_ament_cmake_symlink_install_create_symlink absolute_file symlink)
  # register symlink for being removed during install step
  file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/symlink_install_manifest.txt"
    "${symlink}\n")

  # avoid any work if correct symlink is already in place
  if(EXISTS "${symlink}" AND IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    get_filename_component(real_absolute_file "${absolute_file}" REALPATH)
    if(destination STREQUAL real_absolute_file)
      message(STATUS "Up-to-date symlink: ${symlink}")
      return()
    endif()
  endif()

  message(STATUS "Symlinking: ${symlink}")
  if(EXISTS "${symlink}" OR IS_SYMLINK "${symlink}")
    file(REMOVE "${symlink}")
  endif()

  execute_process(
    COMMAND "/usr/bin/cmake" "-E" "create_symlink"
      "${absolute_file}"
      "${symlink}"
  )
  # the CMake command does not provide a return code so check manually
  if(NOT EXISTS "${symlink}" OR NOT IS_SYMLINK "${symlink}")
    get_filename_component(destination "${symlink}" REALPATH)
    message(FATAL_ERROR
      "Could not create symlink '${symlink}' pointing to '${absolute_file}'")
  endif()
endfunction()

# end of template

message(STATUS "Execute custom install script")

# begin of custom install code

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/rosidl_interfaces/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/rosidl_interfaces")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.h")

# install(FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/opt/ros/humble/lib/python3.10/site-packages/ament_package/template/environment_hook/library_path.sh" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/library_path.dsv" "DESTINATION" "share/ublox_msgs/environment")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_fastrtps_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_fastrtps_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_introspection_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.h")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_introspection_c/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.h")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.hpp")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_fastrtps_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN_EXCLUDE" "*.cpp")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_fastrtps_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN_EXCLUDE" "*.cpp")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_introspection_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.hpp")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_typesupport_introspection_cpp/ublox_msgs/" "DESTINATION" "include/ublox_msgs/ublox_msgs" "PATTERN" "*.hpp")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/pythonpath.sh" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/pythonpath.dsv" "DESTINATION" "share/ublox_msgs/environment")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_python/ublox_msgs/ublox_msgs.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs-2.3.0-py3.10.egg-info")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_python/ublox_msgs/ublox_msgs.egg-info/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs-2.3.0-py3.10.egg-info")

# install(DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_py/ublox_msgs/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_generator_py/ublox_msgs/" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs" "PATTERN_EXCLUDE" "*.pyc" "PATTERN_EXCLUDE" "__pycache__")

# install("TARGETS" "ublox_msgs__rosidl_typesupport_fastrtps_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs")
include("/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_symlink_install_targets_0_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "ublox_msgs__rosidl_typesupport_introspection_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs")
include("/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_symlink_install_targets_1_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install("TARGETS" "ublox_msgs__rosidl_typesupport_c__pyext" "DESTINATION" "local/lib/python3.10/dist-packages/ublox_msgs")
include("/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_symlink_install_targets_2_${CMAKE_INSTALL_CONFIG_NAME}.cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/Ack.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/Ack.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidALM.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidALM.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidEPH.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidEPH.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidHUI.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/AidHUI.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgANT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgANT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgCFG.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgCFG.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgDAT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgDAT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgDGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgDGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgGNSSBlock.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgGNSSBlock.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgHNR.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgHNR.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgINFBlock.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgINFBlock.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgINF.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgINF.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgMSG.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgMSG.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNAV5.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNAV5.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNAVX5.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNAVX5.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA6.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA6.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA7.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA7.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgNMEA.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgPRT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgPRT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgRATE.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgRATE.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgRST.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgRST.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgSBAS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgSBAS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgTMODE3.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgTMODE3.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgUSB.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgUSB.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALDEL.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALDEL.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALGET.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALGET.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALSET.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALSET.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALSETCfgdata.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/CfgVALSETCfgdata.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfINS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfINS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfMEAS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfMEAS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfRAWBlock.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfRAWBlock.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfRAW.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfRAW.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfSTATUS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfSTATUS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfSTATUSSens.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/EsfSTATUSSens.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/HnrPVT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/HnrPVT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/Inf.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/Inf.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MgaGAL.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MgaGAL.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonGNSS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonHW6.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonHW6.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonHW.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonHW.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonVERExtension.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonVERExtension.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonVER.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/MonVER.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavATT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavATT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavCLOCK.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavCLOCK.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavCOV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavCOV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDGPS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDGPS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDGPSSV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDGPSSV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDOP.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavDOP.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPOSECEF.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPOSECEF.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPOSLLH.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPOSLLH.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPVT7.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPVT7.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPVT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavPVT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavRELPOSNED9.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavRELPOSNED9.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavRELPOSNED.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavRELPOSNED.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSAT.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSAT.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSATSV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSATSV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSBAS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSBAS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSBASSV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSBASSV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSOL.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSOL.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSTATUS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSTATUS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVINFO.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVINFO.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVINFOSV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVINFOSV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVIN.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavSVIN.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavTIMEGPS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavTIMEGPS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavTIMEUTC.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavTIMEUTC.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavVELECEF.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavVELECEF.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavVELNED.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/NavVELNED.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmALM.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmALM.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmEPH.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmEPH.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAW.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAW.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWSV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWSV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWXMeas.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWXMeas.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWX.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRAWX.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRTCM.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmRTCM.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSFRB.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSFRB.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSFRBX.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSFRBX.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSVSI.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSVSI.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSVSISV.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/RxmSVSISV.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/TimTM2.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/TimTM2.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/UpdSOSAck.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/UpdSOSAck.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/UpdSOS.idl" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_adapter/ublox_msgs/msg/UpdSOS.idl" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/Ack.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/Ack.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidALM.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidALM.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidEPH.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidEPH.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidHUI.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/AidHUI.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgANT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgANT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgCFG.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgCFG.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgDAT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgDAT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgDGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgDGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgGNSSBlock.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgGNSSBlock.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgHNR.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgHNR.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgINFBlock.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgINFBlock.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgINF.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgINF.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgMSG.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgMSG.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNAV5.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNAV5.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNAVX5.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNAVX5.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA6.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA6.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA7.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA7.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgNMEA.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgPRT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgPRT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgRATE.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgRATE.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgRST.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgRST.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgSBAS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgSBAS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgTMODE3.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgTMODE3.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgUSB.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgUSB.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALDEL.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALDEL.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALGET.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALGET.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALSET.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALSET.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALSETCfgdata.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/CfgVALSETCfgdata.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfINS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfINS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfMEAS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfMEAS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfRAWBlock.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfRAWBlock.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfRAW.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfRAW.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfSTATUS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfSTATUS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfSTATUSSens.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/EsfSTATUSSens.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/HnrPVT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/HnrPVT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/Inf.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/Inf.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MgaGAL.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MgaGAL.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonGNSS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonHW6.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonHW6.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonHW.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonHW.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonVERExtension.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonVERExtension.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonVER.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/MonVER.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavATT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavATT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavCLOCK.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavCLOCK.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavCOV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavCOV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDGPS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDGPS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDGPSSV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDGPSSV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDOP.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavDOP.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPOSECEF.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPOSECEF.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPOSLLH.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPOSLLH.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPVT7.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPVT7.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPVT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavPVT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavRELPOSNED9.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavRELPOSNED9.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavRELPOSNED.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavRELPOSNED.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSAT.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSAT.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSATSV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSATSV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSBAS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSBAS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSBASSV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSBASSV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSOL.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSOL.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSTATUS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSTATUS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVINFO.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVINFO.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVINFOSV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVINFOSV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVIN.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavSVIN.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavTIMEGPS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavTIMEGPS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavTIMEUTC.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavTIMEUTC.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavVELECEF.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavVELECEF.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavVELNED.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/NavVELNED.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmALM.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmALM.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmEPH.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmEPH.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAW.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAW.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWSV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWSV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWXMeas.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWXMeas.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWX.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRAWX.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRTCM.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmRTCM.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSFRB.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSFRB.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSFRBX.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSFRBX.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSVSI.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSVSI.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSVSISV.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/RxmSVSISV.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/TimTM2.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/TimTM2.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/UpdSOSAck.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/UpdSOSAck.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/UpdSOS.msg" "DESTINATION" "share/ublox_msgs/msg")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/msg/UpdSOS.msg" "DESTINATION" "share/ublox_msgs/msg")

# install(DIRECTORY "include/" "DESTINATION" "include/ublox_msgs")
ament_cmake_symlink_install_directory("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" DIRECTORY "include/" "DESTINATION" "include/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/package_run_dependencies")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/parent_prefix_path")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/ament_prefix_path.dsv" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/ublox_msgs/environment")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/path.dsv" "DESTINATION" "share/ublox_msgs/environment")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.bash" "DESTINATION" "share/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.sh" "DESTINATION" "share/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.zsh" "DESTINATION" "share/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/local_setup.dsv" "DESTINATION" "share/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_environment_hooks/package.dsv" "DESTINATION" "share/ublox_msgs")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/packages")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_index/share/ament_index/resource_index/packages/ublox_msgs" "DESTINATION" "share/ament_index/resource_index/packages")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_targets-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/rosidl_cmake/rosidl_cmake_export_typesupport_libraries-extras.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_core/ublox_msgsConfig.cmake" "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_core/ublox_msgsConfig-version.cmake" "DESTINATION" "share/ublox_msgs/cmake")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_core/ublox_msgsConfig.cmake" "/home/jun/ros2_ws/src/build/ublox_msgs/ament_cmake_core/ublox_msgsConfig-version.cmake" "DESTINATION" "share/ublox_msgs/cmake")

# install(FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/package.xml" "DESTINATION" "share/ublox_msgs")
ament_cmake_symlink_install_files("/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs" FILES "/home/jun/ros2_ws/src/sensor_drivers/ublox-ros2/ublox_msgs/package.xml" "DESTINATION" "share/ublox_msgs")
