# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_moonbot_gazebo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED moonbot_gazebo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(moonbot_gazebo_FOUND FALSE)
  elseif(NOT moonbot_gazebo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(moonbot_gazebo_FOUND FALSE)
  endif()
  return()
endif()
set(_moonbot_gazebo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT moonbot_gazebo_FIND_QUIETLY)
  message(STATUS "Found moonbot_gazebo: 0.0.0 (${moonbot_gazebo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'moonbot_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${moonbot_gazebo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(moonbot_gazebo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${moonbot_gazebo_DIR}/${_extra}")
endforeach()
