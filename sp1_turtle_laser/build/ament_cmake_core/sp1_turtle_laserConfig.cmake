# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sp1_turtle_laser_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sp1_turtle_laser_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sp1_turtle_laser_FOUND FALSE)
  elseif(NOT sp1_turtle_laser_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sp1_turtle_laser_FOUND FALSE)
  endif()
  return()
endif()
set(_sp1_turtle_laser_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sp1_turtle_laser_FIND_QUIETLY)
  message(STATUS "Found sp1_turtle_laser: 0.0.0 (${sp1_turtle_laser_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sp1_turtle_laser' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sp1_turtle_laser_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sp1_turtle_laser_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sp1_turtle_laser_DIR}/${_extra}")
endforeach()
