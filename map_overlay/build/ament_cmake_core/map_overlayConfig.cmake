# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_map_overlay_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED map_overlay_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(map_overlay_FOUND FALSE)
  elseif(NOT map_overlay_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(map_overlay_FOUND FALSE)
  endif()
  return()
endif()
set(_map_overlay_CONFIG_INCLUDED TRUE)

# output package information
if(NOT map_overlay_FIND_QUIETLY)
  message(STATUS "Found map_overlay: 0.0.0 (${map_overlay_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'map_overlay' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${map_overlay_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(map_overlay_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${map_overlay_DIR}/${_extra}")
endforeach()
