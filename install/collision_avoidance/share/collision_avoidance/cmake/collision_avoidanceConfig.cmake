# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_collision_avoidance_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED collision_avoidance_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(collision_avoidance_FOUND FALSE)
  elseif(NOT collision_avoidance_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(collision_avoidance_FOUND FALSE)
  endif()
  return()
endif()
set(_collision_avoidance_CONFIG_INCLUDED TRUE)

# output package information
if(NOT collision_avoidance_FIND_QUIETLY)
  message(STATUS "Found collision_avoidance: 0.1.0 (${collision_avoidance_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'collision_avoidance' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT collision_avoidance_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(collision_avoidance_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${collision_avoidance_DIR}/${_extra}")
endforeach()
