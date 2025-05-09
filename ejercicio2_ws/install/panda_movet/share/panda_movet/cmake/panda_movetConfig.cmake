# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_panda_movet_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED panda_movet_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(panda_movet_FOUND FALSE)
  elseif(NOT panda_movet_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(panda_movet_FOUND FALSE)
  endif()
  return()
endif()
set(_panda_movet_CONFIG_INCLUDED TRUE)

# output package information
if(NOT panda_movet_FIND_QUIETLY)
  message(STATUS "Found panda_movet: 0.3.0 (${panda_movet_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'panda_movet' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${panda_movet_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(panda_movet_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${panda_movet_DIR}/${_extra}")
endforeach()
