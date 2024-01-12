# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Get_Camera_Img_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Get_Camera_Img_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Get_Camera_Img_FOUND FALSE)
  elseif(NOT Get_Camera_Img_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Get_Camera_Img_FOUND FALSE)
  endif()
  return()
endif()
set(_Get_Camera_Img_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Get_Camera_Img_FIND_QUIETLY)
  message(STATUS "Found Get_Camera_Img: 0.0.0 (${Get_Camera_Img_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Get_Camera_Img' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${Get_Camera_Img_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Get_Camera_Img_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Get_Camera_Img_DIR}/${_extra}")
endforeach()
