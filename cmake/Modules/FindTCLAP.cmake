# - Try to find TCLAP lib
#
#  TCLAP_FOUND - system has TCLAP lib
#  TCLAP_INCLUDE_DIR - the TCLAP include directory

# Copyright (c) 2015, Benjamin Hepp <benjamin.hepp@posteo.de>

macro(_tclap_check_path)

  if(EXISTS "${TCLAP_INCLUDE_DIR}/tclap/CmdLine.h")
    set(TCLAP_OK TRUE)
  endif()

  if(NOT TCLAP_OK)
    message(STATUS "TCLAP include path was specified but no CmdLine.h file was found: ${TCLAP_INCLUDE_DIR}")
  endif()

endmacro()

if(NOT TCLAP_INCLUDE_DIR)

  find_path(TCLAP_INCLUDE_DIR NAMES tclap/CmdLine.h
	PATHS
	${CMAKE_INSTALL_PREFIX}/include
	${KDE4_INCLUDE_DIR}
	#PATH_SUFFIXES tclap
  )

endif()

if(TCLAP_INCLUDE_DIR)
  _tclap_check_path()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TCLAP DEFAULT_MSG TCLAP_INCLUDE_DIR TCLAP_OK)

mark_as_advanced(TCLAP_INCLUDE_DIR)
