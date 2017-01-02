cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

project(Quad3DR)

include("cmake/CheckC11.cmake")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/;${CMAKE_MODULE_PATH}")
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# Set compiler warning flags
if(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_C_FLAGS "-Wall ${CMAKE_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
endif()
if ( MSVC )
    # This is untested
    set_property( TARGET main APPEND_STRING PROPERTY COMPILE_FLAGS /W3 )
endif ( MSVC )

find_package(Threads REQUIRED)
if (Threads_FOUND)
  link_libraries (${CMAKE_THREAD_LIBS_INIT})
endif (Threads_FOUND)

set(WITH_OPENMP True CACHE BOOL "Use OpenMP")
set(WITH_ZED True CACHE BOOL "Use ZED SDK")
set(USE_OPENCV_VERSION "3.1" CACHE STRING "OpenCV version")
set(WITH_PROFILING True CACHE BOOL "Enable profiling output")
# TODO
set(PLANESWEEPLIB_INCLUDE_DIRS "$ENV{HOME}/Code/PlaneSweepLib/src/PSL" CACHE STRING "PlaneSweepLib include directories")
set(PLANESWEEPLIB_LIBRARIES "$ENV{HOME}/Code/workspace/PlaneSweepLib/lib/libPSL.so" CACHE STRING "PlaneSweepLib libraries")
# Default library variables
set(EIGEN3_INCLUDE_DIR "" CACHE PATH "Manual Eigen3 include dir")

set(WITH_ROS False CACHE BOOL "Enable ROS support")
set(ROS_INCLUDE_DIRS "" CACHE PATH "ROS include directories")
set(ROS_LIBRARY_DIR "" CACHE FILEPATH "ROS library directory")

option(BUILD_VIDEO "Build video project" ON)
option(BUILD_STEREO "Build stereo project" ON)
option(BUILD_PLANE_SWEEP_STEREO "Build plane sweep stereo project" ON)
option(BUILD_QUAD_PLANNER "Build quadrotor planner project" ON)
option(BUILD_VIEWPOINT_PLANNER "Build viewpoint planner project" ON)

find_package(OpenCV ${USE_OPENCV_VERSION} REQUIRED)
if("${EIGEN3_INCLUDE_DIR}" STREQUAL "")
  unset(EIGEN3_INCLUDE_DIR CACHE)
  find_package(Eigen3 REQUIRED)
endif()

#set(Boost_DEBUG ON)
if(WIN32)
  set(Boost_USE_STATIC_LIBS       ON)
  set(Boost_USE_MULTITHREADED      ON)
  find_package(Boost REQUIRED COMPONENTS system filesystem program_options date_time regex serialization)
else()
  if(WITH_ROS)
    find_package(Boost REQUIRED COMPONENTS system filesystem program_options thread serialization)
  else()
    find_package(Boost REQUIRED COMPONENTS system filesystem program_options serialization)
  endif()
endif()

if(!WIN32)
  find_package(ZLIB REQUIRED)
endif()

find_package(TCLAP REQUIRED)

if(WITH_OPENMP)
	find_package(OpenMP REQUIRED)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_C_FLAGS}")
endif()

if("${WITH_ZED}")
  # ZED SDK
  if(WIN32) # Windows
    SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
    if (CMAKE_CL_64) # 64 bits
      SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
    else(CMAKE_CL_64) # 32 bits
      message("32bits compilation is no more available with CUDA7.0")
    endif(CMAKE_CL_64)
    SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
    SET(ZED_LIBRARIES "${ZED_LIBRARY_DIR}/${ZED_LIBRARIES}")
    message("ZED_LIBRARIES: ${ZED_LIBRARIES}")
    SET(ZED_FOUND True)
    SET(OPENCV_DIR $ENV{OPENCV_DIR})
    SET(Eigen_INCLUDE_DIRS $ENV{EIGEN_DIR})
    find_package(OpenCV ${USE_OPENCV_VERSION})
  else() # Linux
    find_package(ZED 1.0 REQUIRED)
  endif()
else()
  SET(ZED_FOUND False)
endif()

if("${WITH_ZED}" AND NOT "${ZED_FOUND}")
  error("Unable to find ZED SDK")
endif()

if("${WITH_ZED}")
  find_package(CUDA REQUIRED)
endif()

# Set profiling preprocessor macro
if(${WITH_PROFILING})
    add_definitions(-DWITH_PROFILING=1)
    message("With profiling")
else()
    message("Without profiling")
endif()

# Generate OpenCV preprocessor macro
if("${OpenCV_VERSION_MAJOR}" EQUAL "3")
    add_definitions(-DOPENCV_3=1)
    if ("${OpenCV_VERSION_MINOR}" EQUAL "1")
        message("Defining OPENCV_3_1 macro")
        add_definitions(-DOPENCV_3_1=1)
    else()
        message("Defining OPENCV_3_0 macro")
        add_definitions(-DOPENCV_3_0=1)
    endif()
elseif("${OpenCV_VERSION_MAJOR}" EQUAL "2")
    add_definitions(-DOPENCV_2=1)
    if ("${OpenCV_VERSION_MINOR}" EQUAL "4")
        add_definitions(-DOPENCV_2_4=1)
        message("Defining OPENCV_2_4 macro")
    else()
        message("Defining OPENCV_2 macro")
    endif()
endif()

# Check for ROS includes and libraries
if(WITH_ROS)
	find_file(ROS_H_FOUND ros/ros.h PATHS ${ROS_INCLUDE_DIRS})
	if(NOT ROS_H_FOUND)
		message(FATAL_ERROR "Unable to find ROS headers")
	endif()
	find_file(ROS_CPPLIB_FOUND libroscpp.so PATHS ${ROS_LIBRARY_DIR})
	if(NOT ROS_CPPLIB_FOUND)
		message(FATAL_ERROR "Unable to find ROS libraries")
	endif()
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/libroscpp.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/libroscpp_serialization.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/librostime.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/librosconsole.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/libactionlib.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/libimage_transport.so")
	list(APPEND ROS_LIBRARIES "${ROS_LIBRARY_DIR}/libcv_bridge.so")
endif()

include_directories(
		include/
		external/
    video/include/
    stereo/include/
    ${OpenCV_INCLUDE_DIRS}
    ${TCLAP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ../mLib/include
    ../mLib/src
)

if(WIN32) # Windows
  link_directories("../mLibExternal/libsWindows/lib64")
endif()

if("${WITH_ZED}")
  include_directories(
      ${ZED_INCLUDE_DIRS}
      ${CUDA_INCLUDE_DIRS}
  )
endif()

if(BUILD_VIDEO)
  add_subdirectory(video)
endif()

if(BUILD_STEREO)
  add_subdirectory(stereo)
endif()

if(BUILD_PLANE_SWEEP_STEREO)
  add_subdirectory(plane_sweep_stereo)
endif()

if(BUILD_QUAD_PLANNER)
  add_subdirectory(quad_planner)
endif()

if(BUILD_VIEWPOINT_PLANNER)
  add_subdirectory(viewpoint_planner)
endif()
