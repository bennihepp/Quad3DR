cmake_minimum_required(VERSION 2.8)

include("cmake/CheckC11.cmake")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/;${CMAKE_MODULE_PATH}")

set(WITH_ZED True CACHE BOOL "Use ZED SDK")
set(OPENCV_VERSION "3.1" CACHE STRING "OpenCV version")

find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TCLAP REQUIRED)

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
    SET(ZED_FOUND True)
    SET(OPENCV_DIR $ENV{OPENCV_DIR})
    SET(Eigen_INCLUDE_DIRS $ENV{EIGEN_DIR})
    find_package(OpenCV ${OPENCV_VERSION})
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


include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${TCLAP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(sparse_stereo_test
    src/sparse_stereo_test.cpp
    src/utilities.cpp
    src/stereo_calibration.cpp
)

target_link_libraries(sparse_stereo_test
    ${OpenCV_LIBRARIES}
)

if("${WITH_ZED}")
  include_directories(
      ../video/include
      ${ZED_INCLUDE_DIRS}
      ${CUDA_INCLUDE_DIRS}
  )

  add_executable(sparse_stereo_zed
      src/sparse_stereo_zed.cpp
      src/utilities.cpp
      src/stereo_calibration.cpp
      ../video/src/video_source.cpp
      ../video/src/video_source_zed.cpp
  )

  target_link_libraries(sparse_stereo_zed
      ${OpenCV_LIBRARIES}
      ${ZED_LIBRARIES}
      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  )

  add_executable(stereo_calibration_tool_zed
      src/stereo_calibration_tool_zed.cpp
      src/utilities.cpp
      ../video/src/video_source.cpp
      ../video/src/video_source_zed.cpp
  )

  target_link_libraries(stereo_calibration_tool_zed
      ${OpenCV_LIBRARIES}
      ${ZED_LIBRARIES}
      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  )
endif()
