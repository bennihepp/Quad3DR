cmake_minimum_required(VERSION 2.8)

include("cmake/CheckC11.cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(OPENCV_VERSION "3.1" CACHE STRING "OpenCV version")
# ZED SDK
IF(WIN32) # Windows
  SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
  if (CMAKE_CL_64) # 64 bits
    SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
  else(CMAKE_CL_64) # 32 bits
    message("32bits compilation is no more available with CUDA7.0")
  endif(CMAKE_CL_64)
  SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
  SET(OPENCV_DIR $ENV{OPENCV_DIR})
  SET(Eigen_INCLUDE_DIRS $ENV{EIGEN_DIR})
  find_package(OpenCV ${OPENCV_VERSION})
ELSE() # Linux
  find_package(ZED 1.0 REQUIRED)
  find_package(Eigen3 REQUIRED)

  ##For Jetson, OpenCV4Tegra is based on OpenCV2.4
  #exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
  #if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
  #  find_package(OpenCV ${OPENCV_VERSION})
  #else()
  #  find_package(OpenCV ${OPENCV_VERSION})
  #endif()
  find_package(OpenCV ${OPENCV_VERSION} REQUIRED)
ENDIF()

find_package(CUDA REQUIRED)
find_package(TCLAP REQUIRED)
find_package(GStreamer REQUIRED COMPONENTS gstreamer-app)


include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
    ${TCLAP_INCLUDE_DIR}
    "${GSTREAMER_INCLUDE_DIRS}"
    ${ZED_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_executable(video_capture
    src/video_capture.cpp
    src/video_source.cpp
    src/video_source_opencv.cpp
)
target_link_libraries(video_capture
    ${OpenCV_LIBRARIES}
)

add_executable(video_capture_zed
    src/video_capture_zed.cpp
    src/video_source.cpp
    src/video_source_zed.cpp
)
target_link_libraries(video_capture_zed
    ${OpenCV_LIBRARIES}
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
)

add_executable(video_streamer
    src/video_streamer.cpp
    src/video_source.cpp
    src/video_source_opencv.cpp
    src/video_source_zed.cpp
)
target_link_libraries(video_streamer
    ${OpenCV_LIBRARIES}
    "${GSTREAMER_LIBRARIES}"
    "${GSTREAMER_APP_LIBRARIES}"
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
)
