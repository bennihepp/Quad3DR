if(WIN32)
    find_package(GStreamerWin REQUIRED COMPONENTS gstreamer-app)
    link_directories($(Boost_LIBRARYDIR))
else()
    find_package(GStreamer REQUIRED COMPONENTS gstreamer-app)
endif()

include_directories(
    include/
    "${GSTREAMER_INCLUDE_DIRS}"
    "${GLIB_INCLUDE_DIRS}"
)

#include_directories(
#    include/
#    ${OpenCV_INCLUDE_DIRS}
#    ${TCLAP_INCLUDE_DIR}
#    "${GSTREAMER_INCLUDE_DIRS}"
#    ${ZED_INCLUDE_DIRS}
#    ${CUDA_INCLUDE_DIRS}
#    ${EIGEN3_INCLUDE_DIR}
#)

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

add_executable(video_streamer_bundlefusion
    src/video_streamer_bundlefusion.cpp
    src/video_source.cpp
    src/video_source_zed.cpp
    src/StereoNetworkSensorClient.cpp
    ../stereo/src/stereo_calibration.cpp
    ../src/mLib.cpp
)
target_link_libraries(video_streamer_bundlefusion
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZED_LIBRARIES}
    "${GSTREAMER_LIBRARIES}"
    "${GSTREAMER_APP_LIBRARIES}"
    ${GLIB_LIBRARIES}
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
    ${GLIB_LIBRARIES}
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
)
