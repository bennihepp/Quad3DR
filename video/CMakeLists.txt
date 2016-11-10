if(WIN32)
    find_package(GStreamerWin REQUIRED COMPONENTS gstreamer-app)
    link_directories($(Boost_LIBRARYDIR))
else()
	find_package(ZLIB REQUIRED)
	link_libraries(${ZLIB_LIBRARIES})

    find_package(GStreamer REQUIRED COMPONENTS gstreamer-app)
endif()

include_directories(
    include/
    "${GSTREAMER_INCLUDE_DIRS}"
)
if (WIN32)
    include_directories(
        "${GLIB_INCLUDE_DIRS}"
    )
endif()

#message("Gstreamer: ${GSTREAMER_INCLUDE_DIRS}")

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

if(WITH_ZED)
	add_executable(video_capture_zed
	    src/video_capture_zed.cpp
	    src/video_source.cpp
	    src/video_source_zed.cpp
	    ../stereo/src/stereo_calibration.cpp
	)
	target_compile_definitions(video_capture_zed PUBLIC WITH_ZED=1)
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
	    ../stereo/src/stereo_calibration.cpp
	)
	target_compile_definitions(video_streamer PUBLIC WITH_ZED=1)
	target_link_libraries(video_streamer
	    ${OpenCV_LIBRARIES}
	    "${GSTREAMER_LIBRARIES}"
	    "${GSTREAMER_APP_LIBRARIES}"
	    ${GLIB_LIBRARIES}
	    ${ZED_LIBRARIES}
	    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
	)

	add_executable(video_streamer_bundlefusion
	    src/video_streamer_bundlefusion.cpp
	    src/video_source.cpp
	    src/video_source_zed.cpp
		src/GstMetaCorrespondence.cpp
	    ../stereo/src/stereo_calibration.cpp
	    ../src/common.cpp
	)
	target_compile_definitions(video_streamer_bundlefusion PUBLIC WITH_ZED=1)
	target_link_libraries(video_streamer_bundlefusion
	    ${Boost_LIBRARIES}
	    ${OpenCV_LIBRARIES}
	    ${ZED_LIBRARIES}
	    "${GSTREAMER_LIBRARIES}"
	    "${GSTREAMER_APP_LIBRARIES}"
	    ${GLIB_LIBRARIES}
	    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
	)

	add_executable(video_streamer_bundlefusion_quad
	    src/video_streamer_bundlefusion_quad.cpp
	    src/video_source.cpp
	    src/video_source_zed.cpp
		src/GstMetaCorrespondence.cpp
	    ../stereo/src/stereo_calibration.cpp
	    ../src/common.cpp
	)
	target_compile_definitions(video_streamer_bundlefusion_quad PUBLIC WITH_ZED=1)
	target_link_libraries(video_streamer_bundlefusion_quad
	    ${Boost_LIBRARIES}
	    ${OpenCV_LIBRARIES}
	    ${ZED_LIBRARIES}
	    "${GSTREAMER_LIBRARIES}"
	    "${GSTREAMER_APP_LIBRARIES}"
	    ${GLIB_LIBRARIES}
	    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
	)
endif()
