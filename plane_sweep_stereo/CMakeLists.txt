include_directories(
    include/
    ${PLANESWEEPLIB_INCLUDE_DIRS}
)

add_executable(plane_sweep_test
	src/plane_sweep_test.cpp
	../stereo/src/stereo_calibration.cpp
	../src/utilities.cpp
)
target_link_libraries(plane_sweep_test
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PLANESWEEPLIB_LIBRARIES}
)

add_executable(plane_sweep_zed
	src/plane_sweep_zed.cpp
	../stereo/src/stereo_calibration.cpp
	../src/utilities.cpp
  ../video/src/video_source.cpp
  ../video/src/video_source_zed.cpp
)
target_link_libraries(plane_sweep_zed
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PLANESWEEPLIB_LIBRARIES}
    ${ZED_LIBRARIES}
    ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
)

add_executable(plane_sweep_test_orig
	src/plane_sweep_test_orig.cpp
)
target_link_libraries(plane_sweep_test_orig
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PLANESWEEPLIB_LIBRARIES}
)
