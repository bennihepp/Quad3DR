include_directories(
    include/
)

add_executable(sparse_stereo_test
    src/sparse_stereo_test.cpp
    ../src/utilities.cpp
    src/stereo_calibration.cpp
)
target_link_libraries(sparse_stereo_test
    ${OpenCV_LIBRARIES}
)

if("${WITH_ZED}")
  add_executable(sparse_stereo_zed
      src/sparse_stereo_zed.cpp
      ../src/utilities.cpp
      src/stereo_calibration.cpp
      ../video/src/video_source.cpp
      ../video/src/video_source_zed.cpp
      ../src/mLibSource.cpp
  )
  set_property(TARGET sparse_stereo_zed PROPERTY CXX_STANDARD 11)
  target_link_libraries(sparse_stereo_zed
      ${OpenCV_LIBRARIES}
      ${ZED_LIBRARIES}
      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  )

  add_executable(stereo_calibration_tool_zed
      src/stereo_calibration_tool_zed.cpp
      ../src/utilities.cpp
      src/stereo_calibration.cpp
      ../video/src/video_source.cpp
      ../video/src/video_source_zed.cpp
  )
  set_property(TARGET stereo_calibration_tool_zed PROPERTY CXX_STANDARD 11)
  target_link_libraries(stereo_calibration_tool_zed
      ${OpenCV_LIBRARIES}
      ${ZED_LIBRARIES}
      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  )

  add_executable(stereo_capture_zed
      src/stereo_capture_zed.cpp
      ../src/utilities.cpp
      src/stereo_calibration.cpp
      ../video/src/video_source.cpp
      ../video/src/video_source_zed.cpp
  )
  set_property(TARGET stereo_capture_zed PROPERTY CXX_STANDARD 11)
  target_link_libraries(stereo_capture_zed
      ${OpenCV_LIBRARIES}
      ${ZED_LIBRARIES}
      ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
  )
endif()
