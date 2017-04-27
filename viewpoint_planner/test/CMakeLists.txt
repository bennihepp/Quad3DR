#find_package(GTest REQUIRED)

#include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(test_ann
    # Executable
    test_ann.cpp
)
target_link_libraries(test_ann
    #${GTEST_LIBRARIES}
    gtest
    gtest_main
)

add_executable(test_vision
        # Executable
        test_vision.cpp
        )
target_link_libraries(test_vision
        #${GTEST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        gtest
        gtest_main
        )

add_executable(test_vision_qt
        # Executable
        test_vision_qt.cpp
        )
target_link_libraries(test_vision_qt
        #${GTEST_LIBRARIES}
        ${OpenCV_LIBRARIES}
        gtest
        gtest_main
        )
target_link_libraries(test_vision_qt Qt5::Core Qt5::Gui)

add_executable(test_se3_transform
        # Executable
        test_se3_transform.cpp
        )
target_link_libraries(test_se3_transform
        #${GTEST_LIBRARIES}
        gtest
        gtest_main
        )

add_executable(test_qt_image
        # Executable
        test_qt_image.cpp
        )
target_link_libraries(test_qt_image
        #${GTEST_LIBRARIES}
        gtest
        gtest_main
        )
target_link_libraries(test_qt_image Qt5::Core Qt5::Gui)
