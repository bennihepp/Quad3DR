#find_package(GTest REQUIRED)

#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test_ann
    # Executable
    test_ann.cpp
)
target_link_libraries(test_ann
    #${GTEST_LIBRARIES}
    gtest
    gtest_main
)
