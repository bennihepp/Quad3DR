#!/bin/bash

rm -rf CMakeCache.txt CMakeFiles

cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=3.1 -DCMAKE_BUILD_TYPE=Debug ../../stereo/ $@
#cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=3.1 -DCMAKE_BUILD_TYPE=RelWithDebInfo ../../stereo/ $@
#cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=3.1 -DCMAKE_BUILD_TYPE=Debug ../../stereo/ -DWITH_ZED=False $@
#cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=3.1 -DCMAKE_BUILD_TYPE=RelWithDebInfo ../../stereo/ -DWITH_ZED=False $@

#cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=2.4 -DCMAKE_BUILD_TYPE=Debug ../../stereo/ $@
#cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=2.4 -DCMAKE_BUILD_TYPE=RelWithDebInfo ../../stereo/ $@

