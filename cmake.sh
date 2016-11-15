#!/bin/bash

pushd .
cd build || { echo "No such directory 'build'"; exit 1; }

BUILD_TYPE=Debug
#BUILD_TYPE=RelWithDebInfo

#OPENCV_VERSION=2.4
OPENCV_VERSION=3.1

#WITH_ZED=False
WITH_ZED=True

#CMAKE_SOURCE_DIR=${HOME}/Projects/Quad3DR/
CMAKE_SOURCE_DIR=..

rm -rf CMakeCache.txt CMakeFiles

cmake -G"Eclipse CDT4 - Unix Makefiles" -DUSE_OPENCV_VERSION=$OPENCV_VERSION -DWITH_ZED=$WITH_ZED -DCMAKE_BUILD_TYPE=$BUILD_TYPE $CMAKE_SOURCE_DIR $@

popd

