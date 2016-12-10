#!/bin/bash

pushd .
cd build || { echo "No such directory 'build'"; exit 1; }

#BUILD_TYPE=Debug
BUILD_TYPE=RelWithDebInfo

#OPENCV_VERSION=2.4
OPENCV_VERSION=3.1

#WITH_ZED=False
WITH_ZED=True

#CMAKE_SOURCE_DIR=${HOME}/Projects/Quad3DR/
CMAKE_SOURCE_DIR=..

rm -rf CMakeCache.txt CMakeFiles

cmake \
    -G"Eclipse CDT4 - Unix Makefiles" \
    -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0" \
    -DCMAKE_C_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0" \
    -DUSE_OPENCV_VERSION=$OPENCV_VERSION \
    -DWITH_ZED=$WITH_ZED \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE $CMAKE_SOURCE_DIR \
    -DBUILD_PLANE_SWEEP_STEREO=OFF \
    -DBUILD_STEREO=OFF \
    -DBUILD_QUAD_PLANNER=OFF \
    -DBUILD_VIDEO=OFF \
    -DBOOST_ROOT=$HOME/Projects/Libraries/boost_1_60_0/ \
    -Doctomap_DIR=$HOME/Projects/Libraries/octomap/lib/cmake/octomap \
    -Doctovis_DIR=$HOME/Projects/Libraries/octomap/lib/cmake/octovis \
    -DCMAKE_PREFIX_PATH=$HOME/Projects/Libraries/ompl \
    -DOMPL_PREFIX=$HOME/Projects/Libraries/ompl \
    $@

popd

