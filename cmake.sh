#!/bin/bash

pushd .
cd build || { echo "No such directory 'build'"; exit 1; }

GENERATOR="Unix Makefiles"
#GENERATOR="Eclipse CDT4 - Unix Makefiles"

#BUILD_TYPE=Debug
BUILD_TYPE=RelWithDebInfo
#BUILD_TYPE=Release

#OPENCV_VERSION=2.4
OPENCV_VERSION=3.1

WITH_ZED=False
#WITH_ZED=True

#CMAKE_SOURCE_DIR=${HOME}/Projects/Quad3DR/
CMAKE_SOURCE_DIR=..

#CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0 -fno-strict-aliasing -march=native -msse2 -mavx"
CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0 -O2 -g3"
#CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0 -g -pg"
C_FLAGS="$CXX_FLAGS"

rm -rf CMakeCache.txt CMakeFiles

cmake \
    -G"$GENERATOR" \
    -DCMAKE_CXX_FLAGS="$CXX_FLAGS" \
    -DCMAKE_C_FLAGS="$C_FLAGS" \
    -DUSE_OPENCV_VERSION=$OPENCV_VERSION \
    -DWITH_ZED=$WITH_ZED \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DBUILD_PLANE_SWEEP_STEREO=OFF \
    -DBUILD_STEREO=OFF \
    -DBUILD_QUAD_PLANNER=OFF \
    -DBUILD_VIDEO=OFF \
    -DBOOST_ROOT=$HOME/Projects/Libraries/boost_1_60_0/ \
    -Doctomap_DIR=$HOME/Projects/Libraries/octomap/lib/cmake/octomap \
    -Doctovis_DIR=$HOME/Projects/Libraries/octomap/lib/cmake/octovis \
    -DCMAKE_PREFIX_PATH=$HOME/Projects/Libraries/ompl \
    -DOMPL_PREFIX=$HOME/Projects/Libraries/ompl \
    $CMAKE_SOURCE_DIR \
    $@

popd

