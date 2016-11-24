#!/bin/bash

pushd .
cd build_jetson || { echo "No such directory 'build_jetson'"; exit 1; }

ROS_DIR=/opt/ros/kinetic
DJI_DIR=$HOME/DJI
DJI_CATKIN_DIR=$DJI_DIR/catkin_ws_jetson

BUILD_TYPE=Release
#BUILD_TYPE=Debug

rm -r CMakeCache.txt CMakeFiles/
#cmake -DUSE_OPENCV_VERSION=2.4 ..
cmake \
	-DBUILD_PLANE_SWEEP_STEREO=OFF \
	-DBUILD_QUAD_PLANNER=OFF \
	-DBUILD_STEREO=OFF \
	-DWITH_ROS=ON \
	-DWITH_DJI=ON \
	-DROS_INCLUDE_DIRS=$ROS_DIR/include \
	-DROS_LIBRARY_DIR=$ROS_DIR/lib \
	-DDJI_LIBRARY=$DJI_CATKIN_DIR/devel/lib/libdji_sdk_lib.a \
	-DDJI_INCLUDE_DIRS="$DJI_CATKIN_DIR/src/dji_sdk/include;$DJI_CATKIN_DIR/src/dji_sdk_lib/include;$DJI_CATKIN_DIR/devel/include" \
	-DCUDA_USE_STATIC_CUDA_RUNTIME=OFF \
	-DUSE_OPENCV_VERSION=2.4 \
	-DCMAKE_BUILD_TYPE=$BUILD_TYPE \
	.. $@

popd

