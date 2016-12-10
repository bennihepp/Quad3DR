#!/bin/bash

pushd .
cd build_jetson || { echo "No such directory 'build_jetson"; exit 1; }

make -j4 video_streamer_bundlefusion_drone

popd

