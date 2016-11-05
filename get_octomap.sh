#!/bin/bash

rosservice call /world/get_octomap '{bounding_box_origin: {x: 0, y: 0, z: 0}, bounding_box_lengths: {x: 25.0, y: 25.0, z: 15}, leaf_size: 0.1, filename: gazebo_octomap.bt}'

