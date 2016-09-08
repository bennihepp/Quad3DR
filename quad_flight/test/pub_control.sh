#!/bin/bash

rostopic pub /quadrotor/gimbal_yaw_controller/command std_msgs/Float64 "{data: ${1}}"
rostopic pub /quadrotor/gimbal_pitch_controller/command std_msgs/Float64 "{data: ${2}}"

