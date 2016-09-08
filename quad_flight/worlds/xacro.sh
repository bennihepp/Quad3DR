#!/bin/bash

rosrun xacro xacro --inorder buildings.world.xacro $@ > buildings.world
