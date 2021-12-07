#!/bin/bash

source /opt/ros/foxy/setup.bash
source ../../install/local_setup.bash
ros2 launch pal_camera_description pal_camera_description.launch.py
