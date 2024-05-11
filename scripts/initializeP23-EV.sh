#!/bin/bash

# This script runs as soon as the PC powers on. The startup happens via a system daemon call (service call).
wall Initializing P23 DV - EV mode

# 1. Change RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=~/.ros/cyclonedds.xml

scriptFolder="/home/prom/P23-DV/scripts"

# 3. Launch ROS2 Nodes
cd $scriptFolder
source setAliases.sh
cd /home/prom/P23-DV/ROS_Workspace
. install/setup.bash
ros2 launch can_reader can_interface.launch.py &
sleep 2
ros2 launch data_logger datalogger_new.launch.py &