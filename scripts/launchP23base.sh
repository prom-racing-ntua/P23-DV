#!/bin/bash

# This script launches the base nodes:
# 1. P23 Status
# 2. Lifecycle Manager
# 3. Vectornav
# 4. CAN Module

cd /home/prom/P23-DV/ROS_Workspace
. install/setup.bash
ros2 launch p23_status p23base.launch.py &

# & indicates that this command starts as a new process and will not block this script, meaning that it will end here
