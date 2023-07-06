#!/bin/bash

# This script launches all the lifecycle nodes:
# 1. Velocity Estimation
# 2. Acquisition Left/Right
# 3. Inference
# 4. SLAM
# 5. Path Planning
# 6. MPC
# 7. PID_PP

cd /home/prom/P23-DV/ROS_Workspace
. install/setup.bash
ros2 launch p23_status lifecycle_nodes.launch.py &

# & indicates that this command starts as a new process and will not block this script, meaning that it will end here
