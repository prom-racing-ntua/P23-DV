#!/bin/bash
# This script runs as soon as the PC powers on. The startup happens via a system daemon call (service call).

# 1. Limit CPU (TO BE TESTED! MAYBE NOT NEEDED)
limitCPU

# 2. Change RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 3. Launch ROS2 Nodes
launchP23nodes
sleep 5
launchP23base

