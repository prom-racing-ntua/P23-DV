#!/bin/bash
# This script runs as soon as the PC powers on. The startup happens via a system daemon call (service call).

# 1. Limit CPU (TO BE TESTED! MAYBE NOT NEEDED)
limitCPU

# 2. Launch ROS2 Nodes
launchP23nodes
sleep 5
launchP23base