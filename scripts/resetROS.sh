#!/bin/bash

# Kill everything that has "ros" in its name
pkill -f "ros"
# Reset the ROS2 daemon
ros2 daemon stop