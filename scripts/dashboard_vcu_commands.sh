#!/bin/bash
cd ~/P23-DV/ROS_Workspace

read -s -n 1

ros2 topic pub --times 1 /canbus/mission_selection custom_msgs/msg/MissionSelection mission_selected:\ 3

read -s -n 1

ros2 topic pub --times 1 /canbus/autonomous_status custom_msgs/msg/AutonomousStatus id:\ 2

read -s -n 1

ros2 topic pub --times 1 /canbus/autonomous_status custom_msgs/msg/AutonomousStatus id:\ 3

read -s -n 1 key

case $key in 
    r|R)
        resetROS
        ;;
esac
