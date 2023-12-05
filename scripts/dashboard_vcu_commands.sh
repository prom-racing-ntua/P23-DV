#!/bin/bash
cd ~/P23-DV/ROS_Workspace
. install/setup.bash

echo "Press any key for mission selection. If key is number, must be the desired mission..."
read -s -n 1 miss
case $miss in
  0|1|2|3|4|5|6|7|8|9)
	ros2 topic pub --times 1 /canbus/mission_selection custom_msgs/msg/MissionSelection mission_selected:\ $miss
	echo "Selected mission: $miss"
 	;;
  *)
	ros2 topic pub --times 1 /canbus/mission_selection custom_msgs/msg/MissionSelection mission_selected:\ 3
esac

echo "Press any key for AS Ready"
read -s -n 1

ros2 topic pub --times 1 /canbus/autonomous_status custom_msgs/msg/AutonomousStatus id:\ 2

echo "Press any key for AS Driving"
read -s -n 1

ros2 topic pub --times 1 /canbus/autonomous_status custom_msgs/msg/AutonomousStatus id:\ 3

echo "Press any key to end script OR R to resetROS"
read -s -n 1 key

case $key in 
    r|R)
        ../scripts/./resetROS.sh
        ;;
esac
