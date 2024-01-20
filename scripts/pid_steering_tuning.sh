cd ~/P23-DV/ROS_Workspace
. install/setup.bash

ros2 launch inspection inspection.launch.py &
ros2 launch can_reader can_interface.launch.py &
echo "Launched bench controller and can interface. Press any button to configure bench controller. Press r to reset ROS."

read -s -n 1 key

case $key in
  r|R)
  ../scripts/./resetROS.sh
  exit 0
  ;;
esac

ros2 lifecycle set /inspection configure
echo "Configured bench controller. Press any button to activate bench controller. Press r to reset ROS."
read -s -n 1 key

case $key in
  r|R)
  ../scripts/./resetROS.sh
  exit 0
  ;;
esac

ros2 lifecycle set /inspection activate
echo "Activated bench controller. Have fun with your steering tuning."
