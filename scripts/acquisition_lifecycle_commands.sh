ros2 launch perception lifecycle_acquisition.launch.py &

sleep 2

ros2 lifecycle set /acquisition_right configure
ros2 lifecycle set /acquisition_left configure
ros2 lifecycle set /saltas configure
ros2 lifecycle set /acquisition_right activate
ros2 lifecycle set /acquisition_left activate
ros2 lifecycle set /saltas activate