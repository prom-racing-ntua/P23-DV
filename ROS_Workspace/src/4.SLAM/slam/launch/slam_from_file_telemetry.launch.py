import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    slam_from_file_dir = get_package_share_directory('slam')
    launch_path_1 = os.path.join(slam_from_file_dir, 'launch', 'slam_from_file.launch.py')    
    slam_from_file_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path_1))

    telemetry_dir = get_package_share_directory('telemetry')
    launch_path_2 = os.path.join(telemetry_dir, 'launch', 'telemetry.launch.py')    
    telemetry_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path_2))
    
    ld.add_action(slam_from_file_launch)
    ld.add_action(telemetry_launch)
    
    return ld
