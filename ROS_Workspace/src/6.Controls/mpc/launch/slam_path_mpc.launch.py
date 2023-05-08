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

    path_planner_dir = get_package_share_directory('path_planner')
    launch_path_2 = os.path.join(path_planner_dir, 'launch', 'path_planner.launch.py')    
    path_planner_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path_2))
    
    mpc_dir = get_package_share_directory('mpc')
    launch_path_3 = os.path.join(mpc_dir, 'launch', 'mpc.launch.py')    
    mpc_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path_3))

    ld.add_action(slam_from_file_launch)
    ld.add_action(path_planner_launch)
    ld.add_action(mpc_launch)
    
    return ld
