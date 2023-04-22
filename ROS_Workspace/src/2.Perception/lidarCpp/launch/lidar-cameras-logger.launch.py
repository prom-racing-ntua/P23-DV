import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Works (for now) by itself since it launches the master, but no canbus...

def generate_launch_description():
    ld = LaunchDescription()

    lidar_inference_dir = get_package_share_directory('lidar_cpp')
    lidar_inference_node = Node(
            package="lidar_cpp",
            name="lidar_cpp",
            executable="lidar_cpp",
            output="screen",
            parameters=[os.path.join(lidar_inference_dir, "config", "params.yaml")],
            emulate_tty=True
    )

    lidar_aqcuisition_dir = get_package_share_directory('hesai_lidar')
    launch_path = os.path.join(lidar_aqcuisition_dir, 'launch', 'hesai_lidar_launch.py')    
    lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))

    cameras_aqcuisition_dir = get_package_share_directory('perception')
    launch_path = os.path.join(cameras_aqcuisition_dir, 'launch', 'perception_logger.launch.py')    
    cameras_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))

    ld.add_action(lidar_launch)
    ld.add_action(cameras_launch)
    ld.add_action(lidar_inference_node)

    return ld
