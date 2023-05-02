import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    velocity_estimation_dir = get_package_share_directory('velocity_estimation')
    perception_dir = get_package_share_directory('perception')
    slam_dir = get_package_share_directory('slam')
    # pathplanning_dir = get_package_share_directory('path_planner')
    # controls_dir = get_package_share_directory('controls')
    saltas_dir = get_package_share_directory('saltas')

    # Inference Node
    inferenceNode = Node(
        package='perception',
        executable='lifecycle_inference',
        name='inference'
    )
    
    saltasNode = Node(
        package='saltas',
        executable='lifecycle_saltas',
        name='saltas'
    )

    pathplanningNode = Node(
        package='path_planner',
        executable='lifecycle_path_planner',
        name='path_planning'
    )

    acquisitionNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition'
    )

    ldList = [inferenceNode, saltasNode, pathplanningNode]

    ld = LaunchDescription(ldList)
    return ld