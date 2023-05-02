import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

"""
This Launch file launches only the nodes that are NON Managed (meaning, not lifecycle).
The rest of the nodes are launched through the P23 Status Node when we receive a mission
from the VCU. The architecture behind this is that we have different configuration files
AND different node launches in general, depending on the mission.
"""

def generate_launch_description():
    velocity_estimation_dir = get_package_share_directory('velocity_estimation')
    perception_dir = get_package_share_directory('perception')
    slam_dir = get_package_share_directory('slam')
    pathplanning_dir = get_package_share_directory('path_planner')
    # controls_dir = get_package_share_directory('controls')
    saltas_dir = get_package_share_directory('saltas')

    # Velocity Estimation Node
    velocityEstimationNode = Node(
        package='velocity_estimation',
        executable='lifecycle_velocity_estimation',
        name='velocity_estimation'
    )

    # Acquisition Nodes
    acquisitionLeftNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition_left'
    )
    
    acquisitionRightNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition_right'
    )
    
    acquisitionCenterNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition_center'
    )

    # Inference Node
    inferenceNode = Node(
        package='perception',
        executable='lifecycle_inference',
        name='inference'
    )

    # SLAM Node
    slamNode = Node(
        package='slam',
        executable='lifecycle_slam',
        name='slam'
    )

    # Pathplanning Node
    pathplanningNode = Node(
        package='path_planner',
        executable='lifecycle_path_planner',
        name='path_planning'
    )

    # Controls Node

    # Saltas Node
    saltasNode = Node(
        package='saltas',
        executable='lifecycle_saltas',
        name='saltas'
    )
    
    ldList = [velocityEstimationNode, acquisitionCenterNode, acquisitionLeftNode, acquisitionRightNode, slamNode, inferenceNode, saltasNode, pathplanningNode]

    ld = LaunchDescription(ldList)
    return ld