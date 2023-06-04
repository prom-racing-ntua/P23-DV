import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

"""
This Launch file launches only the nodes that are  Managed (meaning, lifecycle).
"""

def generate_launch_description():
    velocity_estimation_dir = get_package_share_directory('velocity_estimation')
    perception_dir = get_package_share_directory('perception')
    slam_dir = get_package_share_directory('slam')
    pathplanning_dir = get_package_share_directory('path_planner')
    mpc_dir = get_package_share_directory('mpc')
    saltas_dir = get_package_share_directory('saltas')
    pure_pursuit_dir = get_package_share_directory("pid_pp_controller")

    ldList = []

    # Saltas Node
    saltasNode = Node(
        package='saltas',
        executable='lifecycle_saltas',
        name='saltas'
    )
    ldList.append(saltasNode)

    # Velocity Estimation Node
    velocityEstimationNode = Node(
        package='velocity_estimation',
        executable='lifecycle_velocity_estimation',
        name='velocity_estimation'
    )
    ldList.append(velocityEstimationNode)

    # Acquisition Nodes
    acquisitionLeftNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition_left'
    )
    ldList.append(acquisitionLeftNode)
    
    acquisitionRightNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition_right'
    )
    ldList.append(acquisitionRightNode)

    # Inference Node
    inferenceNode = Node(
        package='perception',
        executable='lifecycle_inference',
        name='inference'
    )
    ldList.append(inferenceNode)

    # SLAM Node
    slamNode = Node(
        package='slam',
        executable='lifecycle_slam',
        name='slam'
    )
    ldList.append(slamNode)

    # Pathplanning Node
    pathplanningNode = Node(
        package='path_planner',
        executable='lifecycle_path_planner',
        name='path_planning'
    )
    ldList.append(pathplanningNode)

    # Controls Node
    mpcNode = Node(
        package='mpc',
        executable='lifecycle_mpc',
        name='mpc'
    )
    ldList.append(mpcNode)

    purePursuitNode = Node(
        package="pid_pp_controller",
        executable='lifecycle_pure_pursuit',
        name='pure_pursuit'
    )
    ldList.append(purePursuitNode)
    
    ld = LaunchDescription(ldList)
    return ld