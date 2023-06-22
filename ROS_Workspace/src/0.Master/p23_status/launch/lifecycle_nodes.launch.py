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
        executable='l_sal',
        name='saltas',
        emulate_tty=True
    )
    ldList.append(saltasNode)

    # Velocity Estimation Node
    velocityEstimationNode = Node(
        package='velocity_estimation',
        executable='l_vel',
        name='velocity_estimation',
        emulate_tty=True
    )
    ldList.append(velocityEstimationNode)

    # Acquisition Nodes
    acquisitionLeftNode = Node(
        package='perception',
        executable='l_acq',
        name='acquisition_left',
        emulate_tty=True
    )
    ldList.append(acquisitionLeftNode)
    
    acquisitionRightNode = Node(
        package='perception',
        executable='l_acq',
        name='acquisition_right',
        emulate_tty=True
    )
    # ldList.append(acquisitionRightNode)

    # Inference Node
    inferenceNode = Node(
        package='perception',
        executable='l_inf',
        name='inference',
        emulate_tty=True
    )
    ldList.append(inferenceNode)

    # SLAM Node
    slamNode = Node(
        package='slam',
        executable='l_slm',
        name='slam',
        emulate_tty=True
    )
    ldList.append(slamNode)

    # Pathplanning Node
    pathplanningNode = Node(
        package='path_planner',
        executable='l_pth',
        name='path_planning',
        emulate_tty=True
    )
    ldList.append(pathplanningNode)

    # Controls Node
    mpcNode = Node(
        package='mpc',
        executable='l_mpc',
        name='mpc',
        emulate_tty=True
    )
    ldList.append(mpcNode)

    purePursuitNode = Node(
        package="pid_pp_controller",
        executable='l_pid',
        name='pure_pursuit',
        emulate_tty=True
    )
    ldList.append(purePursuitNode)
    
    # Can be uncommented when running the inspection mission
    # inspectionNode = Node(
    #     package="inspection",
    #     executable="inspection",
    #     name="inspection",
    #     emulate_tty=True
    # )
    # ldList = [inspectionNode]

    ld = LaunchDescription(ldList)
    return ld
