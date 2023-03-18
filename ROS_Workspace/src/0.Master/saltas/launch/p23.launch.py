import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('saltas')
    perception_dir = get_package_share_directory('perception')
    vel_est_dir = get_package_share_directory('velocity_estimation')
    slam_dir = get_package_share_directory('slam')

    masterConfig = os.path.join(
        this_dir,
        'config',
        'global_configuration.yaml'
    )

    # Nested Launch Files
    velocityEstimationLaunch = os.path.join(
        vel_est_dir,
        'launch',
        'velocity_estimation.launch.py'
    )

    perceptionLaunch = os.path.join(
        perception_dir,
        'launch',
        'saltas_acquisition.launch.py'
    )

    slamLaunch = os.path.join(
        slam_dir,
        'launch',
        "slam.launch.py"
    )

    # TODO: First start every node EXCEPT salta, then start salta when you are ready to run

    # Master Node
    saltas = Node(
        name='saltas_node',
        package='saltas',
        executable='run_salta',
        output='screen',
        parameters=[masterConfig]
    )

    # Perception Nodes
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perceptionLaunch)
    )

    # Velocity Estimation Nodes
    velocity_estimation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(velocityEstimationLaunch)
    )

    # SLAM Nodes
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slamLaunch)
    )

    ldList = [perception, velocity_estimation, slam]

    ld = LaunchDescription(ldList)
    return ld