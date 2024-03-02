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

    ldList = []

    # Velocity Estimation Node
    velocityEstimationNode = Node(
        package='velocity_estimation',
        executable='l_vel',
        name='velocity_estimation',
        parameters=[os.path.join(velocity_estimation_dir, "config", "params.yaml")],
        emulate_tty=True
    )
    ldList.append(velocityEstimationNode)

    ld = LaunchDescription(ldList)
    return ld
