import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_dir = get_package_share_directory('slam')
    return LaunchDescription([
        Node(
            package='slam',
            name='slam_node',
            executable='slam',
            parameters=[os.path.join(slam_dir, "config", "params.yaml")],
            output="screen",
            emulate_tty=True
        ),
        Node(
            package='fake_velocity_to_slam',
            executable='fake_velocity_to_slam',
            name='fake_velocity_node',
        ),
        Node(
            package='fake_perc_to_slam',
            executable='fake_perc_to_slam',
            name='fake_perception_node'
        )
    ])
