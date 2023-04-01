import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    lifecycle_manager_dir = get_package_share_directory("lifecycle_manager")
    lifecycle_manager_node = Node(
        package="lifecycle_manager",
        name="lifecycle_manager",
        executable="lifecycle_manager",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(lifecycle_manager_node)
    return ld