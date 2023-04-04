import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    p23_status_dir = get_package_share_directory("p23_status")
    p23_status_node = Node(
        package="p23_status",
        name="p23_status",
        executable="p23_status",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(p23_status_node)
    return ld