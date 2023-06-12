import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    simple_sim_dir = get_package_share_directory("simple_sim")
    simple_sim_node = Node(
        package="simple_sim",
        name="simple_sim",
        executable="simple_sim",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(simple_sim_node)
    return ld