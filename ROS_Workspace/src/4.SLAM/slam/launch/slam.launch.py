import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    slam_dir = get_package_share_directory("slam")
    slam_node = Node(
        package="slam",
        name="slam_node",
        executable="l_slm",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(slam_dir, "config", "params.yaml")]
    )

    ld.add_action(slam_node)
    return ld