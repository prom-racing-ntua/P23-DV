import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    slam_dir = get_package_share_directory("slam")
    slam_from_file_node = Node(
        package="slam",
        name="slam_from_file_node",
        executable="read_files",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(slam_dir, "config", "from_file_params.yaml")]
    )

    ld.add_action(slam_from_file_node)
    return ld