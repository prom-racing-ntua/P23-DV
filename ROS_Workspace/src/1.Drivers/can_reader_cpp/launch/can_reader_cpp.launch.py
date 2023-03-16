import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    can_reader_cpp_dir = get_package_share_directory('can_reader_cpp')
    return LaunchDescription([
        Node(
            package='can_reader_cpp',
            name='can_reader_cpp_node',
            executable='can_reader_cpp',
            parameters=[os.path.join(can_reader_cpp_dir, "config", "params.yaml")],
            output="screen",
            emulate_tty=True
        )
    ])
