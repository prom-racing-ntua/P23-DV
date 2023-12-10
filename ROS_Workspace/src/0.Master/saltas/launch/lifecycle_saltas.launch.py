import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('saltas')

    saltas = Node(
        name='l_sal',
        package='saltas',
        executable='l_sal',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'global_configuration.yaml')]
    )

    ld = LaunchDescription()
    ld.add_action(saltas)
    return ld