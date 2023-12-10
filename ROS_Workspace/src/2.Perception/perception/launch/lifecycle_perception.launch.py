from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'lifecycle_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='perception',
            executable='l_inf',
            name='inference',
        ),
        Node(
            package='perception',
            executable='l_acq',
            name='acquisition_left',
            parameters=[config],
        ),
        Node(
            package='perception',
            executable='l_acq',
            name='acquisition_right',
            parameters=[config],
        )
    ])
