from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('Perception'),
        'config',
        'acquisition_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='Perception',
            namespace='acquisition_left',
            executable='acquisition',
            name='acquisition_left',
            parameters=[config]
        ),
        Node(
            package='Perception',
            namespace='acquisition_right',
            executable='acquisition',
            name='acquisition_right',
            parameters=[config]
        ),
        Node(
            package='Perception',
            namespace='acquisition_center',
            executable='acquisition',
            name='acquisition_center',
            parameters=[config]
        )
    ])