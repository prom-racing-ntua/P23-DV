from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Perception',
            namespace='camera1',
            executable='camera',
            name='camera',
            parameters=[{"serialNumber":"KE0220030137"}]
        ),
        Node(
            package='Perception',
            namespace='camera2',
            executable='camera',
            name='camera',
            parameters=[{"serialNumber":"KE0220030138"}]
        ),
        Node(
            package='Perception',
            namespace='camera3',
            executable='camera',
            name='camera',
            parameters=[{"serialNumber":"KE0220040196"}]
        )
    ])