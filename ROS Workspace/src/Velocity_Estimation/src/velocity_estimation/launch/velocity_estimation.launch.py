import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('velocity_estimation')
    vectornav_dir = get_package_share_directory('vectornav')

    estimation_node = Node(
            package="velocity_estimation",
            executable="velocity_estimation",
            output="screen",
            parameters=[os.path.join(this_dir, "config", "params.yaml")])

    vn200_node = Node(
            name='node',
            package='vectornav', 
            executable='vectornav',
            output='screen',
            parameters=[os.path.join(vectornav_dir, 'config', 'vn_200.yaml')],
            namespace='vn_200')

    ld = LaunchDescription()
    ld.add_action(estimation_node)
    ld.add_action(vn200_node)
    
    return ld