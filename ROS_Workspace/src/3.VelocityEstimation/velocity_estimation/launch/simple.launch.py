import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Works (for now) by itself since it launches the master, but no canbus...

def generate_launch_description():
    ld = LaunchDescription()

    vel_estimation_dir = get_package_share_directory('velocity_estimation')
    estimation_node = Node(
            package="velocity_estimation",
            name="velocity_estimation_node",
            executable="velocity_estimation",
            output="screen",
            parameters=[os.path.join(vel_estimation_dir, "config", "params.yaml")],
            emulate_tty=True
    )

    master_dir = get_package_share_directory('saltas')
    master = Node(
        name='saltas_node',
        package='saltas',
        executable='run_salta',
        output='screen',
        parameters=[os.path.join(master_dir, 'config', 'global_configuration.yaml')]
    )

    vectornav_dir = get_package_share_directory('vectornav')
    launch_path = os.path.join(vectornav_dir, 'launch', 'both_sensors.launch.py')    
    vectornav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))

    ld.add_action(vectornav_launch)
    ld.add_action(estimation_node)
    ld.add_action(master)
    return ld