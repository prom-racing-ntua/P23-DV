import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# This launch doesn't work by itself, it needs the master node to be running

def generate_launch_description():
    # Set to True to launch the data logger as well
    logging = True
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

    ld.add_action(estimation_node)
    
    if logging:
        logger_cmd = Node(
            package="data_logger",
            executable="data_logger",
            output="screen",
            emulate_tty=True
        )
        ld.add_action(logger_cmd)
    
    return ld