import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    vectornav_dir = get_package_share_directory('vectornav')
    launch_path = os.path.join(vectornav_dir, 'launch', 'both_sensors.launch.py')    
    vectornav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))

    logger_cmd = Node(
        package="data_logger",
        executable="data_logger",
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(vectornav_launch)
    ld.add_action(logger_cmd)

    return ld