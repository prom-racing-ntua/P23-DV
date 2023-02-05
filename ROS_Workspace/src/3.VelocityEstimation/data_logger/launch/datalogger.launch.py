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

    camera_logger = Node(
        package="perception",
        executable="perception",
        output="screen"
    )

    logger_cmd = Node(
        package="data_logger",
        executable="data_logger",
        output="screen",
        emulate_tty=True,
    )

    vn200_node = Node(
            name='node',
            package='vectornav', 
            executable='vectornav',
            output='screen',
            parameters=[os.path.join(vectornav_dir, 'config', 'vn_200.yaml')],
            namespace='vn_200')

    ld = LaunchDescription()
    # ld.add_action(vectornav_launch)
    ld.add_action(logger_cmd)
    ld.add_action(camera_logger)
    ld.add_action(vn200_node)

    return ld