import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

"""
"""

def generate_launch_description():
    ldList = []
    config = os.path.join(
        get_package_share_directory('perception'),
        'config',
        'lifecycle_params.yaml'
    )

    # Saltas Node
    saltasNode = Node(
        package='saltas',
        executable='l_sal',
        name='saltas',
        emulate_tty=True,
        parameters=[config]
    )
    ldList.append(saltasNode)

    # Acquisition Nodes
    acquisitionLeftNode = Node(
        package='perception',
        executable='l_acq',
        name='acquisition_left',
        emulate_tty=True,
        parameters=[config]
    )
    ldList.append(acquisitionLeftNode)
    
    acquisitionRightNode = Node(
        package='perception',
        executable='l_acq',
        name='acquisition_right',
        emulate_tty=True,
        parameters=[config]
    )
    ldList.append(acquisitionRightNode)

    ld = LaunchDescription(ldList)
    return ld