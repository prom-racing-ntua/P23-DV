import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

"""
This Launch file launches only the nodes that are NON Managed (meaning, not lifecycle).
The rest of the nodes are launched through the P23 Status Node when we receive a mission
from the VCU. The architecture behind this is that we have different configuration files
AND different node launches in general, depending on the mission.
"""

def generate_launch_description():
    p23_status_dir = get_package_share_directory('p23_status')
    lifecycle_manager_dir = get_package_share_directory('lifecycle_manager')
    canbus_dir = get_package_share_directory('can_reader_cpp')
    vectornav_dir = get_package_share_directory('vectornav')

    # P23 Status Node
    p23_status_launch = os.path.join(p23_status_dir, 'launch', 'p23status.launch.py')
    p23_status = IncludeLaunchDescription(PythonLaunchDescriptionSource(p23_status_launch))

    # Lifecycle Manager
    lifecycle_manager_launch = os.path.join(lifecycle_manager_dir, 'launch', 'lifecycle_manager.launch.py')
    lifecycle_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource(lifecycle_manager_launch))

    # CANBUS Module
    canbus_launch = os.path.join(canbus_dir, 'launch', 'can_reader_cpp.launch.py')
    canbus = IncludeLaunchDescription(PythonLaunchDescriptionSource(canbus_launch))

    # Vectornav Sensors
    vectornav_launch = os.path.join(vectornav_dir, 'launch', 'both_sensors.launch.py')
    vectornav = IncludeLaunchDescription(PythonLaunchDescriptionSource(vectornav_launch))

    ldList = [p23_status, lifecycle_manager, canbus, vectornav]

    ld = LaunchDescription(ldList)
    return ld


    
