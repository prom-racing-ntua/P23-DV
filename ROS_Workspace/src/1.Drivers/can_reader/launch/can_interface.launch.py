import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('can_reader')
    
    can_interface_node = Node(
        name='can_interface',
        package='can_reader',
        executable='can_interface',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'params.yaml')],
        namespace='canbus',
        # remappings=[
        #     ('/p23_status/control_commands', '/control_commands', ),
        #     ('/p23_status/system_state', '/system_state')
        # ],
        emulate_tty=True)

    ld = LaunchDescription()
    ld.add_action(can_interface_node)

    return ld