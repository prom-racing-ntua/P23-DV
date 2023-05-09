import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    mpc_from_file_dir = get_package_share_directory("mpc")
    mpc_from_file_node = Node(
        package="mpc",
        name="mpc_from_file",
        executable="mpc_from_file_exe",
        output="screen",
        emulate_tty=True
        # ,parameters=[os.path.join(mpc_from_file_dir, "config", "parameters.yaml")]
    )

    ld.add_action(mpc_from_file_node)
    return ld