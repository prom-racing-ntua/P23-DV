import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    mpc_dir = get_package_share_directory("mpc")
    mpc_node = Node(
        package="mpc",
        name="mpc_node",
        executable="mpc",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(mpc_dir, "config", "mpc_params.yaml")]
    )

    ld.add_action(mpc_node)
    return ld