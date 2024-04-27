import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    mpc_dir = get_package_share_directory("mpc_new")
    mpc_new = Node(
        package="mpc_new",
        name="l_mpc_new",
        executable="l_mpc_new",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(mpc_dir, "config", "mpc_new_parameters.yaml")]
    )

    ld.add_action(mpc_new)

    return ld