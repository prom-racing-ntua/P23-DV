import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    mpc_dir = get_package_share_directory("mpc")
    mpc = Node(
        package="mpc",
        name="mpc",
        executable="mpc",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(mpc_dir, "config", "mpc_params.yaml")]
    )
    # rviz2_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=['-d' + os.path.join(mpc_dir, "launch", "control_viz.rviz")]
    # )

    ld.add_action(mpc)
    # ld.add_action(rviz2_node)
    return ld