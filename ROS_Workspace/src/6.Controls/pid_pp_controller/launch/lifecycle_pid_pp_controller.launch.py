import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    pid_pp_con_dir = get_package_share_directory("pid_pp_controller")
    pid__pp_node = Node(
        package="pid_pp_controller",
        name="pure_pursuit",
        executable="lifecycle_pid_pp_controller",
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(pid_pp_con_dir, "config", "parameters.yaml")]
    )

    ld.add_action(pid__pp_node)
    return ld