import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    simple_sim_dir = get_package_share_directory("simple_sim")
    simple_sim_node = Node(
        package="simple_sim",
        name="sim_with_brake",
        executable="sim_with_brake", 
        output="screen",
        emulate_tty=True,
        parameters=[os.path.join(simple_sim_dir, "config", "parameters.yaml")]
    )

    ld.add_action(simple_sim_node)
    return ld