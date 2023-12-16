import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    telemetry_dir = get_package_share_directory("telemetry")
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + os.path.join(telemetry_dir, "launch", "telemetry.rviz")]
    )

    telemetry_node = Node(
        package="telemetry",
        name="telemetry",
        executable="telemetry",
        output="screen",
        emulate_tty=True
    )

    telemetry_data_node = Node(
        package = "telemetry_data",
        name = "telemetry_data", 
        executable = "telemetry_data", 
        output = "screen",
        emulate_tty = True
    )

    ld.add_action(rviz2_node)
    ld.add_action(telemetry_node)
    ld.add_action(telemetry_data_node)
    return ld