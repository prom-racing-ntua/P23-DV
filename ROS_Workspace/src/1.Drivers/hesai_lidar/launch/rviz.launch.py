import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    slam_dir = get_package_share_directory("hesai_lidar")
    print(slam_dir)
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=['-d' + os.path.join(slam_dir, "config", "lidar.rviz")]
    )

    ld.add_action(rviz2_node)
    return ld