import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('inspection'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='inspection',
            executable='keyboard_publisher', #same as name given on setup.py
            name='keyboard_publisher' #needed for config file
        ),
        Node(
            package='inspection',
            executable='insp',
            name='inspection',
            parameters=[config]
        ),
    ])

# def generate_launch_description():

#     config = os.path.join(
#         get_package_share_directory('bench_controller'),
#         'config',
#         'params.yaml'
#     )
#     return LaunchDescription([
#         Node(
#             package='bench_controller',
#             executable='keyboard_publisher',
#             name='keyboard_publisher'
#         ),
#         Node(
#             package='bench_controller',
#             executable='bench_controller', #same as name given on setup.py
#             name='bench_controller',  #needed for config file
#             parameters=[config]
#         ),
#     ])