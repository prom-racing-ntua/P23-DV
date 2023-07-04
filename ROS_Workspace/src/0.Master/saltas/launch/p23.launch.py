import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    this_dir = get_package_share_directory('saltas')


    # Sensor Nodes
    can_interface_dir = get_package_share_directory('can_reader')
    can_launch = os.path.join(can_interface_dir, 'launch', 'can_interface.launch.py')
    can_interface = IncludeLaunchDescription(PythonLaunchDescriptionSource(can_launch))

    vectornav_dir = get_package_share_directory('vectornav')
    vectornav_launch = os.path.join(vectornav_dir, 'launch', 'both_sensors.launch.py')    
    vectornav = IncludeLaunchDescription(PythonLaunchDescriptionSource(vectornav_launch))


    # Velocity Estimation Node
    vel_est_dir = get_package_share_directory('velocity_estimation')
    velocityEstimationLaunch = os.path.join(vel_est_dir, 'launch', 'velocity_estimation.launch.py')
    velocity_estimation = IncludeLaunchDescription(PythonLaunchDescriptionSource(velocityEstimationLaunch))


    # Perception Nodes
    perception_dir = get_package_share_directory('perception')
    perceptionLaunch = os.path.join(perception_dir, 'launch', 'saltas_acquisition.launch.py')
    perception = IncludeLaunchDescription(PythonLaunchDescriptionSource(perceptionLaunch))


    # SLAM - Localization Node
    slam_dir = get_package_share_directory('slam')
    slamLaunch = os.path.join(slam_dir, 'launch', 'slam.launch.py')
    slam = IncludeLaunchDescription(PythonLaunchDescriptionSource(slamLaunch))


    # Path Planning Node
    path_planning_dir = get_package_share_directory('path_planner')
    pathPlanningLaunch = os.path.join(path_planning_dir, 'launch', 'path_planner.launch.py')
    path_planning = IncludeLaunchDescription(PythonLaunchDescriptionSource(pathPlanningLaunch))


    # TODO: First start every node EXCEPT salta, then start salta when you are ready to run
    # Master Node
    masterConfig = os.path.join(
        this_dir,
        'config',
        'global_configuration.yaml'
    )

    saltas = Node(
        name='saltas_node',
        package='saltas',
        executable='run_salta',
        output='screen',
        parameters=[masterConfig]
    )


    ldList = [can_interface, vectornav, perception, velocity_estimation, slam, path_planning, saltas]

    ld = LaunchDescription(ldList)
    return ld