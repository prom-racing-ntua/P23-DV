from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Inference Node
    inferenceNode = Node(
        package='perception',
        executable='lifecycle_inference',
        name='inference'
    )
    
    saltasNode = Node(
        package='saltas',
        executable='lifecycle_saltas',
        name='saltas'
    )

    pathplanningNode = Node(
        package='path_planner',
        executable='lifecycle_path_planner',
        name='path_planning'
    )

    acquisitionNode = Node(
        package='perception',
        executable='lifecycle_acquisition',
        name='acquisition'
    )

    velocityEstimationNode = Node(
        package='velocity_estimation',
        executable='lifecycle_velocity_estimation',
        name='velocity_estimation'
    )
    
    slamNode = Node(
        package='slam',
        executable='lifecycle_slam',
        name='slam'
    )

    mpcNode = Node(
        package='mpc',
        executable='lifecycle_mpc',
        name='mpc'
    )

    ldList = [inferenceNode, saltasNode, pathplanningNode, velocityEstimationNode, slamNode, mpcNode]

    ld = LaunchDescription(ldList)
    return ld