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

    ldList = [inferenceNode, saltasNode, pathplanningNode]

    ld = LaunchDescription(ldList)
    return ld