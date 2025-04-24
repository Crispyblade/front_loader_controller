from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='front_loader_control',
            executable='loader_controller',
            name='front_loader_controller',
            output='screen'
        ),
        Node(
            package='front_loader_control',
            executable='rviz',
            name='front_loader_rviz',
            output='screen'
        )
    ])