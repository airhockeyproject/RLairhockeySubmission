from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puck_detection',
            executable='puck_detection',
            name='puck_detector_node',
            output='screen'
        ),
        Node(
            package='puck_detection',
            executable='command_maker',
            name='puck_center_publisher',
            output='screen'
        )
    ])