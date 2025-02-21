from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_detector',
            executable='detector',
        ),
        Node(
            package='drone_detector',
            executable='detection_subscriber',
        ),
    ])