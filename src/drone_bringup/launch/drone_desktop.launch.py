from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_detector',
            executable='image_subscriber',
        ),
        Node(
            package='drone_gui',
            executable='detector_gui',
        ),
        Node(
            package='drone_gui',
            executable='drone_control_gui',
        )
    ])