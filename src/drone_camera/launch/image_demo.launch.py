from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_camera',
            executable='image_publisher',
        ),
        Node(
            package='drone_camera',
            executable='image_subscriber',
        ),
    ])