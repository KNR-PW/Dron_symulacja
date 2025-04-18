from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_hardware',
            executable='drone_handler',
        ),
        Node(
            package='drone_camera',
            executable='camera_publisher',
        )
        
    ])