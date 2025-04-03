from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_detector',
            executable='camera_publisher',
            parameters=[
                {'camera': '0'}
            ]
        ),
        Node(
            package='drone_detector',
            executable='aruco_detector',
        )
        
    ])