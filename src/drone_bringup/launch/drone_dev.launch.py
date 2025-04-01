from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    log_dir = os.path.expanduser('~/ros2_logs/logi.txt')  # Folder na logi
    os.makedirs(log_dir, exist_ok=True) 

    return LaunchDescription([
        Node(
            package='drone_hardware',
            executable='drone_handler',
            parameters=[
                {'dev': 'true'}
            ]
        ),
        Node(
            package='drone_detector',
            executable='camera_publisher'
        )
    ])