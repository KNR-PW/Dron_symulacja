from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_hardware',
            executable='drone_handler',
            parameters=[
                {'fc_ip': 'udp:127.0.0.1:14540'},
                {'dev': 'true'}
            ]
        )
        
    ])