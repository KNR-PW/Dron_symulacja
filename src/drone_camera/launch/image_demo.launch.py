from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[
                {'format': 'RGB888'}
            ]
        ),
        Node(
            package='drone_camera',  # lub inna paczka gdzie jest images_recorder
            executable='images_recorder',
            name='images_recorder',
            parameters=[{
                'camera_topic': '/camera/image_raw', 
                'enable_timer': False,
            }],
            output='screen',
        ),
    ])
