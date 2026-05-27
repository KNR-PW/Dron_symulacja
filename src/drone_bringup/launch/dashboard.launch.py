from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/oak/rgb/image_raw',
            description='ROS 2 image topic for WebRTC video stream'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP port for the dashboard web server'
        ),

        Node(
            package='drone_web',
            executable='webcam_dashboard',
            output='screen',
            arguments=[
                '--camera-topic', LaunchConfiguration('camera_topic'),
                '--port', LaunchConfiguration('port'),
            ]
        ),
    ])
