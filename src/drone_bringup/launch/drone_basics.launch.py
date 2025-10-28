from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='drone_hardware',
        #     executable='drone_handler',
        # ),
        # Node(
        #     package='drone_camera',
        #     executable='camera_publisher',
        # )
        Node(
            package='ros2_aruco',
            executable='aruco_node',
            parameters=[{
                'image_topic': '/oak/rgb/image_raw'
            }]
        ),
        Node(
            package='drone_camera',
            executable='images_recorder',
            parameters=[
                {'camera_topic': 'oak/rgb/image_raw'}
                # {'camera_topic': 'camera/image_raw'}
                # {'camera_topic': 'camera'}
            ]
        )
    ])