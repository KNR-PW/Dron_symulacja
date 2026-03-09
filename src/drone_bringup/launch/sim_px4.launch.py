from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ]
    )

    px4_bridge = Node(
            package='drone_hardware',
            executable='drone_handler_px4',
        )

    mission_make_photo_server = Node(
        package='drone_camera',
        executable='images_recorder',
        output='screen',
        parameters=[{
            'camera_topic': '/rgb_camera/image'
        }]
    )


    return LaunchDescription([
        px4_bridge,
        bridge,
        mission_make_photo_server,
    ])