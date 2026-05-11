from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # OAK PoE publisher (bypass for depthai_ros_driver - direct dai.Device path).
    # Override 'ip' from CLI for PoE: ros2 launch ... oak_ip:=169.254.1.222
    oak_node = Node(
        package='drone_camera',
        executable='oak_publisher',
        name='oak_publisher',
        parameters=[{
            'ip': '169.254.1.222',
            'fps': 60,
            'width': 1920,
            'height': 1080,
            'frame_id': 'oak_rgb_camera_optical_frame',
        }],
    )

    drone_node = Node(
        package='drone_hardware',
        executable='drone_handler',
    )

    recorder_node = Node(
        package='drone_camera',
        executable='images_recorder',
        parameters=[
            {'camera_topic': 'oak/rgb/image_raw'}
        ]
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[{
            'image_topic': 'oak/rgb/image_raw'
        }]
    )

    return LaunchDescription([
        oak_node,
        drone_node,
        recorder_node,
        aruco_node,
    ])
