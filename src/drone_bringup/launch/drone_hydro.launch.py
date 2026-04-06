from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    oak_d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('oak_d_ros'),
                'launch',
                'oak_hydro.launch.py'
            )
        )
    )

    drone_node = Node(
        package='drone_hardware',
        executable='drone_handler',
    )

    recorder_node = Node(
        package='drone_camera',
        executable='images_recorder',
        parameters=[{'camera_topic': '/oak/rgb/image_raw'}]
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[{'image_topic': '/oak/rgb/image_raw'}]
    )

    return LaunchDescription([
        oak_d_launch,
        drone_node,
        recorder_node,
        aruco_node,
    ])
