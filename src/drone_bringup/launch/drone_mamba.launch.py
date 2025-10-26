from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Ścieżka do launch file kamery DepthAI
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'rgbd_pcl.launch.py'
            )
        )
    )

    #Node od drona
    drone_node = Node(
        package='drone_hardware',
        executable='drone_handler',
    )

    #Node do zapisu obrazu z kamery
    recorder_node = Node(
        package='drone_camera',
        executable='images_recorder',
        parameters=[
            {'camera_topic': 'oak/rgb/image_raw'}
        ]
    )

    # Node do detekcji ArUco
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[{
            'image_topic': 'oak/rgb/image_raw'
        }]
    )

    #wszystko
    return LaunchDescription([
        depthai_launch,   # kamera
        drone_node,       # sterowanie dronem
        recorder_node,    # nagrywanie
        aruco_node        # detekcja ArUco
    ])
