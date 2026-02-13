from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # launch kamery
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthai_ros_driver'),
                'launch',
                'rgbd_pcl.launch.py'
            )
        )
    )

    # Node do drona
    drone_node = Node(
        package='drone_hardware',
        executable='drone_handler_px4',
    )

    # Node do kamery
    recorder_node = Node(
        package='drone_camera',
        executable='images_recorder',
        parameters=[
            {'camera_topic': 'oak/rgb/image_raw'}
        ]
    )
    # Node do kalibracji serw 
    calibration_node = Node(
        package='drone_hardware',
        executable='actuator_test_server',  
        name='preflight_calibration_publisher',
        output='screen',
        emulate_tty=True,
    )

    # Node aruco
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[{
            'image_topic': 'oak/rgb/image_raw'
        }]
    )

    #
    return LaunchDescription([
        depthai_launch,   # kamera
        calibration_node, # kalibracja ser
        drone_node,       # sterowanie dronem
        recorder_node,    # nagrywanie
        aruco_node        # detekcja ArUco
    ])
