
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import SetEnvironmentVariable

web_telemetry = Node(
           package='drone_web',
           executable='ros_mission_website',
           parameters=[
               {'base_url': 'https://telemetria-osadniik.pythonanywhere.com/',
                "camera_topic": 'camera/image_raw',}
           ]
        )
    
def generate_launch_description():
    log_dir = os.path.expanduser('~/ros2_logs')  # Folder na logi
    os.makedirs(log_dir, exist_ok=True) 

    healthcheck = Node(
            package='drone_hardware',
            executable='healthcheck',
            parameters=[
                {'camera_topic': 'camera/image_raw'},
                # {'camera_topic': 'camera'},
                {'required_nodes': ['ros_mission_website']}
            ],
        )

    healthcheck_action = TimerAction(
            period=14.0,
            actions=[
                healthcheck
            ]
        )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', log_dir),
        SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY', 'INFO'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        Node(
            package='drone_hardware',
            executable='drone_handler',
            parameters=[
                {'dev': 'true'}
            ]
        ),
         Node(
             package='camera_ros',
             executable='camera_node',
            #  parameters=[
            #      {'format': 'RGB888'}
            #  ]
         ),
        Node(
            package='drone_camera',
            executable='images_recorder',
            parameters=[
                {'camera_topic': 'camera/image_raw'}
                # {'camera_topic': 'camera'}
            ]
        ),
        Node(
            package='drone_camera',
            executable='video_recorder',
            parameters=[
                {'camera_topic': 'camera/image_raw'}
                # {'camera_topic': 'camera'}
            ]
        ),
       Node(
           package='ros2_aruco',
           executable='aruco_node',
           parameters=[
               {'image_topic': 'camera/image_raw',
               "aruco_dictionary_id": "DICT_ARUCO_ORIGINAL",
               "intrinsic_matrix": [3.97045297e+03, 0.00000000e+00, 2.04507985e+03, 0.00000000e+00, 3.97159037e+03, 1.55103947e+03, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
            #    "distortion": [-4.91966963e-01, 8.04697976e-01, -4.74937802e-04, 1.52222453e-03, -1.36235112e+00]
               },

               # {'camera_topic': 'camera'}
           ]
       ),
        healthcheck_action,
        web_telemetry,
    ])