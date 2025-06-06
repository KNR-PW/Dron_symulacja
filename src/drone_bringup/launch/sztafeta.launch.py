
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

    host_bridge = Node(
            package='drone_hardware',
            executable='host_bridge',
            parameters = [
                {'uart_port': '/dev/ttyUSB0'}
            ]
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
        healthcheck_action,
        host_bridge,
        web_telemetry,
    ])