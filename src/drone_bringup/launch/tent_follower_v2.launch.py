"""
Launch: most GZ + YOLO + tent_follower_v2 (Militarny poscig UGV) + Drone GUI.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Argumenty ---
    target_alt_arg = DeclareLaunchArgument(
        "target_alt",
        default_value="4.0",
        description="Wysokosc vzlotu dla V2 [m]",
    )

    # --- 1. Tent Follower V2 (Zaawansowany - Kalman/3D) ---
    tent_follower_v2 = Node(
        package="drone_autonomy",
        executable="tent_follower_v2",
        output="screen",
        parameters=[{
            "target_alt": LaunchConfiguration("target_alt"),
            "detections_topic": "/tent_detections",
            "image_width": 1920,
            "image_height": 1080,
        }],
    )

    # --- 2. Drone GUI ---
    drone_gui = Node(
        package="drone_gui",
        executable="drone_control_gui",
        name="drone_control_gui",
    )

    return LaunchDescription([
        target_alt_arg,
        tent_follower_v2,
        drone_gui,
    ])

