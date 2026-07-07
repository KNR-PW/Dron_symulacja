"""
Launch: suas_flight_controller — maszyna stanów SEARCH → APPROACH → HOVER
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    args = [
        DeclareLaunchArgument("target_alt", default_value="5.0"),
        DeclareLaunchArgument("vfov_deg", default_value="114.6"),
        DeclareLaunchArgument("damping", default_value="0.4"),
        DeclareLaunchArgument("kp_vx", default_value="4.0"),
        DeclareLaunchArgument("kp_vy", default_value="1.0"),
        DeclareLaunchArgument("kp_alt", default_value="0.5"),
        DeclareLaunchArgument("kp_yaw", default_value="0.3"),
        DeclareLaunchArgument("max_vel", default_value="4.0"),
        DeclareLaunchArgument("max_vz", default_value="1.5"),
        DeclareLaunchArgument("max_yaw_rate", default_value="0.5"),
        DeclareLaunchArgument("ema_alpha", default_value="0.15"),
        DeclareLaunchArgument("lost_timeout", default_value="3.0"),
        DeclareLaunchArgument("hover_deadzone", default_value="0.08"),
    ]

    # ─── Tent Tracker ─────────────────────────────────────
    tracker = Node(
        package="drone_autonomy",
        executable="suas_flight_controller",
        parameters=[{
            "target_alt":     LaunchConfiguration("target_alt"),
            "vfov_deg":       LaunchConfiguration("vfov_deg"),
            "damping":        LaunchConfiguration("damping"),
            "kp_vx":          LaunchConfiguration("kp_vx"),
            "kp_vy":          LaunchConfiguration("kp_vy"),
            "kp_alt":         LaunchConfiguration("kp_alt"),
            "kp_yaw":         LaunchConfiguration("kp_yaw"),
            "max_vel":        LaunchConfiguration("max_vel"),
            "max_vz":         LaunchConfiguration("max_vz"),
            "max_yaw_rate":   LaunchConfiguration("max_yaw_rate"),
            "ema_alpha":      LaunchConfiguration("ema_alpha"),
            "lost_timeout":   LaunchConfiguration("lost_timeout"),
            "hover_deadzone": LaunchConfiguration("hover_deadzone"),
        }],
    )

    return LaunchDescription(args + [tracker])
