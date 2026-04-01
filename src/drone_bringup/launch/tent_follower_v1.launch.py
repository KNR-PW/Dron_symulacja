"""
Launch:  tent_follower V1 - Prosty P-regulator) 
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Argumenty ze standardowymi nowymi (agresywniejszymi) wartosciami ---
    target_alt_arg = DeclareLaunchArgument("target_alt", default_value="40.0", description="Wysokosc wzlotu [m]")
    kp_xy_arg = DeclareLaunchArgument("kp_xy", default_value="1.3", description="Wzmocnienie X/Y (P)")
    kp_yaw_arg = DeclareLaunchArgument("kp_yaw", default_value="0.5", description="Wzmocnienie Osi Z (Yaw)")
    kp_alt_arg = DeclareLaunchArgument("kp_alt", default_value="0.5", description="Wzmocnienie Osi Z (Wysokosc)")
    kp_forward_arg = DeclareLaunchArgument("kp_forward", default_value="0.15", description="Wzmocnienie ucieczki na wprost [P]")
    kp_hover_arg = DeclareLaunchArgument("kp_hover", default_value="5.0", description="Wzmocnienie mikroruchow w zawisie [P]")
    max_vel_arg = DeclareLaunchArgument("max_vel", default_value="3.5", description="Limit predkosci XY [m/s]")
    max_yaw_rate_arg = DeclareLaunchArgument("max_yaw_rate", default_value="0.5", description="Limit predkosci Yaw [rad/s]")
    max_vz_arg = DeclareLaunchArgument("max_vz", default_value="1.5", description="Limit predkosci pionowej [m/s]")

    # --- 1. Tent Follower V1 (Prosty) ---
    tent_follower = Node(
        package="drone_autonomy",
        executable="tent_follower",
        parameters=[{
            "target_alt": LaunchConfiguration("target_alt"),
            "kp_xy": LaunchConfiguration("kp_xy"),
            "kp_yaw": LaunchConfiguration("kp_yaw"),
            "kp_alt": LaunchConfiguration("kp_alt"),
            "kp_forward": LaunchConfiguration("kp_forward"),
            "kp_hover": LaunchConfiguration("kp_hover"),
            "max_vel": LaunchConfiguration("max_vel"),
            "max_yaw_rate": LaunchConfiguration("max_yaw_rate"),
            "max_vz": LaunchConfiguration("max_vz"),
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
        kp_xy_arg,
        kp_yaw_arg,
        kp_alt_arg,
        kp_forward_arg,
        kp_hover_arg,
        max_vel_arg,
        max_yaw_rate_arg,
        max_vz_arg,
        tent_follower,
        drone_gui,
    ])

