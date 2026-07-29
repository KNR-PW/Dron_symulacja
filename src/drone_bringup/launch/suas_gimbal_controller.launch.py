"""
Launch: suas_gimbal_controller (test sledzenia namiotu samym pitchem gimbala)
+ podglad detekcji w rqt_image_view.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    args = [
        DeclareLaunchArgument("vfov_deg", default_value="114.6"),
        DeclareLaunchArgument("damping", default_value="0.4"),
        DeclareLaunchArgument("img_h", default_value="1024"),
        DeclareLaunchArgument("control_rate", default_value="10.0"),
        DeclareLaunchArgument("lost_timeout", default_value="2.0"),
        DeclareLaunchArgument("deadzone", default_value="0.06"),
        # Konwencja realu: 0 = poziomo, -45 = pod katem w dol/przod, -90 = prosto w dol
        DeclareLaunchArgument("pitch_search", default_value="-45.0"),
        DeclareLaunchArgument("pitch_min", default_value="-90.0"),
        DeclareLaunchArgument("pitch_max", default_value="-45.0"),
    ]

    # ─── Gimbal Tracker ───────────────────────────────────
    gimbal_tracker = Node(
        package="drone_autonomy",
        executable="suas_gimbal_controller",
        parameters=[{
            "vfov_deg":     LaunchConfiguration("vfov_deg"),
            "damping":      LaunchConfiguration("damping"),
            "img_h":        LaunchConfiguration("img_h"),
            "control_rate": LaunchConfiguration("control_rate"),
            "lost_timeout": LaunchConfiguration("lost_timeout"),
            "deadzone":     LaunchConfiguration("deadzone"),
            "pitch_search": LaunchConfiguration("pitch_search"),
            "pitch_min":    LaunchConfiguration("pitch_min"),
            "pitch_max":    LaunchConfiguration("pitch_max"),
        }],
    )

    return LaunchDescription(args + [gimbal_tracker])
