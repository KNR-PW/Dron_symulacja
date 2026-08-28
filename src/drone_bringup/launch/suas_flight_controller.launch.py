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
        # Zmierzone dla OAK-D PRO W (ISP 1/4 z 12MP -> 1014x760)
        DeclareLaunchArgument("vfov_deg", default_value="64.4"),
        DeclareLaunchArgument("img_w", default_value="1014"),
        DeclareLaunchArgument("img_h", default_value="760"),
        DeclareLaunchArgument("damping", default_value="0.6"),
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
        DeclareLaunchArgument("gimbal_deadzone", default_value="0.06"),
        # Filtr falszywych detekcji: M trafien z ostatnich N klatek przed APPROACH
        # (detekcja chodzi ~7-14 FPS, wiec 6/8 to ok. 0.6-1.1 s potwierdzania)
        DeclareLaunchArgument("det_confirm_frames", default_value="6"),
        DeclareLaunchArgument("det_window_frames", default_value="8"),
        DeclareLaunchArgument("det_confirm_gap", default_value="0.5"),
        # Prog pewnosci: parametr 'conf' w launchach detekcji, nie tutaj
        DeclareLaunchArgument("require_same_track", default_value="true"),
        # Konwencja realu: 0 = poziomo, -45 = pod katem w dol/przod, -90 = prosto w dol
        # Dla Gazebo: pitch_search=30.0 pitch_min=-45.0 pitch_max=45.0 pitch_hover_thr=-38.0
        DeclareLaunchArgument("pitch_search", default_value="-45.0"),
        DeclareLaunchArgument("pitch_min", default_value="-90.0"),
        # -30 (nie -45 jak w gimbal_controller): daje zapas nad linia szukania,
        # dzieki czemu vx nie startuje od razu na 100% kp_vx
        DeclareLaunchArgument("pitch_max", default_value="-30.0"),
        DeclareLaunchArgument("pitch_hover_thr", default_value="-83.0"),
    ]

    # ─── Tent Tracker ─────────────────────────────────────
    tracker = Node(
        package="drone_autonomy",
        executable="suas_flight_controller",
        parameters=[{
            "target_alt":     LaunchConfiguration("target_alt"),
            "vfov_deg":       LaunchConfiguration("vfov_deg"),
            "img_w":          LaunchConfiguration("img_w"),
            "img_h":          LaunchConfiguration("img_h"),
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
            "gimbal_deadzone":    LaunchConfiguration("gimbal_deadzone"),
            "det_confirm_frames": LaunchConfiguration("det_confirm_frames"),
            "det_window_frames":  LaunchConfiguration("det_window_frames"),
            "det_confirm_gap":    LaunchConfiguration("det_confirm_gap"),
            "require_same_track": LaunchConfiguration("require_same_track"),
            "pitch_search":       LaunchConfiguration("pitch_search"),
            "pitch_min":          LaunchConfiguration("pitch_min"),
            "pitch_max":          LaunchConfiguration("pitch_max"),
            "pitch_hover_thr":    LaunchConfiguration("pitch_hover_thr"),
        }],
    )

    return LaunchDescription(args + [tracker])
