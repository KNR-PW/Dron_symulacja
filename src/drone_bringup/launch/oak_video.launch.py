"""
Launch: nagrywanie wideo z kamery OAK-D.

Sam zapis — kamera musi juz chodzic (np. z suas_detect_jetson.launch.py).
Domyslnie nagrywa od razu po starcie, a Ctrl+C konczy i domyka plik .mp4.

Recznie serwisami (autostart:=false): knr_video/turn_on_video i
knr_video/turn_off_video.

Zapisuje z topicu SUROWEGO (/oak/rgb/image_raw), tego samego, ktory czyta
detektor YOLO — nie z wersji /compressed.

Uzycie:
  ros2 launch drone_bringup oak_video.launch.py          # nagrywa; Ctrl+C konczy
  ros2 launch drone_bringup oak_video.launch.py autostart:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/oak/rgb/image_raw",
        description="Topic SUROWEGO obrazu (ten sam, ktory czyta detektor)",
    )
    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="15.0",
        description="FPS zapisywany do pliku — ustaw na REALNA czestotliwosc "
                    "topicu (ros2 topic hz <topic>), inaczej wideo bedzie "
                    "odtwarzane za szybko lub za wolno",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="true = nagrywanie rusza od razu, Ctrl+C je konczy; "
                    "false = sterowanie serwisami knr_video/turn_on_video",
    )
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.expanduser("~/oak_video"),
        description="Katalog bazowy; kazde uruchomienie tworzy w nim kolejny "
                    "podkatalog 1, 2, 3...",
    )

    # ─── Nagrywanie ──────────────────────────────────────
    video_recorder = Node(
        package="drone_camera",
        executable="video_recorder",
        name="video_recorder",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "save_directory_base": LaunchConfiguration("save_dir"),
                "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
                # OAK publikuje BEST_EFFORT — bez tego nie przyszlaby ani jedna klatka
                "best_effort": True,
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            fps_arg,
            autostart_arg,
            save_dir_arg,
            video_recorder,
        ]
    )
