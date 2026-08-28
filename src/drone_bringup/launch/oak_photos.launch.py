"""
Launch: zapisywanie zdjec z kamery OAK-D.

Sam zapis — kamera musi juz chodzic (np. z suas_detect_jetson.launch.py).

Zapisuje klatki z topicu SUROWEGO (/oak/rgb/image_raw), tego samego, ktory
czyta detektor YOLO. Nie uzywa wersji /compressed, wiec zdjecia sa w pelnej
jakosci ISP, bez strat JPEG z podgladu.

Uzycie:
  ros2 launch drone_bringup oak_photos.launch.py
  ros2 launch drone_bringup oak_photos.launch.py fps:=2.0 save_dir:=/home/jetsonknr/zdjecia
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
        default_value="1.0",
        description="Ile zdjec na sekunde zapisywac",
    )
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.expanduser("~/oak_photos"),
        description="Katalog bazowy; kazde uruchomienie tworzy w nim kolejny "
                    "podkatalog 1, 2, 3...",
    )

    # ─── Zapis zdjec ─────────────────────────────────────
    images_recorder = Node(
        package="drone_camera",
        executable="images_recorder",
        name="images_recorder",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "save_directory_base": LaunchConfiguration("save_dir"),
                # value_type=float, bo node deklaruje fps jako double —
                # bez tego "fps:=2" (int) wywaliloby sie na niezgodnosci typu
                "fps": ParameterValue(LaunchConfiguration("fps"), value_type=float),
                # OAK publikuje BEST_EFFORT — bez tego nie przyszlaby ani jedna klatka
                "best_effort": True,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            fps_arg,
            save_dir_arg,
            images_recorder,
        ]
    )
