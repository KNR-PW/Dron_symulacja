"""
Launch: zapisywanie zdjec z kamery OAK-D.

Sam zapis — kamera musi juz chodzic (np. z suas_detect_jetson.launch.py).

Domyslnie zapisuje z topicu SUROWEGO (/oak/rgb/image_raw), tego samego, ktory
czyta detektor YOLO. Opcjonalnie moze rownolegle pisac drugi zestaw zdjec z
innego topicu (np. /tent_detections/image z ramkami) do osobnego katalogu.

Uzycie:
  ros2 launch drone_bringup oak_photos.launch.py
  ros2 launch drone_bringup oak_photos.launch.py camera_topic:=/tent_detections/image
  ros2 launch drone_bringup oak_photos.launch.py second:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ─── Zapis nr 1 ───────────────────────────────────────
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/oak/rgb/image_raw",
        description="Topic obrazu (sensor_msgs/Image)",
    )
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.expanduser("~/oak_photos"),
        description="Katalog bazowy; kazde uruchomienie tworzy w nim kolejny "
                    "podkatalog 1, 2, 3...",
    )
    fps_arg = DeclareLaunchArgument(
        "fps",
        default_value="1.0",
        description="Ile zdjec na sekunde zapisywac",
    )

    # ─── Zapis nr 2 (opcjonalny, inny topic + inny katalog) ──
    second_arg = DeclareLaunchArgument(
        "second",
        default_value="false",
        description="true = uruchom drugi zapis, z camera_topic2 do save_dir2",
    )
    camera_topic2_arg = DeclareLaunchArgument(
        "camera_topic2",
        default_value="/tent_detections/image",
        description="Topic drugiego zapisu",
    )
    save_dir2_arg = DeclareLaunchArgument(
        "save_dir2",
        default_value=os.path.expanduser("~/oak_photos_det"),
        description="Katalog bazowy drugiego zapisu",
    )
    fps2_arg = DeclareLaunchArgument(
        "fps2",
        default_value="1.0",
        description="Ile zdjec na sekunde w drugim zapisie",
    )

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

    # Inna nazwa node'a, inaczej dwa procesy bilyby sie o ta sama nazwe w grafie
    images_recorder2 = Node(
        package="drone_camera",
        executable="images_recorder",
        name="images_recorder2",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic2"),
                "save_directory_base": LaunchConfiguration("save_dir2"),
                "fps": ParameterValue(LaunchConfiguration("fps2"), value_type=float),
                "best_effort": True,
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("second")),
    )

    return LaunchDescription(
        [
            camera_topic_arg,
            save_dir_arg,
            fps_arg,
            second_arg,
            camera_topic2_arg,
            save_dir2_arg,
            fps2_arg,
            images_recorder,
            images_recorder2,
        ]
    )
