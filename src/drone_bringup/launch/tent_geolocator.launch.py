"""
Launch: zapis wspolrzednych GPS namiotu podczas przelotu ortofoto.

Sam node — kamera, detektor i drone_handler musza juz chodzic. Node ustawia
gimbal na -90 (prosto w dol), slucha /tent_detections i /knr_hardware/telemetry,
rzutuje detekcje na ziemie i grupuje je przestrzennie. Wynik: tent_target.json.

Uzycie:
  ros2 launch drone_bringup tent_geolocator.launch.py
  ros2 launch drone_bringup tent_geolocator.launch.py cluster_radius:=6.0
  ros2 launch drone_bringup tent_geolocator.launch.py lock_nadir:=false

UWAGA: przy lock_nadir:=true (domyslnie) NIE uruchamiaj rownolegle
suas_gimbal_controller — biliby sie o kat gimbala.
Szczegoly: docs/tent_geolocator.md
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.expanduser("~/suas_targets"),
        description="Katalog wynikow; tent_target.json + podkatalog na kazdy lot",
    )
    lock_nadir_arg = DeclareLaunchArgument(
        "lock_nadir",
        default_value="true",
        description="true = node sam trzyma gimbal w nadirze (co 2 s wysyla kat)",
    )
    mount_pitch_arg = DeclareLaunchArgument(
        "mount_pitch_deg",
        default_value="-90.0",
        description="Kat gimbala; -90 = prosto w dol. Rzutowanie zaklada nadir",
    )
    cluster_radius_arg = DeclareLaunchArgument(
        "cluster_radius",
        default_value="10.0",
        description="Promien laczenia detekcji w jeden cel [m]; ~0.12 * wysokosc",
    )
    min_obs_arg = DeclareLaunchArgument(
        "min_obs",
        default_value="10",
        description="Tyle trafien musi zebrac klaster, zeby trafic do 'best'",
    )
    det_latency_arg = DeclareLaunchArgument(
        "det_latency",
        default_value="0.20",
        description="Szacowane opoznienie detekcji [s] — o tyle cofamy telemetrie",
    )
    snapshots_arg = DeclareLaunchArgument(
        "snapshots",
        default_value="true",
        description="Zapisz klatke podgladu przy kazdym nowym kandydacie",
    )

    tent_geolocator = Node(
        package="drone_autonomy",
        executable="tent_geolocator",
        name="tent_geolocator",
        parameters=[
            {
                "save_dir": LaunchConfiguration("save_dir"),
                # ParameterValue(...) bo LaunchConfiguration oddaje string,
                # a node deklaruje te parametry jako bool/double/int
                "lock_nadir": ParameterValue(
                    LaunchConfiguration("lock_nadir"), value_type=bool),
                "mount_pitch_deg": ParameterValue(
                    LaunchConfiguration("mount_pitch_deg"), value_type=float),
                "cluster_radius": ParameterValue(
                    LaunchConfiguration("cluster_radius"), value_type=float),
                "min_obs": ParameterValue(
                    LaunchConfiguration("min_obs"), value_type=int),
                "det_latency": ParameterValue(
                    LaunchConfiguration("det_latency"), value_type=float),
                "snapshots": ParameterValue(
                    LaunchConfiguration("snapshots"), value_type=bool),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            save_dir_arg,
            lock_nadir_arg,
            mount_pitch_arg,
            cluster_radius_arg,
            min_obs_arg,
            det_latency_arg,
            snapshots_arg,
            tent_geolocator,
        ]
    )
