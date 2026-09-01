"""
Launch: zapis wspolrzednych GPS namiotu podczas przelotu ortofoto.

Sam node — kamera, detektor i drone_handler musza juz chodzic. Node ustawia
gimbal na -90 (prosto w dol), slucha /detections, /operator_mark i /knr_hardware/telemetry,
rzutuje detekcje na ziemie i grupuje je przestrzennie. Wynik: targets.json (sekcje tent i people).

Uzycie:
  ros2 launch drone_bringup suas_geolocator.launch.py
  ros2 launch drone_bringup suas_geolocator.launch.py cluster_radius:=6.0
  ros2 launch drone_bringup suas_geolocator.launch.py lock_nadir:=false

UWAGA: przy lock_nadir:=true (domyslnie) NIE uruchamiaj rownolegle
suas_gimbal_controller — biliby sie o kat gimbala.
Szczegoly: docs/suas_geolocator.md
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
        description="Tyle trafien musi zebrac klaster NAMIOTU, zeby trafic do 'best'",
    )
    min_obs_person_arg = DeclareLaunchArgument(
        "min_obs_person",
        default_value="5",
        description="To samo dla czlowieka; nizej, bo jest mniejszy i gorzej wykrywany",
    )
    person_max_alt_arg = DeclareLaunchArgument(
        "person_max_alt",
        default_value="50.0",
        description="Powyzej tej wysokosci automat NIE zapisuje czlowieka. "
                    "50 m to wysokosc zrzutu; z pulapu ortofoto (80 m) czlowiek "
                    "ma 5 px, wiec detekcje sa smieciem. "
                    "Znaczniki operatora dzialaja na kazdej wysokosci",
    )
    person_size_m_arg = DeclareLaunchArgument(
        "person_size_m",
        default_value="0.6",
        description="Rozmiar czlowieka z nadiru [m] - do bramki na rozmiar boxa",
    )
    gimbal_stabilized_arg = DeclareLaunchArgument(
        "gimbal_stabilized",
        default_value="false",
        description="false = gimbal bez stabilizacji, rzutowanie samo odejmuje "
                    "roll/pitch. true tylko przy stabilizowanym mouncie, inaczej "
                    "kompensacja policzylaby sie podwojnie",
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

    suas_geolocator = Node(
        package="drone_autonomy",
        executable="suas_geolocator",
        name="suas_geolocator",
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
                "min_obs_person": ParameterValue(
                    LaunchConfiguration("min_obs_person"), value_type=int),
                "person_max_alt": ParameterValue(
                    LaunchConfiguration("person_max_alt"), value_type=float),
                "person_size_m": ParameterValue(
                    LaunchConfiguration("person_size_m"), value_type=float),
                "gimbal_stabilized": ParameterValue(
                    LaunchConfiguration("gimbal_stabilized"), value_type=bool),
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
            min_obs_person_arg,
            person_size_m_arg,
            person_max_alt_arg,
            gimbal_stabilized_arg,
            det_latency_arg,
            snapshots_arg,
            suas_geolocator,
        ]
    )
