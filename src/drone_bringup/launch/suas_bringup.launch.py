"""
Launch: caly stack SUAS OPROCZ misji (Jetson, real).

Jeden terminal zamiast czterech z docs/plan_testow_suas.md:
  - drone_handler        (MAVLink do Orange Cube: telemetria, gimbal_pitch, serwa)
  - suas_detect_jetson   (kamera OAK-D + YOLO + web_video_server + marker_web)
  - suas_geolocator      (rzutuje /*_detections na ziemie -> ~/suas_targets/targets.json)

Misji (suas_full_mission) TU NIE MA — ona musi isc osobno przez `ros2 run`, bo
czyta klawiature (wait_confirm) i potrzebuje stdin podpietego do terminala.

Linki po starcie (zamien IP na swoj / Tailscale):
  http://<ip>:5000/   GUI recznego oznaczania celow (marker_web)
  http://<ip>:8080/   podglad topicow z obrazem (web_video_server)

Kolejnosc: handler startuje pierwszy, kamera+detekcja+geolokator z opoznieniem
detect_delay, zeby handler zdazyl zlapac FC zanim ladowanie modelu obciazy CPU.

UWAGA: geolokator domyslnie trzyma gimbal w nadirze (lock_nadir:=true). NIE
odpalaj rownolegle suas_gimbal_controller — biliby sie o kat gimbala.

Uzycie:
  ros2 launch drone_bringup suas_bringup.launch.py
  ros2 launch drone_bringup suas_bringup.launch.py fc_ip:=/dev/ttyACM0 conf:=0.30
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")

    # ─── Argumenty ────────────────────────────────────────
    fc_ip_arg = DeclareLaunchArgument(
        "fc_ip",
        default_value="/dev/ttyACM0",
        description="Port szeregowy Orange Cube (sprawdz: ls /dev/ttyACM*)",
    )
    dev_arg = DeclareLaunchArgument(
        "dev",
        default_value="false",
        description="Tryb deweloperski drone_handler",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL4.engine",
        description="Sciezka do modelu TensorRT (.engine) — zbudowany dla imgsz=1024",
    )
    confidence_arg = DeclareLaunchArgument(
        "conf",
        default_value="0.35",
        description="Prog pewnosci detekcji (0.0-1.0)",
    )
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        # Wejscie DETEKTORA: kwadratowe preview 1024x1024. To NIE jest topic
        # marker_web (ten ma wlasny domyslny /oak/rgb/image_raw/compressed).
        default_value="/oak/rgb/preview/image_raw",
        description="Topic obrazu wejsciowego dla YOLO (OAK-D preview)",
    )
    debug_jpeg_quality_arg = DeclareLaunchArgument(
        "debug_jpeg_quality",
        default_value="20",
        description="Jakosc JPEG podgladu (1-100)",
    )
    detect_delay_arg = DeclareLaunchArgument(
        "detect_delay",
        default_value="5.0",
        description="Opoznienie startu kamery/YOLO/geolokatora [s], zeby handler "
                    "zdazyl zlapac FC zanim ladowanie modelu obciazy CPU",
    )
    lock_nadir_arg = DeclareLaunchArgument(
        "lock_nadir",
        default_value="true",
        description="Geolokator sam trzyma gimbal w nadirze (co 2 s wysyla kat)",
    )
    cluster_radius_arg = DeclareLaunchArgument(
        "cluster_radius",
        default_value="10.0",
        description="Promien laczenia detekcji w jeden cel [m]; ~0.12 * wysokosc",
    )

    # ─── drone_handler (pierwszy, laczy sie z FC) ─────────
    drone_handler = Node(
        package="drone_hardware",
        executable="drone_handler",
        name="drone_handler",
        output="screen",
        parameters=[{
            "fc_ip": ParameterValue(LaunchConfiguration("fc_ip"), value_type=str),
            # value_type=str obowiazkowo: drone_handler deklaruje 'dev' jako string
            # 'false'; bez tego launch zapisalby dev jako YAML-owy bool i node
            # wywala InvalidParameterTypeException
            "dev": ParameterValue(LaunchConfiguration("dev"), value_type=str),
        }],
    )

    # ─── kamera OAK-D + YOLO + web_video_server + marker_web ──
    # Wlaczamy gotowy suas_detect_jetson.launch.py zamiast powielac jego node'y.
    detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "suas_detect_jetson.launch.py")
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "conf": LaunchConfiguration("conf"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "debug_jpeg_quality": LaunchConfiguration("debug_jpeg_quality"),
        }.items(),
    )

    # ─── geolokator (rzutuje detekcje na ziemie) ──────────
    geolocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "suas_geolocator.launch.py")
        ),
        launch_arguments={
            "lock_nadir": LaunchConfiguration("lock_nadir"),
            "cluster_radius": LaunchConfiguration("cluster_radius"),
        }.items(),
    )

    # Kamera, detekcja i geolokator startuja PO handlerze.
    delayed = TimerAction(
        period=LaunchConfiguration("detect_delay"),
        actions=[detect_launch, geolocator_launch],
    )

    return LaunchDescription([
        fc_ip_arg,
        dev_arg,
        model_path_arg,
        confidence_arg,
        camera_topic_arg,
        debug_jpeg_quality_arg,
        detect_delay_arg,
        lock_nadir_arg,
        cluster_radius_arg,
        drone_handler,     # najpierw polaczenie z FC
        delayed,           # potem kamera + detekcja + geolokator
    ])
