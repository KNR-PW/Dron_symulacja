"""
Launch: drone_handler + kamera OAK-D + detekcja YOLO (SUAS, Jetson).

Laczy T1 i T2 z docs/plan_testow_suas.md w jednym terminalu:
  - drone_handler          (MAVLink do Orange Cube, telemetria, gimbal_pitch)
  - suas_detect_jetson     (kamera OAK-D + YOLO -> /tent_detections)

Gimbal (T3) zostaje osobno — odpalasz go dopiero gdy /tent_detections publikuje:
  ros2 launch drone_bringup suas_gimbal_controller.launch.py img_h:=1024 vfov_deg:=64.4
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
        # Kwadratowe preview 1024x1024 — osobny topic niz /oak/rgb/image_raw,
        # ktory oddaje pelna klatke 4:3 (2028x1520). Musi byc taki sam jak
        # domyslny w suas_detect_jetson.launch.py, bo ta wartosc go nadpisuje.
        default_value="/oak/rgb/preview/image_raw",
        description="Topic obrazu z kamery OAK-D",
    )
    debug_every_n_arg = DeclareLaunchArgument(
        "debug_every_n",
        default_value="1",
        description="Publikuj podglad co N-ta klatke (1 = kazda)",
    )
    debug_jpeg_quality_arg = DeclareLaunchArgument(
        "debug_jpeg_quality",
        default_value="20",
        description="Jakosc JPEG podgladu (1-100)",
    )
    detect_delay_arg = DeclareLaunchArgument(
        "detect_delay",
        default_value="5.0",
        description="Opoznienie startu kamery i YOLO [s], zeby handler zdazyl "
                    "zlapac FC zanim ladowanie modelu obciazy CPU",
    )

    # ─── T1: drone_handler ────────────────────────────────
    drone_handler = Node(
        package="drone_hardware",
        executable="drone_handler",
        name="drone_handler",
        output="screen",
        parameters=[{
            "fc_ip": ParameterValue(LaunchConfiguration("fc_ip"), value_type=str),
            # value_type=str obowiazkowo: drone_handler deklaruje 'dev' jako string
            # 'false', a launch bez tego zapisalby dev:=true jako YAML-owy bool
            # i node wywala InvalidParameterTypeException
            "dev": ParameterValue(LaunchConfiguration("dev"), value_type=str),
        }],
    )

    # ─── T2: kamera OAK-D + YOLO ──────────────────────────
    # Wlaczamy gotowy suas_detect_jetson.launch.py zamiast powielac jego node'y —
    # kamera, config oak_rgb.yaml i detektor zostaja w jednym miejscu.
    detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "suas_detect_jetson.launch.py")
        ),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "conf": LaunchConfiguration("conf"),
            "camera_topic": LaunchConfiguration("camera_topic"),
            "debug_every_n": LaunchConfiguration("debug_every_n"),
            "debug_jpeg_quality": LaunchConfiguration("debug_jpeg_quality"),
        }.items(),
    )

    detect_delayed = TimerAction(
        period=LaunchConfiguration("detect_delay"),
        actions=[detect_launch],
    )

    return LaunchDescription([
        fc_ip_arg,
        dev_arg,
        model_path_arg,
        confidence_arg,
        camera_topic_arg,
        debug_every_n_arg,
        debug_jpeg_quality_arg,
        detect_delay_arg,
        drone_handler,     # najpierw polaczenie z FC
        detect_delayed,    # potem kamera + detekcja
    ])
