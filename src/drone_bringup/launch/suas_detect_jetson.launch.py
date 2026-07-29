"""
Launch: Detekcja na Jetsonie (Real)
Oczekuje dzialajacej kamery (np. OAK-D) publikujacej na zadany topic.
Nie uruchamia mostu Gazebo.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ─── Kamera OAK-D (depthai) — tryb RGB-only ──────────
    # Publikuje obraz na /oak/rgb/image_raw. Config wymusza pipeline RGB
    # (bez glebi/chmury punktow) — lekkie, dziala nawet po USB2.
    oak_rgb_config = os.path.join(
        get_package_share_directory("drone_bringup"),
        "config",
        "oak_rgb.yaml",
    )
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("depthai_ros_driver"),
                "launch",
                "camera.launch.py",
            )
        ),
        launch_arguments={"params_file": oak_rgb_config}.items(),
    )

    # ─── Argumenty ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL4.engine",
        description="Sciezka do modelu TensorRT (.engine) — musi byc zbudowany dla imgsz=1024",
    )
    confidence_arg = DeclareLaunchArgument(
        "conf",
        default_value="0.35",
        description="Prog pewnosci detekcji (0.0-1.0)",
    )
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/oak/rgb/image_raw",
        description="Topic obrazu z prawdziwej kamery (np. OAK-D)",
    )
    debug_every_n_arg = DeclareLaunchArgument(
        "debug_every_n",
        default_value="1",
        description="Publikuj podglad co N-ta klatke (1 = kazda)",
    )
    debug_jpeg_quality_arg = DeclareLaunchArgument(
        "debug_jpeg_quality",
        default_value="20",
        description="Jakosc JPEG podgladu (1-100); nizsza = mniej danych po Wi-Fi",
    )

    # ─── YOLO detektor (Jetson) ──────────────────────────
    yolo_detector = Node(
        package="drone_detector",
        executable="yolo_detector_jetson",
        name="yolo_detector_jetson",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "conf": LaunchConfiguration("conf"),
                "input_size": 1024,
                # ParameterValue(int) bo LaunchConfiguration oddaje string,
                # a node deklaruje te parametry jako int
                "debug_every_n": ParameterValue(
                    LaunchConfiguration("debug_every_n"), value_type=int),
                "debug_jpeg_quality": ParameterValue(
                    LaunchConfiguration("debug_jpeg_quality"), value_type=int),
            }
        ],
    )

    # Kompresja do /tent_detections/image/compressed leci teraz w samym detektorze
    # (cv2.imencode). Osobny node republish wymagal wyslania raw bgr8 960x720
    # (2.1 MB/klatke) miedzy procesami — to bylo drozsze niz sam JPEG.

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            camera_topic_arg,
            debug_every_n_arg,
            debug_jpeg_quality_arg,
            depthai_launch,     # kamera OAK-D
            yolo_detector,      # detekcja YOLO
        ]
    )
