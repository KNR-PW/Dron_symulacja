"""
Launch: Detekcja na Jetsonie (Real)
Oczekuje dzialajacej kamery (np. OAK-D) publikujacej na zadany topic.
Nie uruchamia mostu Gazebo.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ─── Kamera OAK-D (depthai) — tryb RGB-only ──────────
    # Config wymusza pipeline RGB (bez glebi/chmury punktow) — lekkie, dziala
    # nawet po USB2. Sterownik publikuje DWA strumienie:
    #   /oak/rgb/image_raw          pelna klatka 4:3 (2028x1520 przy ISP/2)
    #   /oak/rgb/preview/image_raw  kwadrat 1024x1024 (crop bokow) — dla YOLO
    # Detektor czyta preview, bo MODEL4.engine ma sztywne wejscie 1024x1024.
    # oak_config wybiera plik z config/: oak_rgb.yaml (lot z detekcja) albo
    # oak_rgb_4k.yaml (pelna rozdzielczosc do zbierania materialu).
    oak_config_arg = DeclareLaunchArgument(
        "oak_config",
        default_value="oak_rgb.yaml",
        description="Nazwa pliku config kamery w drone_bringup/config/",
    )
    oak_rgb_config = PathJoinSubstitution([
        get_package_share_directory("drone_bringup"),
        "config",
        LaunchConfiguration("oak_config"),
    ])
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
        default_value="/oak/rgb/preview/image_raw",
        description="Topic obrazu z prawdziwej kamery (np. OAK-D)",
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value="",
        description="Filtr klas modelu: \"\" = wszystkie, \"0\" = tylko namioty, "
                    "\"1\" = tylko ludzie. Indeksy wypisuje log detektora przy starcie",
    )
    debug_every_n_arg = DeclareLaunchArgument(
        "debug_every_n",
        default_value="1",
        description="Publikuj podglad co N-ta klatke (1 = kazda)",
    )
    debug_jpeg_quality_arg = DeclareLaunchArgument(
        "debug_jpeg_quality",
        default_value="10",
        description="Jakosc JPEG podgladu (1-100); nizsza = mniej danych po Wi-Fi",
    )
    web_port_arg = DeclareLaunchArgument(
        "web_port",
        default_value="8080",
        description="Port serwera podgladu w przegladarce",
    )
    web_address_arg = DeclareLaunchArgument(
        "web_address",
        default_value="0.0.0.0",
        description="Interfejs nasluchu; 0.0.0.0 = wszystkie (Tailscale + LAN)",
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
                # value_type=str, bo "0" launch odczytalby jako INTEGER,
                # a node deklaruje ten parametr jako string ("0,1" tez ma dzialac)
                "classes": ParameterValue(
                    LaunchConfiguration("classes"), value_type=str),
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

    # ─── Podglad w przegladarce ──────────────────────────
    # http://<ip-jetsona>:8080/  — lista topicow z obrazem.
    # Nie obciaza Jetsona, dopoki nikt nie patrzy (detektor publikuje podglad
    # tylko przy aktywnej subskrypcji). Szczegoly: docs/podglad_web_kamera.md
    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        parameters=[
            {
                "port": ParameterValue(
                    LaunchConfiguration("web_port"), value_type=int),
                "address": LaunchConfiguration("web_address"),
            }
        ],
    )

    return LaunchDescription(
        [
            # oak_config musi byc przed depthai_launch — podstawia sie w sciezke configu
            oak_config_arg,
            model_path_arg,
            confidence_arg,
            classes_arg,
            camera_topic_arg,
            debug_every_n_arg,
            debug_jpeg_quality_arg,
            web_port_arg,
            web_address_arg,
            depthai_launch,     # kamera OAK-D
            yolo_detector,      # detekcja YOLO
            web_video_server,   # podglad w przegladarce
        ]
    )
