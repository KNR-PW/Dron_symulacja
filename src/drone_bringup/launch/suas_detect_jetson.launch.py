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
            }
        ],
    )

    # ─── Kompresja obrazu do przesylu po Wi-Fi (stacja naziemna) ───
    image_compressor = Node(
        package="image_transport",
        executable="republish",
        name="image_compressor",
        arguments=["raw", "compressed"],
        parameters=[{"out.jpeg_quality": 20}],
        remappings=[
            ("in", "/tent_detections/image"),
            ("out/compressed", "/tent_detections/image/compressed"),
        ],
    )

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            camera_topic_arg,
            depthai_launch,     # kamera OAK-D
            yolo_detector,      # detekcja YOLO
            image_compressor,   # kompresja obrazu do stacji naziemnej
        ]
    )
