"""
Launch: Detekcja na Jetsonie (Real)
Oczekuje dzialajacej kamery (np. OAK-D) publikujacej na zadany topic.
Nie uruchamia mostu Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine",
        description="Sciezka do modelu TensorRT (.engine)",
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
                "input_size": 640, # Czesto 640 na Jetsonie dla szybkosci
            }
        ],
    )

    # ─── Kompresja obrazu do przesylu po Wi-Fi (stacja naziemna) ───
    image_compressor = Node(
        package="image_transport",
        executable="republish",
        name="image_compressor",
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "/tent_detections/image"),
            ("out/compressed", "/tent_detections/image_compressed"),
        ],
    )

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            camera_topic_arg,
            yolo_detector,
            image_compressor,
        ]
    )
