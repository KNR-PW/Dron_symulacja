"""
Launch: Detekcja na Jetsonie (Real, PoE) — YOLO na strumieniu z OAK-D PoE.

UWAGA: depthai_ros_driver NIE laczy sie z nasza kamera OAK-D-PRO-W-POE
(znany bug, patrz testy_kamery/OAK_NA_JETSONIE.md). Zamiast tego uzywamy
wlasnego node'a drone_camera/oak_publisher, ktory omija drivera i publikuje
kompatybilny /oak/rgb/image_raw.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
        description="Topic obrazu z kamery OAK-D (PoE)",
    )
    oak_ip_arg = DeclareLaunchArgument(
        "oak_ip",
        default_value="169.254.1.222",
        description="IP kamery OAK-D PoE",
    )

    # ─── Kamera OAK-D PoE (wlasny publisher, bypass depthai_ros_driver) ───
    oak_node = Node(
        package="drone_camera",
        executable="oak_publisher",
        name="oak_publisher",
        parameters=[
            {
                "ip": LaunchConfiguration("oak_ip"),
                "fps": 30,
                "width": 1920,
                "height": 1080,
                "frame_id": "oak_rgb_camera_optical_frame",
            }
        ],
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
            oak_ip_arg,
            oak_node,
            yolo_detector,
            image_compressor,
        ]
    )
