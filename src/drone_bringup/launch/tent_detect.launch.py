"""
Launch: most GZ (kamera) + YOLO detektor.
Sama detekcja bez autonomii.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/root/Dron_symulacja/yolo/best_openvino_model",
        description="Sciezka do modelu YOLO",
    )
    confidence_arg = DeclareLaunchArgument(
        "conf",
        default_value="0.5",
        description="Prog pewnosci detekcji (0.0-1.0)",
    )

    # ─── 1. Most Gazebo → ROS (obraz z kamery) ───────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        arguments=[
            "/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    # ─── 2. YOLO detektor ────────────────────────────────
    yolo_detector = Node(
        package="drone_detector",
        executable="yolo_detector_OpenVino",
        name="yolo_detector_OpenVino",
        parameters=[
            {
                "camera_topic": "/rgb_camera/image",
                "model_path": LaunchConfiguration("model_path"),
                "conf": LaunchConfiguration("conf"),
            }
        ],
    )

    # ─── 3. Podgląd detekcji (rqt) ────────────────────────
    rqt_image_view = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/tent_detections/image"],
        output="screen",
    )

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            gz_bridge,
            yolo_detector,
            rqt_image_view,
        ]
    )


