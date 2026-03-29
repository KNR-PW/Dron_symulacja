"""
Launch file uruchamiajacy most Gazebo->ROS (obraz z kamery) i node wykrywania namiotow.

Uruchomienie:
    ros2 launch drone_detector yolo_detector.launch.py

Opcjonalne nadpisanie parametrow:
    ros2 launch drone_detector yolo_detector.launch.py \
        model_path:=/root/Dron_symulacja/yolo/best.onnx \
        confidence_threshold:=0.5 \
        camera_topic:=/rgb_camera/image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/root/Dron_symulacja/yolo/best.onnx",
        description="Sciezka do modelu ONNX (best.onnx)",
    )

    confidence_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Prog pewnosci detekcji (0.0 - 1.0)",
    )

    nms_arg = DeclareLaunchArgument(
        "nms_threshold",
        default_value="0.45",
        description="Prog NMS",
    )

    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/rgb_camera/image",
        description="Topic z obrazem z kamery",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        output="screen",
        arguments=[
            "/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    yolo_detector = Node(
        package="drone_detector",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "model_path": LaunchConfiguration("model_path"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "nms_threshold": LaunchConfiguration("nms_threshold"),
            }
        ],
    )

    rqt_image_view = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/tent_detections/image"],
        output="screen",
    )

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            nms_arg,
            camera_topic_arg,
            gz_bridge,
            yolo_detector,
            rqt_image_view,
        ]
    )
