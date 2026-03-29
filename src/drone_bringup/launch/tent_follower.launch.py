"""
Launch: most GZ (kamera) + YOLO detektor + tent_follower.

Wymaga: PX4 SITL + MicroXRCEAgent + drone_handler_px4 juz uruchomione
        (np. przez ./scripts/start_sim/start_px4_sim.sh terrain)

Uruchomienie (w kontenerze):
    ros2 launch drone_bringup tent_follower.launch.py

Opcjonalne parametry:
    ros2 launch drone_bringup tent_follower.launch.py \\
        model_path:=/root/Dron_symulacja/yolo/best.onnx \\
        target_alt:=15.0 kp_xy:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/root/Dron_symulacja/yolo/best.onnx",
        description="Sciezka do modelu ONNX YOLO",
    )
    confidence_arg = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.5",
        description="Prog pewnosci detekcji (0.0-1.0)",
    )
    target_alt_arg = DeclareLaunchArgument(
        "target_alt",
        default_value="55.0",
        description="Wysokosc wzlotu [m]",
    )
    kp_xy_arg = DeclareLaunchArgument(
        "kp_xy",
        default_value="1.0",
        description="Wzmocnienie regulatora XY",
    )
    kp_yaw_arg = DeclareLaunchArgument(
        "kp_yaw",
        default_value="0.5",
        description="Wzmocnienie regulatora yaw",
    )
    max_vel_arg = DeclareLaunchArgument(
        "max_vel",
        default_value="1.5",
        description="Maks. predkosc [m/s]",
    )

    # ─── 1. Most Gazebo → ROS (obraz z kamery) ───────────
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        output="screen",
        arguments=[
            "/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
    )

    # ─── 2. YOLO detektor ────────────────────────────────
    yolo_detector = Node(
        package="drone_detector",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[
            {
                "camera_topic": "/rgb_camera/image",
                "model_path": LaunchConfiguration("model_path"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            }
        ],
    )

    # ─── 3. tent_follower ────────────────────────────────
    tent_follower = Node(
        package="drone_autonomy",
        executable="tent_follower",
        name="tent_follower",
        output="screen",
        parameters=[
            {
                "target_alt": LaunchConfiguration("target_alt"),
                "kp_xy": LaunchConfiguration("kp_xy"),
                "kp_yaw": LaunchConfiguration("kp_yaw"),
                "max_vel": LaunchConfiguration("max_vel"),
            }
        ],
    )

    # ─── 4. Podglad detekcji (rqt) ────────────────────────
    rqt_image_view = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/tent_detections/image"],
        output="screen",
    )

    return LaunchDescription(
        [
            # argumenty
            model_path_arg,
            confidence_arg,
            target_alt_arg,
            kp_xy_arg,
            kp_yaw_arg,
            max_vel_arg,
            # node'y
            gz_bridge,
            yolo_detector,
            tent_follower,
            rqt_image_view,
        ]
    )
