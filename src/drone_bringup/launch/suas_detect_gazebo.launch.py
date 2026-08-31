"""
Launch: most GZ (kamera) + YOLO detektor.
Sama detekcja bez autonomii.

Detektor jest dwuklasowy i publikuje trzy rzeczy:
  /detections                  komplet boxow z klatki (obie klasy, z naglowkiem)
  /tent_detections             najpewniejszy namiot, KAZDA klatke
  /people_detections           najpewniejszy czlowiek, KAZDA klatke

Zestaw argumentow jest celowo taki sam jak w suas_detect_jetson.launch.py —
te same komendy maja dzialac w symulacji i na realu.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ─── Argumenty ────────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/root/ros_ws/src/drone_detector/models/MODEL5.pt",
        description="Sciezka do modelu YOLO",
    )
    confidence_arg = DeclareLaunchArgument(
        "conf",
        default_value="0.35",
        description="Prog pewnosci detekcji (0.0-1.0)",
    )
    camera_topic_arg = DeclareLaunchArgument(
        "camera_topic",
        default_value="/rgb_camera/image",
        description="Topic obrazu z kamery Gazebo",
    )
    classes_arg = DeclareLaunchArgument(
        "classes",
        default_value="",
        description="Filtr klas modelu: \"\" = obie klasy (kazda na wlasnym "
                    "topicu), \"0\" = tylko namioty, \"1\" = tylko ludzie. "
                    "Indeksy wypisuje log detektora przy starcie",
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
        executable="yolo_detector_ultralitycs",
        name="yolo_detector_ultralitycs",
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

    # ─── 3. Podgląd detekcji (rqt) ────────────────────────
    rqt_image_view = ExecuteProcess(
        cmd=["ros2", "run", "rqt_image_view", "rqt_image_view", "/tent_detections/image"],
        output="screen",
    )

    return LaunchDescription(
        [
            model_path_arg,
            confidence_arg,
            camera_topic_arg,
            classes_arg,
            debug_every_n_arg,
            debug_jpeg_quality_arg,
            gz_bridge,
            yolo_detector,
            rqt_image_view,
        ]
    )
