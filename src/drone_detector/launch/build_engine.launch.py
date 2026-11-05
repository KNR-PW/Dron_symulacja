"""
Launch: budowa silnika TensorRT (.engine) z modelu .pt.

Jednorazowe narzedzie, nie node — uruchamia eksport z ultralytics i konczy sie.
Plik .engine ladnie obok zrodlowego .pt (MODEL5.pt -> MODEL5.engine).

MUSI byc uruchomiony NA JETSONIE. Silnik TensorRT jest kompilowany pod
konkretny sprzet i wersje TensorRT — engine zbudowany na laptopie nie ruszy
na Jetsonie i odwrotnie.

Uzycie:
  ros2 launch drone_detector build_engine.launch.py
  ros2 launch drone_detector build_engine.launch.py model:=/sciezka/do/INNY.pt

Potem detekcja z nowym modelem:
  ros2 launch drone_bringup suas_detect_jetson.launch.py \
      model_path:=/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL5.engine
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="/home/jetsonknr/Dron_symulacja/src/drone_detector/models/MODEL5.pt",
        description="Sciezka do modelu zrodlowego .pt",
    )
    imgsz_arg = DeclareLaunchArgument(
        "imgsz",
        default_value="1024",
        description="Rozmiar wejscia. MUSI zgadzac sie z input_size detektora "
                    "i z preview kamery (1024x1024)",
    )
    half_arg = DeclareLaunchArgument(
        "half",
        default_value="True",
        description="FP16 — szybsze na Jetsonie, tak zbudowany jest MODEL4",
    )

    export = ExecuteProcess(
        cmd=[
            "yolo", "export",
            ["model=", LaunchConfiguration("model")],
            "format=engine",
            ["imgsz=", LaunchConfiguration("imgsz")],
            ["half=", LaunchConfiguration("half")],
            "device=0",
        ],
        output="screen",
    )

    return LaunchDescription([model_arg, imgsz_arg, half_arg, export])
