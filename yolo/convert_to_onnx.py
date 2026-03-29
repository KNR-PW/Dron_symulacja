"""
Skrypt konwersji modelu YOLOv8 (.pt) do formatu ONNX.

Uruchomienie (z aktywowanym .venv w folderze Dron_symulacja):
    source .venv/bin/activate
    python3 yolo/convert_to_onnx.py

Wynik: yolo/best.onnx

Wymagania:
    pip install ultralytics

Uwagi dotyczące Jetson Nano (TensorRT):
    Po wygenerowaniu best.onnx na Jetsonie uruchom:
        trtexec --onnx=yolo/best.onnx --saveEngine=yolo/best.engine --fp16
    Następnie w node'zie podmień backend cv2.dnn na TensorRT Python API.
    Alternatywnie: ultralytics obsługuje natywny eksport do TensorRT:
        model.export(format='engine', device=0)
"""

import os
from pathlib import Path

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError(
        "Zainstaluj ultralytics: pip install ultralytics\n"
        "Lub aktywuj venv: source .venv/bin/activate"
    )

SCRIPT_DIR = Path(__file__).parent
PT_PATH = SCRIPT_DIR / "best.pt"
ONNX_PATH = SCRIPT_DIR / "best.onnx"


def main():
    if not PT_PATH.exists():
        raise FileNotFoundError(f"Nie znaleziono modelu: {PT_PATH}")

    print(f"Wczytywanie modelu: {PT_PATH}")
    model = YOLO(str(PT_PATH))

    print(f"Eksport do ONNX (opset 12, imgsz=1024)...")
    exported_path = model.export(
        format="onnx",
        imgsz=1024,
        opset=12,
        simplify=True,
        dynamic=False,
    )

    print(f"Model wyeksportowany do: {exported_path}")
    print(f"Klasy modelu: {model.names}")
    print(
        "\nGotowe! Skopiuj plik best.onnx do kontenera lub użyj ścieżki bezwzględnej "
        "w parametrze 'model_path' node'a yolo_detector."
    )


if __name__ == "__main__":
    main()
