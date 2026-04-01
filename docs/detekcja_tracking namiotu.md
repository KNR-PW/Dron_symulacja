## 1. Uruchamianie (Kolejność ma znaczenie)

Najpierw uruchom detekcję (mostek GZ + YOLO):
`ros2 launch drone_bringup tent_detect.launch.py`

**Tylko detekcja YOLO (Fizyczny dron Jetson Nano)**
`ros2 run drone_detector yolo_detector_jetson`

---

## 3. Jak działa Detektor (Tent Detector)

Silnik detekcji bazuje na modelu **YOLOv8** i przemyślanej hierarchii klas:

1.  **YoloDetectorBase**: Wspólna logika (ROS 2, Ultralytics, monitorowanie FPS).
2.  **Symulacja (yolo_detector.py)**: Działa na PC, obsługuje standardowe wagi `.pt`.
3.  **Jetson (yolo_detector_jetson.py)**: Zoptymalizowany pod **TensorRT (.engine)** i półprecyzję (FP16).

**Przepływ danych:**
Obraz z kamery → Skalowanie (320px dla szybkości) → Inference YOLO → Filtracja klas (namiot/UGV) → Publikacja `TentDetection` (Bounding Box + Scoring) na temat `/tent_detections`.

---

Następnie wybierz i uruchom wersję autonomii (Follower + GUI):

*   **Wersja V1 (Namiot stacjonarny):** 
    `ros2 launch drone_bringup tent_follower_v1.launch.py`
    *Prosty regulator P, uśrednione sterowanie.*

*   **Wersja V2 (Pościg UGV / Dynamiczny):**
    `ros2 launch drone_bringup tent_follower_v2.launch.py`
    *Filtr Kalmana, predykcja 3D, precyzyjne sterowanie gimbalem.*


---

## 2. regulator podarzania za namiotem

### `tent_follower_prosty.py` (Zrzut nad obszarem statycznym)
- **Typ celu:** Statyczne lub bardzo wolne obiekty (gniazda, namioty).
- **Sterowanie:** Utrzymuje sztywną wysokość (Altitude Hold). Dron odkręca celownik po osi Yaw i "leci do przodu".
- **Zgubiony cel:** Dron zawisa w miejscu (Hover). Czeka do 5 sekund na powrót celu w ramy kamery.
- **Kamera:** Sztywny 1-osiowy sterownik (Pitch) wymuszający ruch na serwie w dół w miarę dolatywania nad dany obszar.
- **Zasilanie obrazowe:** Odbiera z YOLO małe obrabiane dane z topicu `/tent_detections` (Własna, najlżejsza struktura dla CPU).

---

### `tent_follower_v2.py`
- **Typ celu:** Uciekające, nieprzewidywalne obiekty lądowe (UGV, inne maszyny).
- **Sterowanie:** Matematyka 3D. Ekstraktuje pozycje z żyroskopu oraz akcelerometru a z pomocą trygonometrii mapuje wyliczoną wirtualną odległość terenową w metrach i wysyła stany do niezależnego kontrolera P-I-D.
- **Zgubiony cel:** Wykorzystuje zaawansowany moduł przewidywania trajektorii (**Filtr Kalmana EKF_CTRV**). Jeśli cel chowa się np. pod wiatą, dron przewiduje pozycję wektora wychodzącego.
- **Narzędzia analityczne:** Rejestruje wektory prawdziwej i estymowanej lokalizacji celu z systemu *Gazebo Ground Truth* a potem wykreśla na ekran w czasie rzeczywistym uchyby rzędu RMSE i MAE - idealne rozwiązanie do strojenia fizyki algorytmu sterowania na 100%.
- **Zasilanie obrazowe:** Analizuje globalne ramki ROS (`Detection2DArray`). Pozwala to na sortowanie np. 15 obiektów obecnych na scenie naraz i skryptowanie by celował w `id_class: 0` ignorując resztę, z pełnym zachowaniem poprawności integracji z modułami Aruco.
