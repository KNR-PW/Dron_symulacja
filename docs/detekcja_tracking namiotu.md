## 1. Uruchomienie
0. Uruchom swiat testowy np. `aruco_plain`
1. Podleć dronem (np. przez gui_panel) w rejon docelowy.
2. **Wyłącz panel GUI** (uniknięcie konfliktów komend sterowania).
3. detekcja - YOLO OpenVino, most Gazebo → ROS, rqt


   `ros2 launch drone_bringup tent_detect.launch.py`


   *(aby uruchomić detekcję na ultralytics zamiast OpenVINO, w pliku `tent_detect.launch.py` w węźle detektora podmień wartości `executable` i `name` z `"yolo_detector_OpenVino"` na `"yolo_detector_ultralitycs"`)*
   
4. śledzenie:
   `ros2 launch drone_bringup tent_tracker.launch.py`

---

## 2. Działanie (3 stany trackera)

- w tle działa niezależny regulator stałej wysokości. Sterowanie wzdłuż osi Z:
  `vz = -kp_alt * (target_alt - aktualna_wysokosc)`
- log telemetryczny zapisywany jest do pliku `~/Dron_symulacja/tent_tracker_log.csv`.

**Stan 1: CZUWANIE**
Oczekiwanie na detekcję YOLO. Gimbal wychylony w przód. Brak komend ruchu z trackera.

**Stan 2: LOT W PRZÓD**
Lot w kierunku wektora celu. Sterowanie w oparciu o pozycję ekranową namiotu:
- **Gimbal:** Kompenscja przemieszczenia obiektu po osi domykanej głowicy.

  `nowy_gimbal = stary_gimbal + (uchyb_y_kamery * kp_gimbal)`
- **Prędkość postępowa (vx):** Stopniowa redukcja prędkości wraz z pochylaniem układu gimbala (automatyczne hamowanie na końcu).

  `vx = kp_vx * (1.0 - (aktualny_kat_gimbala / max_wychylenie_gimbala))`
- **Odchylenie kierunkowe (yaw):** Wyrównywanie osi podłużnej do środka obrazu.

  `yaw = kp_yaw * uchyb_x_kamery`

Przejście następuje po osiągnięciu pełnego skrajnego odchylenia głowicy do podłoża.

**Stan 3: ZAWIS**
Pozycjonowanie centralnie nad celem. Zablokowany przegub kamery. Prędkość kątowa usztywniona (`yaw = 0`).

- (przód-tył osi x statku): 

`vx = -kp_vy * uchyb_y_kamery`
-(lewo-prawo osi y statku):

 `vy = kp_vy * uchyb_x_kamery`

Filtr układu dynamicznego (EMA) do wygładzania sterowania:
`ostateczna_predkosc_lotu = (wskaznik_wagi * swiezo_wyliczona_predkosc) + ((1 - wskaznik_wagi) * stara_predkosc)`

---

## 3. Parametry

- `target_alt` – Referencyjna wysokość lotu do wymuszenia (m).
- `kp_alt` – Wzmocnienie uchybu osi Z (redukcja błędu wysokości).
- `kp_gimbal` – Wzmocnienie błędu dla predykcji kąta pochylenia mechanizmu kamery.
- `kp_vx` – Wzmocnienie zadanej prędkości postępowej w Stanie 2.
- `kp_yaw` – Wzmocnienie naprowadzania korekty odchylenia w Stanie 2 (oś Z).
- `kp_vy` – Wzmocnienie osi dojazdowej (zarówno osi x i y) w trakcie planarnego mikro-ślizgu nad celem (Stan 3 ZAWIS).
- `ema_alpha` – Współczynnik dla filtru EMA (przedział m.in `0.0 - 1.0`). Mniejsza waga to dużo łagodniejsze przebiegi momentów 
- `lost_timeout` – Limit czasu (s) gubienia obwiedni detekcji przez YOLO, po osiągnięciu przedawnienia przechodzi do Stanu 1 (CZUWANIE).
