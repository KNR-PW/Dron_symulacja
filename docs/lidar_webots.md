# Konfiguracja Symulacji lidaru 3D

## Zasada Działania
LIDAR działa na zasadzie wysyłania impulsów światła laserowego i mierzenia czasu ich powrotu (Time of Flight). Na podstawie tego czasu oblicza odległość od przeszkody.
W tej symulacji sensor skanuje **dolną półkulę** (360° dookoła i 90° w dół)
## Uruchomienie


```bash
ros2 launch drone_bringup drone_simulation.launch.py world:=lidar_test.wbt
```

### Specyfikacja Danych Wyjściowych

*   **Temat ROS:** `/lidar`
*   **Typ Wiadomości:** `sensor_msgs/msg/PointCloud2`
*   **Frame ID:** `lidar_link`
*   **Częstotliwość:** Zgodna z parametrem `--lidar-fps` (domyślnie 10 Hz)

### Struktura Punktów (PointCloud2)
Chmura punktów jest przesyłana w formacie binarnym z następującymi polami:
1.  **x** (float32): Pozycja przód/tył [m]
2.  **y** (float32): Pozycja lewo/prawo [m]
3.  **z** (float32): Wysokość względem sensora [m]

> ℹ️ **Refleksyjność (Intensity):** Obecnie symulacja w Webots **nie udostępnia** informacji o refleksyjności (intensywności) odbicia. Dane zawierają tylko geometrię (X, Y, Z). Symulacja "Lidar 4D" (z intensywnością) wymagałaby dodatkowej implementacji (np. fuzji z kamerą), która nie została tu wykonana.

> ℹ️ **Symulacja Lidaru się nie kręci**
> Mimo że rzeczywisty Unitree L2 posiada wirującą głowicę, w symulacji Webots jest użyty model statycznego (`type "fixed"`). Daje to **natychmiastowy skan 360°** w każdej klatce,

## Zmodfikowane pliki do obsługi symulacji lidaru

| Plik | Opis |
|------|------|
| `webots_vehicle_ros.py` | **Mostek Webots-ROS.** Pobiera surowe dane z symulatora (kamera, lidar), przetwarza je i publikuje na tematy ROS (m.in. `/lidar`). |
| `ardupilot_sitl_controller.py` | **Główny kontroler drona.** Inicjuje pojazd w Webots, parsuje argumenty (np. `--lidar`, `--lidar-fps`) i uruchamia mostek ROS. |
| `lidar_test.wbt` | **Plik świata (Scena).** Definiuje fizyczną konfigurację drona, w tym parametry lidaru (zasięg, rozdzielczość, pozycja na dronie). | 

---


## Konfiguracja parametrów - plik Swiata webots (.wbt)

```
DEF Iris Iris {
  translation 0 0 0.13
  controller "ardupilot_sitl_controller"
  controllerArgs [
    # ...
    # ...
    "--lidar-fps"          # <-- OPCJONALNA ZMIANA
    "5"
  ]
  extensionSlot [
    Lidar {
      translation 0 0 -0.18       # pozycja względem środka drona (x,y,z)
      rotation 0 0 1 0            # orientacja
      name "Lidar"                # musi pasować do --lidar
      horizontalResolution 180    # liczba promieni w poziomie <-- OPCJONALNA ZMIANA
      fieldOfView 6.28318         # 360° (2π radianów)
      verticalFieldOfView 3.14159 # 180° (π radianów) - półkula
      numberOfLayers 30           # liczba warstw pionowych <-- OPCJONALNA ZMIANA
      minRange 0.05                # minimalna odległość [m] 
      maxRange 30                 # maksymalna odległość [m]
      noise 0.02                  # szum pomiarowy [m] (symulacja niedoskonałości sensora)
    }
    # ... reszta sensorów (kamera, GPS, etc.)
  ]
}
```
> ⚠️ **Uwaga**: W Webots **nie udało się skonfigurować lidaru tak, aby skanował tylko dolną półkulę**. Dlatego zastosowano obejście - w kodzie dodano filtr `if p.z > 0: continue`, który odrzuca punkty z górnej półkuli. Jeśli ustawisz `numberOfLayers = 30`, efektywnie tylko **15 warstw** będzie używanych (dolna połowa).



