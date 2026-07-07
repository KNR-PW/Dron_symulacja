# Gimbal na prawdziwym dronie (sterowanie pitch przez MAVLink)

Ten dokument opisuje, jak podłączyć serwo gimbala i skonfigurować ArduPilota na
**fizycznym dronie**, żeby działało **tak samo jak w symulacji** — bez zmian w kodzie
(`drone_handler` / `suas_flight_controller` / `suas_gimbal_controller`).

## Jak to działa (sim i real identycznie)

```
tracker: set_gimbal_pitch(stopnie)
   -> ROS topic  knr_hardware/gimbal_pitch  (std_msgs/Float32)
   -> drone_handler:  MAVLink  MAV_CMD_DO_MOUNT_CONTROL (dronekit)
   -> ArduPilot: wystawia PWM na wyjscie serwa montazu
        SIM : plugin Gazebo czyta to serwo i rusza przegubem
        REAL: fizyczne serwo gimbala
```

Kod (ROS + `drone_handler`) **nie wie**, czy to sim czy real — różni się tylko to, co
siedzi za ArduPilotem. Dlatego na realu **nie zmieniamy kodu**, tylko podłączamy serwo
i ustawiamy parametry ArduPilota.

---

## 1. Podłączenie serwa (hardware)

1. **Sygnał** serwa (żółty/biały) → wolne wyjście PWM kontrolera lotu.
   U nas: **Orange Cube, MAIN 7** (= `SERVO7` w parametrach).
   > Na Cube: MAIN 1–8 = `SERVO1–8` (wyjścia IO, idealne dla serw PWM 50 Hz),
   > AUX 1–6 = `SERVO9–14` (wyjścia FMU, potrzebne tylko np. dla DShot).
   > Silniki quada siedzą na MAIN 1–4, więc MAIN 7 jest wolny i bezkonfliktowy.
2. **Zasilanie** serwa (czerwony +, czarny −) → z **BEC/UBEC** o odpowiednim napięciu
   (zwykle 5–6 V) i prądzie dla serwa. **Nie zasilaj mocnego serwa z railu FC.**
3. **Masa wspólna**: GND serwa/BEC musi być wspólny z GND kontrolera lotu.

> Na ilu osiach? Tracker używa **tylko pitch** (przechył góra/dół). Roll/yaw są opcjonalne.

---

## 2. Parametry ArduPilota (Mission Planner / QGroundControl / MAVProxy)

Ustaw i **zrestartuj** kontroler (część parametrów montażu inicjalizuje się dopiero
przy reboocie).

| Parametr | Wartość | Znaczenie |
|---|---|---|
| `MNT1_TYPE` | `1` | Typ montażu = Servo |
| `SERVO7_FUNCTION` | `7` | Wyjście MAIN 7 = **Mount1Pitch** (dopasuj numer do gniazda z kroku 1) |
| `SERVO7_MIN` | np. `1100` | PWM przy skrajnym wychyleniu (kalibracja serwa) |
| `SERVO7_MAX` | np. `1900` | PWM przy drugim skrajnym wychyleniu |
| `SERVO7_TRIM` | np. `1500` | PWM neutralny |
| `SERVO7_REVERSED` | `0`/`1` | Odwróć, jeśli serwo jedzie w złą stronę |
| `MNT1_PITCH_MIN` | np. `-90` | Dolny limit kąta pitch [deg] |
| `MNT1_PITCH_MAX` | np. `45` | Górny limit kąta pitch [deg] |
| `MNT1_DEFLT_MODE` | `2` | Domyślnie MAVLink targeting (śledzi komendy kątowe) |

> **Kalibracja kąt → PWM:** `SERVO7_MIN/MAX` dobierasz tak, żeby serwo osiągało
> fizyczne krańce gimbala, a `MNT1_PITCH_MIN/MAX` to **rzeczywiste kąty** w tych
> krańcach. Po ustawieniu sprawdź, że komenda pitch `-90°` daje **prosto w dół**,
> `0°` ≈ poziomo w przód. Jeśli kierunek odwrotny — `SERVO7_REVERSED`.

> **Uwaga (parytet z symulacją):** w sim używamy `SERVO10` i zakresu `MNT1_PITCH
> -90..90` (tam `-45°` = prosto w dół, bo plugin Gazebo ma własne `offset/multiplier`).
> Na realu kalibrujesz pod swoje serwo, więc fizyczne kąty będą „ładne" (`-90` = dół).
> **Kod się nie zmienia** — różnią się tylko te parametry per-platforma.


---

## 3. Test na realu

1. Po ustawieniu parametrów **zrestartuj** kontroler lotu (reboot).
2. Z naziemnej stacji (Mission Planner/QGC) lub MAVProxy wyślij komendę montażu i sprawdź,
   czy serwo reaguje. W MAVProxy:
   ```
   mount pitch -45
   ```
   (lub `MAV_CMD_DO_MOUNT_CONTROL` z param1 = pitch).
3. Test przez nasz ROS (to, czego używa tracker). Najpierw uruchom `drone_handler`
   podłączony do Cube (USB lub telemetria) — na realu fc_ip to port szeregowy:
   ```bash
   ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0
   ```
   a potem w drugim terminalu:
   ```bash
   ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -45.0}"
   ```
   Serwo powinno ustawić gimbal na −45°. Spróbuj `0`, `-90`.

> **Bezpieczeństwo:** cały test rób **bez śmigieł** (albo przynajmniej bez uzbrajania) —
> sterowanie gimbalem nie wymaga ARM.

Jeśli to działa — `suas_gimbal_controller` i `suas_flight_controller` działają na realu **bez żadnych
zmian w kodzie** (publikują kąt na ten sam topic).

---

## 4. Najczęstsze problemy

- **Serwo nie rusza po ustawieniu parametrów** → brak reboota kontrolera. Zrestartuj.
- **Serwo jedzie w złą stronę / inny zakres** → `SERVOx_REVERSED`, `SERVOx_MIN/MAX`.
- **Reaguje, ale kąty się nie zgadzają** → dostrój `MNT1_PITCH_MIN/MAX` do realnych krańców.
- **Nie reaguje na MAVLink, działa tylko z RC** → ustaw `MNT1_DEFLT_MODE = 2`
  (MAVLink targeting) i/lub wyślij `DO_MOUNT_CONFIGURE` na ten tryb.
- **Numer wyjścia** w `SERVOx_FUNCTION` musi odpowiadać gniazdu, do którego wpięte serwo.

---

## 5. Co po stronie kodu (bez zmian)

- `drone_handler` subskrybuje `knr_hardware/gimbal_pitch` (Float32, stopnie) i wysyła
  `MAV_CMD_DO_MOUNT_CONTROL` przez dronekit — patrz `gimbal_pitch_callback`.
- `DroneController.set_gimbal_pitch(deg)` publikuje na ten topic.
- Trackery wołają `set_gimbal_pitch(...)` (lub publikują na topic) — i to wszystko.

Czyli **jedyne, co robisz przy przejściu na realny dron, to: podłączenie serwa +
parametry ArduPilota z punktu 2.**

---

## 6. Podgląd wideo na stacji naziemnej (Laptop)

Aby uniknąć zapchania sieci Wi-Fi (łącza telemetrii) surowym obrazem z OAK-D, Jetson kompresuje obraz z narysowanymi ramkami w locie. Skrypt `suas_detect_jetson.launch.py` robi to automatycznie za pomocą węzła `image_transport`.

**Jak odebrać obraz na laptopie:**

1. Na Jetsonie musisz mieć zainstalowane wtyczki (wystarczy raz):
   ```bash
   sudo apt install ros-humble-image-transport-plugins
   ```
2. Po uruchomieniu na Jetsonie misji z kamerą (`suas_detect_jetson.launch.py`), upewnij się, że laptop i Jetson są w tej samej sieci.
3. Na laptopie uruchom standardowy podgląd ROS:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
4. W lewym górnym rogu programu rozwiń listę i wybierz skompresowany topic:
   `/tent_detections/image_compressed`

Obraz będzie płynny i nie obciąży łącza między dronem a operatorem.
