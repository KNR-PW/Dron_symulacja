
# Coverage Node – Mapowanie terenu (ROS 2)

## Opis
`coverage_node` to node ROS 2 służący do **automatycznego mapowania terenu dronem**.  
Na podstawie **4 punktów GPS** definiujących obszar generuje trasę typu **lawnmower (zygzak)** i wysyła ją do autopilota jako **waypointy GPS (GotoGlobal)**.

Node został zaprojektowany pod:
- mapowanie / ortofotomapę,
- testy w symulacji (PX4 / ArduPilot + QGroundControl),
- zadania typu *Risk Mapping*.

---

## Jak działa (w skrócie)

1. Użytkownik podaje **4 punkty GPS** (`lat,lon`) – obszar mapowania.
2. Node:
   - przelicza punkty GPS → lokalny układ metryczny (E, N),
   - generuje trasę lawnmower z zadanym odstępem pasów,
   - przelicza trasę z powrotem na GPS.
3. Dron:
   - armuje się,
   - startuje na stałą wysokość,
   - leci po waypointach GPS, pokrywając cały obszar.

---

## Założenia techniczne

- Planowanie odbywa się w **metrycznym układzie lokalnym**.
- Wykonanie odbywa się przez **GotoGlobal (GPS)**.
- Dla obszarów < ~1 km stosowane jest **lokalne przybliżenie geograficzne**:
  - bez Haversine,
  - bez UTM,
  - bez kalibracji ruchem.

To podejście jest **wystarczająco dokładne do mapowania i ortofoto**.

---

## Wysokość lotu

Wysokość lotu jest zdefiniowana jako stała:

```python
ALTITUDE = 30.0  # metry AGL
````

Ta sama wysokość jest używana:

* przy starcie (`takeoff`),
* przy wszystkich waypointach (`send_goto_global`).

---

## Uruchomienie

### 1. Zbuduj workspace

```bash
colcon build --packages-select drone_autonomy
source install/setup.bash
```

### 2. Uruchom symulację i QGroundControl

(np. `drone_simulation.launch.py`)

### 3. Uruchom node

```bash
ros2 run drone_autonomy coverage_node \
  --p1 LAT1,LON1 \
  --p2 LAT2,LON2 \
  --p3 LAT3,LON3 \
  --p4 LAT4,LON4 \
  --step 14 \
  --angle 0
```

---

## Parametry CLI

| Parametr       | Opis                                    |
| -------------- | --------------------------------------- |
| `--p1 .. --p4` | Cztery punkty GPS obszaru (lat,lon)     |
| `--step`       | Odstęp między pasami lawnmowera (metry) |
| `--angle`      | Rotacja trasy (stopnie)                 |

---

## Testowanie

* Trasa jest widoczna w **QGroundControl**.
* Poprawne działanie:

  * równoległe pasy,
  * pełne pokrycie obszaru,
  * brak „dryfu” względem mapy.

---

## Dlaczego GotoGlobal?

* zgodność z mapą i ortofotomapą,
* brak zależności od yaw i lokalnych błędów,
* łatwa weryfikacja w QGroundControl.

---

## Zastosowania

* mapowanie terenu,
* generowanie danych do ODM / MicMac,
* zadania typu *Search / Risk Mapping*,
* testy algorytmów coverage planning.

---

## Uwagi

* Kolejność punktów powinna tworzyć poprawny wielokąt (np. zgodnie z ruchem wskazówek zegara).
* Dla pierwszych testów zaleca się większy `step` (np. 25 m).
* Node **nie zajmuje się robieniem zdjęć** – tylko nawigacją.

```

---

ać **diagra