# Obstacle Avoidance - Unikanie Przeszkód

## Spis Treści
1. [Uruchomienie do testów](#uruchomienie-do-testów)
2. [1. Filtr](#1-filtr-)
3. [2. Safety Guard](#2-safety-guard-)
4. [Architektura](#architektura)
5. [Parametry](#parametry)
6. [Szczegóły Techniczne](#szczegóły)

## Uruchomienie do testów

#### Symulacja Webots

```bash
ros2 launch drone_bringup drone_simulation.launch.py world:=lidar_test.wbt
```

#### Obstacle Avoidance

```bash
ros2 run drone_detector simple_obstacle_avoidance
```

#### GUI do sterowania

```bash
ros2 run drone_gui gui_panel --ros-args -p obstacle_avoidance:=true
```

## 1. Filtr :
 #### Filtruje komendy prędkości (`velocity_vectors_user`) - ogranicza prędkość gdy przeszkoda jest zbyt blisko.

Aby system działał poprawnie, musisz wysyłać komendy prędkości na odpowiedni topic:

1.  **Dla Filtra Manualnego (z blokadą)**:
    - Wysyłaj na: `knr_hardware/velocity_vectors_user`
    - Node `simple_obstacle_avoidance` przefiltruje komendę i prześle bezpieczną wartość dalej.

2.  **Bezpośrednio (bez blokady)**:
    - Wysyłaj na: `knr_hardware/velocity_vectors`

 ## 2. Safety Guard :
 #### Działa w tle (np. w misjach `goto`). Jeśli wykryje, że dron autonomicznie leci w przeszkodę, automatycznie **przejmuje kontrolę, zatrzymuje drona i kończy misję**.

### Sterowanie w skrypcie misji 

Aby używać strażnika w skryptach misji (np. `test_mission.py`), użyj metody w klasie `DroneController`. 
**Implementacja**: W klasie `DroneController` dodano klienta serwisu `_set_guard_client` oraz metodę `set_obstacle_avoidance(active)`, która wysyła żądanie `SetMode` ("ON"/"OFF") do noda strażnika. Dzięki temu nie trzeba ręcznie tworzyć klienta w każdym skrypcie.

Przykład użycia wrapper'a:

```python
# Włącz strażnika przed ryzykownym przelotem
mission.set_obstacle_avoidance(True)

# Wyłącz, gdy nie jest potrzebny (np. precyzyjne lądowanie)
mission.set_obstacle_avoidance(False)

```
### Algorytm Safety Guard 

Strażnik monitoruje rzeczywistą prędkość drona (z `current_velocity`). Jeśli wykryje, że dron leci na przeszkodę z prędkością, która uniemożliwi bezpieczne wyhamowanie, podejmuje interwencję:

1. **Włącza sterowanie wektorowe** (co przerywa misję `goto`).
2. **Wysyła komendę STOP** (0,0,0).
3. **Zostawia drona w trybie V-Control** (dron wisi w miejscu).

## Architektura

```
[skrypty / misja]    [gui_panel]
        │                │
        ▼                ▼
   (goto_relative)   velocity_vectors_user
        │                │
        │         ┌──────▼──────────────────────┐
        │         │ simple_obstacle_avoidance   │
        │         │                             │◄──── /lidar
        │         │ • Filtr (Manual)            │◄──── knr_hardware/current_velocity
        │         │ • Safety Guard (Auto)       │
        │         └──────┬───────┬──────────────┘
        │                │       │
        ▼ (Mavlink)      ▼       │ (Interwencja: STOP)
   [drone_handler] ◄─────┘       │
        │                        ▼
        ▼              knr_hardware/toggle_v_control
   [ArduPilot]
```

## Parametry

Parametry można zmienić przez ROS2 parameters:

| Parametr | Domyślnie | Opis |
|----------|-----------|------|
| `tunnel_radius` | 1.0 m | Promień tunelu bezpieczeństwa. Pozwala na lot **wzdłuż** ścian. |
| `vertical_limit` | 0.75 m | Pionowy wycinek 2D (±) |
| `stop_margin` | 1.5 m | Minimalna odległość do zatrzymania |
| `braking_decel` | 0.8 m/s² | Opóźnienie hamowania. Im mniejsze, tym wcześniej zacznie hamować. |
| `max_range` | 15.0 m | Maksymalny zasięg lidaru |
| `guard_enabled` | True | Czy strażnik jest domyślnie aktywny po uruchomieniu |

## Szczegóły

### Jak działa `simple_obstacle_avoidance` z `drone_handler`?

Poniżej znajduje się szczegółowy opis przepływu danych i logiki działania systemu:

1.  **Odczyt Danych (Input)**:
    *   **Lidar**: Node `simple_obstacle_avoidance` odbiera chmurę punktów (`PointCloud2`). Przetwarza ją na wycinek 2D (slice) na wysokości drona, ignorując podłogę i sufit. Używa algorytmu "tunelowego" – sprawdza, czy w wąskim tunelu wzdłuż wektora prędkości znajduje się przeszkoda.
    *   **Prędkość Rzeczywista**: Node nasłuchuje na temat `knr_hardware/current_velocity`, który jest publikowany przez `drone_handler`. Jest to niezbędne, ponieważ w trybie autonomicznym (`goto`) dron nie otrzymuje komend od użytkownika, więc node musi znać jego faktyczny ruch, aby przewidzieć kolizję.

2.  **Safety Guard (Pętla Kontrolna)**:
    *   Strażnik oblicza **bezpieczną prędkość** ($v_{safe}$) dla aktualnej odległości od przeszkody wg wzoru fizycznego: $v_{safe} = \sqrt{2 \cdot a \cdot d}$.
    *   Porównuje aktualną prędkość drona ($v_{curr}$) z bezpieczną.
    *   Jeśli $v_{curr} > v_{safe}$ (plus margines histerezy), uznaje to za stan zagrożenia.

3.  **Interwencja (Emergency Brake)**:
    *   Gdy wykryto zagrożenie, node podejmuje decyzję o przerwaniu misji.
    *   **Krok 1: Przejęcie Kontroli**: Wysyła żądanie do serwisu `toggle_v_control` w `drone_handler`.
        *   Ważne: ArduPilot/DroneHandler ma flagę wewnętrzną `_velocity_control_flag`. Jeśli jest `False`, ignoruje komendy prędkości. Strażnik upewnia się, że ta flaga zostanie ustawiona na `True`.
    *   **Krok 2: Komenda STOP**: Node wysyła serię wiadomości `VelocityVectors(0,0,0)` na temat `knr_hardware/velocity_vectors`.
    *   **Efekt w `drone_handler`**:
        *   Odbierając `VelocityVectors` przy aktywnej fladze `_velocity_control`, handler zamienia je na komendy MAVLink `SET_POSITION_TARGET_GLOBAL_INT` (cel: prędkość 0).
        *   Te komendy MAVLink nadpisują poprzednią misję (np. `goto`), zmuszając Flight Controller do natychmiastowego zatrzymania w trybie GUIDED.
    
4.  **Po Zatrzymaniu**:
    *   Strażnik przestaje wysyłać komendy hamowania, gdy prędkość spadnie do bezpiecznego poziomu (czyli 0).
    *   Tryb sterowania wektorowego (`Velocity Control`) pozostaje włączony. Dron "wisi" w miejscu, czekając na dalsze rozkazy operatora. Misja `goto` nie jest wznawiana automatycznie.
