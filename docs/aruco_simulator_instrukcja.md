# 🎯 Prosty Symulator ArUco - Instrukcja

## Po co to?

Testuj swoje misje ArUco **bez kamery i markerów**. 
Symulator udaje prawdziwy `aruco_node` - twój kod nie wie że to symulacja!

## Instalacja (raz)

```bash
cd ~/ros_ws
colcon build --packages-select drone_autonomy
source install/setup.bash
```

## Podstawowe użycie

### 1. Marker stoi w miejscu (środek ekranu)
```bash
ros2 run drone_autonomy aruco_simulator
```

### 2. Marker porusza się po okręgu
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle
```

### 3. Marker porusza się poziomo (linia)
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=line
```

### 4. Marker porusza się po kwadracie
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=square
```

### 5. Dodaj szum (jak prawdziwa kamera)
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle -p noise:=3.0
```

## Testowanie z twoją misją

### Terminal 1: Uruchom symulator
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle
```

### Terminal 2: Uruchom swoją misję
```bash
ros2 run drone_autonomy follow_aruco_centroid
```

## Zmiana parametrów w trakcie działania

Nie musisz restartować! Zmień parametry w locie:

```bash
# Zmień tryb
ros2 param set /aruco_simulator mode circle

# Zmień prędkość
ros2 param set /aruco_simulator speed 60.0

# Dodaj szum
ros2 param set /aruco_simulator noise 5.0

# Zmień promień okręgu
ros2 param set /aruco_simulator radius 150
```

## Podgląd danych

```bash
# Zobacz pozycje markera
ros2 topic echo /aruco_markers

# Sprawdź częstotliwość (powinno być ~30 Hz)
ros2 topic hz /aruco_markers
```

## Wszystkie parametry

| Parametr | Domyślna | Co robi |
|----------|----------|---------|
| `mode` | `'static'` | Tryb: `static`, `circle`, `line`, `square` |
| `speed` | `40.0` | Prędkość ruchu (większa = szybciej) |
| `noise` | `0.0` | Szum jak kamera (0=brak, 3=lekki, 5=duży) |
| `center_x` | `320` | Środek X (środek ekranu 640px) |
| `center_y` | `240` | Środek Y (środek ekranu 480px) |
| `radius` | `100` | Promień okręgu w pikselach |

## Przykłady użycia

### Test 1: Podstawowa weryfikacja
```bash
# Marker stoi - sprawdź czy twój kod odbiera dane
ros2 run drone_autonomy aruco_simulator
```

### Test 2: Czy dron śledzi ruchomy marker?
```bash
# Terminal 1
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle -p speed:=30.0

# Terminal 2
ros2 run drone_autonomy follow_aruco_centroid
```

### Test 3: Jak radzi sobie z szumem?
```bash
# Duży szum = trudne warunki
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=static -p noise:=8.0
```

### Test 4: Szybki ruch
```bash
ros2 run drone_autonomy aruco_simulator --ros-args -p mode:=circle -p speed:=80.0
```

## Rozwiązywanie problemów

**Nie widzę topicu `/aruco_markers`?**
```bash
# Sprawdź czy node działa
ros2 node list | grep aruco

# Sprawdź topici
ros2 topic list | grep aruco
```

**Pozycje są dziwne?**
```bash
# Reset do domyślnych
ros2 param set /aruco_simulator mode static
ros2 param set /aruco_simulator center_x 320
ros2 param set /aruco_simulator center_y 240
```

## Kluczowe informacje

✅ **Publikuje na**: `/aruco_markers` (typ: `MiddleOfAruco`)
✅ **Częstotliwość**: 30 Hz (jak prawdziwa kamera)
✅ **Kompatybilność**: 100% z istniejącym kodem
✅ **Bez zależności**: Tylko standardowy Python
✅ **Parametry w runtime**: Zmieniaj bez restartu

## Co dalej?

Jak opanujesz podstawy, możemy dodać:
- Symulację zaników sygnału (dropout)
- Więcej trajektorii
- Zapisywanie/odtwarzanie sekwencji
- Wizualizację GUI
- Launch files
- Itp.

Ale na początek - to wystarczy! 🚁

---

**Pytania? Problemy? Daj znać!**
