# 🚁 KNR Drone Mission Agent

Agent do tworzenia misji dronowych za pomocą języka naturalnego. Konwertuje polecenia tekstowe (PL/EN) na kod ROS2 kompatybilny z `DroneController`.

## 📋 Spis treści

- [Jak to działa](#-jak-to-działa)
- [Wymagania](#-wymagania)
- [Instalacja](#-instalacja)
- [Konfiguracja](#-konfiguracja)
- [Użycie](#-użycie)
- [Dostępne komendy](#-dostępne-komendy)
- [Architektura](#-architektura)
- [Przykłady](#-przykłady)
- [Troubleshooting](#-troubleshooting)

---

## 🔄 Jak to działa

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Użytkownik    │────▶│   Claude LLM    │────▶│   Kod ROS2      │
│   "leć 10m N"   │     │   (parsowanie)  │     │   .py file      │
└─────────────────┘     └─────────────────┘     └─────────────────┘
```

1. **Wpisujesz misję** w języku naturalnym (PL lub EN)
2. **Agent parsuje** tekst na listę waypointów
3. **Walidacja** sprawdza bezpieczeństwo (wysokość, dystans, lądowanie)
4. **Generuje kod** Python/ROS2 używający `DroneController`
5. **Zapisujesz** plik i uruchamiasz na dronie

---

## 📦 Wymagania

```
Python 3.10+
langchain-anthropic
langgraph
```

Oraz działający system ROS2 z pakietem `drone_comunication` (DroneController).

---

## 🔧 Instalacja

```bash
# Klonuj repo (jeśli jeszcze nie masz)
cd ~/Desktop/inzynierka

# Zainstaluj zależności
pip install langchain-anthropic langgraph
```

---

## ⚙️ Konfiguracja

### Klucz API Anthropic

Agent wymaga klucza API do Claude. Ustaw go przed uruchomieniem:

```bash
# Opcja 1: Zmienna środowiskowa (zalecane)
export ANTHROPIC_API_KEY="sk-ant-api03-twój-klucz"

# Opcja 2: Dodaj do ~/.bashrc (permanentnie)
echo 'export ANTHROPIC_API_KEY="sk-ant-api03-twój-klucz"' >> ~/.bashrc
source ~/.bashrc
```

Klucz możesz wygenerować na: https://console.anthropic.com/settings/keys

### Katalog wyjściowy

 zapisuje w bieżącym katalogu.

---

## 🚀 Użycie

### Uruchomienie

```bash
python3 mission_agent.py
```

### Interfejs

```
╔═══════════════════════════════════════════════════════════════════╗
║       🚁 KNR Drone Mission Planner (Natural Language)            ║
╠═══════════════════════════════════════════════════════════════════╣
║  Examples:                                                        ║
║  • "takeoff 10m, fly 20m north, hover 5s, land"                  ║
║  • "wystartuj 15m, leć 30m północ, obróć 90°, ląduj"             ║
╚═══════════════════════════════════════════════════════════════════╝

🎯 Mission: _
```

### Komendy CLI

| Komenda | Opis |
|---------|------|
| `help` | Wyświetla pomoc |
| `quit` / `exit` / `q` | Wyjście z programu |

---

## 🎮 Dostępne komendy misji

### Podstawowe

| Komenda | Opis | Przykład |
|---------|------|----------|
| `takeoff` | Start do wysokości | "takeoff 10m", "wystartuj 15m" |
| `land` | Lądowanie | "land", "ląduj" |
| `rtl` | Powrót do startu | "return home", "wróć" |

### Nawigacja

| Komenda | Opis | Przykład |
|---------|------|----------|
| `goto_relative` | Lot względny (NED) | "fly 20m north", "leć 10m na wschód" |
| `goto_global` | Lot do GPS | "go to GPS 52.23, 21.01" |
| `hover` | Zawis na czas | "hover 5s", "poczekaj 3 sekundy" |

### Sterowanie

| Komenda | Opis | Przykład |
|---------|------|----------|
| `set_yaw` | Obrót | "rotate 90°", "obróć 180 stopni" |
| `set_speed` | Zmiana prędkości | "set speed 5 m/s" |

### Kierunki (układ NED)

| Kierunek | Oś | Wartość |
|----------|-----|---------|
| North (północ) | N | + |
| South (południe) | N | - |
| East (wschód) | E | + |
| West (zachód) | E | - |
| Up (góra) | D | - |
| Down (dół) | D | + |

---

## 🏗️ Architektura

Agent używa **LangGraph** do budowy pipeline'u przetwarzania:

```
┌─────────────────────────────────────────────────────────────────┐
│                        LangGraph Flow                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   START                                                         │
│     │                                                           │
│     ▼                                                           │
│   ┌─────────────────┐                                          │
│   │ detect_language │  ← Wykrywa PL/EN                         │
│   └────────┬────────┘                                          │
│            ▼                                                    │
│   ┌─────────────────┐                                          │
│   │  parse_mission  │  ← LLM parsuje na waypoints              │
│   └────────┬────────┘                                          │
│            ▼                                                    │
│   ┌─────────────────┐                                          │
│   │validate_mission │  ← Sprawdza bezpieczeństwo               │
│   └────────┬────────┘                                          │
│            │                                                    │
│            ├──────────────────┐                                │
│            ▼                  ▼                                 │
│   ┌─────────────────┐  ┌─────────────┐                         │
│   │ human_review    │  │ (skip)      │  ← Jeśli ryzyko=low     │
│   │ (if risky)      │  └──────┬──────┘                         │
│   └────────┬────────┘         │                                │
│            │                  │                                 │
│            ▼                  ▼                                 │
│   ┌─────────────────────────────┐                              │
│   │    generate_ros2_code       │  ← Generuje .py              │
│   └──────────────┬──────────────┘                              │
│                  ▼                                              │
│   ┌─────────────────┐                                          │
│   │  output_result  │                                          │
│   └────────┬────────┘                                          │
│            ▼                                                    │
│           END                                                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Walidacja bezpieczeństwa

| Warunek | Poziom ryzyka | Akcja |
|---------|---------------|-------|
| Wysokość > 120m | 🔴 Critical | Wymaga potwierdzenia |
| Wysokość > 50m | 🟡 Medium | Wymaga potwierdzenia |
| Dystans > 500m | 🟡 Medium | Wymaga potwierdzenia |
| Brak lądowania | 🟡 Medium | Wymaga potwierdzenia |
| Brak takeoff | 🟡 Medium | Wymaga potwierdzenia |

---

## 📝 Przykłady

### Przykład 1: Prosty kwadrat

**Input:**
```
takeoff 5m, fly 10m north, fly 10m east, fly 10m south, fly 10m west, land
```

**Output:** `mission_20250610_143052.py`
```python
#!/usr/bin/env python3
import rclpy
import time
from drone_comunication import DroneController

def main(args=None):
    rclpy.init(args=args)
    mission = DroneController()
    
    mission.arm()
    mission.takeoff(5.0)
    time.sleep(2)
    mission.send_goto_relative(10.0, 0.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(0.0, 10.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(-10.0, 0.0, 0.0)
    time.sleep(2)
    mission.send_goto_relative(0.0, -10.0, 0.0)
    time.sleep(2)
    mission.land()
    
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

### Przykład 2: Polski z GPS

**Input:**
```
wystartuj 20m, leć do GPS 52.2297, 21.0122, poczekaj 10s, wróć do domu
```

**Output:**
```python
mission.arm()
mission.takeoff(20.0)
time.sleep(2)
mission.send_goto_global(52.2297, 21.0122, 20.0)
time.sleep(2)
time.sleep(10.0)
mission.rtl()
time.sleep(2)
```

### Przykład 3: Z obrotem

**Input:**
```
takeoff 10m, fly 15m north, rotate 180 degrees, hover 3s, land
```

**Output:**
```python
mission.arm()
mission.takeoff(10.0)
time.sleep(2)
mission.send_goto_relative(15.0, 0.0, 0.0)
time.sleep(2)
mission.send_set_yaw(3.1416, True)
time.sleep(2)
time.sleep(3.0)
mission.land()
```

---

## 🔧 Troubleshooting

### "Could not resolve authentication method"

Brak klucza API. Ustaw zmienną:
```bash
export ANTHROPIC_API_KEY="sk-ant-api03-..."
```

### "Parse failed"

LLM nie zrozumiał polecenia. Spróbuj bardziej konkretnie:
- ❌ "zrób coś fajnego"
- ✅ "takeoff 10m, fly 20m north, land"

### Dron nie reaguje na komendy (symulacja)

Użyj lekkiego świata Gazebo:
```bash
PX4_GZ_WORLD=default make px4_sitl gz_x500
```

### Plik nie zapisuje się

Sprawdź czy katalog wyjściowy istnieje:
```bash
mkdir -p ~/missions
```

---

## 📁 Struktura plików


