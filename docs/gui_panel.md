# Panel Sterowania: Follow ArUco Simulator
## Uruchomienie
```bash
ros2 run drone_gui gui_panel
```
Ten panel to  narzędzie do testowania autonomii (śledzenie markera) oraz ręcznej kontroli drona w trybie wektorowym oraz umożliwia uzbrajanie takeoff oraz lądowanie 

## Kluczowe Funkcje

### 1. Sterowanie Ręczne (Klawiatura)
Aby sterować klawiaturą, **kliknij w okno aplikacji**, aby złapało fokus.

### 2. Autonomia (Śledzenie Markera)
Panel posiada wbudowany regulator PID, który steruje dronem tak, aby marker ArUco znajdował się na środku kamery.
*   **START ŚLEDZENIA** - Aktywuje regulator PID. Dron zacznie podążać za wykrytym markerem.
*   **Parametry PID** - Możesz zmieniać wzmocnienia `Kp`, `Ki`, `Kd` w czasie rzeczywistym, aby dostroić reakcję drona (np. jeśli wpada w oscylacje).

### 3. Bezpiecznik: Sterowanie Wektorowe
*   Musisz go włączyć, aby latać w trybie `GUIDED` za pomocą wektorów prędkości.


