# Instrukcja obsługi Aruco Simulator

Narzędzie służy do symulowania wykrycia markera ArUco bez konieczności uruchamiania pełnej symulacji (Webots/Gazebo) czy podłączania kamery. 

## Uruchomienie

W terminalu wykonaj polecenie:

```bash
ros2 run ros2_aruco aruco_simulator
```

## Funkcjonalność

Po uruchomieniu otworzy się okno z przyciskami sterowania. Symulator publikuje wiadomości typu `MiddleOfAruco` na temat `/aruco_markers`.

- Gdy żaden przycisk nie jest wciśnięty, marker jest "niewidoczny" (nie publikuje wiadomości).
- Po wciśnięciu kierunku, marker pojawia się w określonej, stałej pozycji (np. góra ekranu) i jest publikowany z częstotliwością 30Hz.

### Stałe pozycje markera

Symulator operuje na 4 predefiniowanych pozycjach, które zależą od skonfigurowanych wymiarów obrazu (parametry `image_width`, `image_height`) oraz marginesu (`margin`).

Dla domyślnych ustawień (640x480, margines 50px) pozycje wynoszą:
1. **Góra (W):** `(320, 50)` - środek szerokości, góra.
2. **Dół (S):** `(320, 430)` - środek szerokości, dół.
3. **Lewo (A):** `(50, 240)` - lewa strona, środek wysokości.
4. **Prawo (D):** `(590, 240)` - prawa strona, środek wysokości.

Ogólne wzory obliczania pozycji:
- **Góra:** `(width / 2, margin)`
- **Dół:** `(width / 2, height - margin)`
- **Lewo:** `(margin, height / 2)`
- **Prawo:** `(width - margin, height / 2)`
