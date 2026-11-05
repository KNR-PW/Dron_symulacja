# Kalibracja nadiru gimbala — znajdź PWM dla −90°

Narzędzie: `src/drone_autonomy/drone_autonomy/suas_gimbal_calib.py`
Mierzy w locie, pod jakim kątem kamera **naprawdę** jest, i podaje PWM do wpisania
w `drone_handler`. Nic nie zmienia na stałe.

## Przed lotem — sprawdź w Cube

- `SERVO7_FUNCTION = 0` — przy `7` ArduPilot nadpisuje `DO_SET_SERVO` i PWM **nie dochodzi do serwa**, a w logach wygląda normalnie.
- `SERVO7_MIN` — utnie PWM od dołu. Przy `1100` nie zejdziesz niżej, choćby węzeł prosił.

## Przebieg

Zostaw kontrastowy znacznik dokładnie w miejscu startu.

```bash
# terminal 1 — normalny stack, bez zmian
ros2 launch drone_bringup suas_bringup.launch.py

# terminal 2 — kalibracja
python3 ~/Dron_symulacja/src/drone_autonomy/drone_autonomy/suas_gimbal_calib.py
```

Wznieś się **pionowo** na 80 m, zawiśnij, poczekaj aż się uspokoi.
W marker_web (`:5000`) zamroź klatkę i kliknij znacznik — klasa nieważna.

```
────── POMIAR #1 ──────────────────────────────
  piksel (523, 626)   odchylka od srodka (+11, +114) px
  alt 80.0 m   roll +0.8  pitch +1.5  yaw 137
  przy PWM 1100 us punkt pod dronem rzutuje sie 9.1 m od drona
  kamera jest FIZYCZNIE na -83.50 st.  (o 6.50 st. za bardzo W PRZOD)
  reszta boczna 0.00 m  (jedna os — pitch tego nie poprawi)
  >>> PWM dla -90 st. = 1028 us (-72 od obecnego)
```

Kliknij **3–5 razy**. Od trzeciego pomiaru dostajesz gotowy blok do skopiowania
do `gimbal_pitch_callback`. Rozrzut > 30 µs = dron się bujał, powtórz.

Po wpisaniu sprawdź `pwm_min` w tej samej funkcji — przy `1100` obetnie nową wartość.

## Liczby

| | |
|---|---|
| 1° kąta | 14.2 px, **1.40 m** w terenie z 80 m |
| 1 µs PWM | 0.090° = 1.28 px — nie ma sensu szukać dokładniej niż ~5 µs |
| pitch kadłuba +2° | +28.5 px |

Ostatni wiersz to powód, dla którego węzeł nie patrzy na same piksele, tylko
przepuszcza je przez `_project_pixel` z rollem i pitchem z chwili klatki.

Błąd montażu jest **systematyczny** — zawsze w tę samą stronę względem osi drona —
więc uśrednianie po 100 obserwacjach go nie skasuje. Klaster będzie ciasny
i konsekwentnie przesunięty.

## Ręczne dojeżdżanie (opcjonalne)

```bash
ros2 param set /suas_gimbal_calib pwm 1028
ros2 topic pub --once /gimbal_calib/nudge std_msgs/msg/Int32 "{data: -10}"
ros2 topic pub --once /gimbal_calib/reset std_msgs/msg/Bool "{data: true}"
```

Sugerowane PWM nie zależy od tego, przy jakim PWM klikasz — możesz kręcić i klikać
na przemian, średnia dalej zbiega do tej samej wartości.

**Punkt poniżej środka kadru → kamera patrzy za bardzo w przód → obniż PWM.**

## Uwagi

- Węzeł publikuje `/geolocator/lock_nadir = false`, żeby geolokator nie nadpisywał
  surowego PWM. Geolokator **dalej rzutuje zakładając −90°** — i o to chodzi:
  on wierzy w −90°, my sprawdzamy czy to prawda.
- **Nie uruchamiaj równolegle `suas_gimbal_controller`** — biłby się o gimbal.
- `reszta boczna` > 1–2 m to nie robota dla PWM (gimbal ma jedną oś): albo mechanika,
  albo `cam_yaw_offset_deg` geolokatora.
- Wpisywane są **oba** punkty kalibracyjne przesunięte o tyle samo — pomiar
  jednopunktowy mierzy offset montażu, nie skalę serwa.
- `docs/gimbal setup.md` każe `SERVO7_FUNCTION = 7` i `MNT1_TYPE = 1` — to **nie
  zgadza się z kodem**, który leci surowym PWM. `SITL_param/gazebo_iris.parm` ma
  poprawnie `0` i `0`.
