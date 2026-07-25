# Gimbal — skrócona instrukcja

## Podłączenie
1. Sygnał serwa → **MAIN 7** na Cube.
2. Zasilanie serwa → **BEC 5 V** wpięty w rail MAIN (+/−). Cube sam nie zasila railu — bez BEC-a serwo dostanie sygnał, ale nie ruszy.
3. Masa BEC-a wspólna z Cube (załatwia to wpięcie w rail).

## Parametry (Mission Planner)

| Parametr | Wartość | Po co |
|---|---|---|
| `MNT1_TYPE` | 1 | Włącza obsługę gimbala (typ: serwo) |
| `SERVO7_FUNCTION` | 7 | Pin MAIN 7 = pitch gimbala |
| `SERVO7_MIN/MAX` | ~1100/1900 | Zakres PWM = fizyczne krańce gimbala |
| `SERVO7_TRIM` | ~1500 | Pozycja neutralna |
| `SERVO7_REVERSED` | 0/1 | Jedzie w złą stronę → zmień |
| `MNT1_PITCH_MIN/MAX` | −90 / 45 | Jakie kąty odpowiadają krańcom |
| `MNT1_DEFLT_MODE` | 2 | Słuchaj komend MAVLink (tego używa nasz kod) |

**Po zapisaniu → reboot Cube'a.** Bez restartu gimbal nie działa — to najczęstszy błąd.

## Kalibracja (bez śmigieł)
1. Wyślij pitch `0` (MP/MAVProxy: `mount pitch 0`) → kamera poziomo. Nie jest → koryguj `TRIM` lub przełóż orczyk.
2. Wyślij `-90` → prosto w dół. Jedzie w górę → `SERVO7_REVERSED=1`.
3. Na krańcach serwo buczy/dociska → zawęź `MIN/MAX` o 20–50 µs.
4. `MNT1_PITCH_MIN/MAX` ustaw na rzeczywiste kąty osiągane na krańcach.

## Test przez ROS
```bash
ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0
```
Drugi terminal:
```bash
ros2 topic pub --once knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -45.0}"
```
Serwo ma stanąć na −45°. Sprawdź też `0` i `-90`. Działa → tracker działa bez zmian w kodzie.

## Gdy coś nie gra
- **Nic nie rusza** → brak reboota albo brak zasilania railu (BEC).
- **PWM na ch7 zmienia się (Status → servo outputs), serwo stoi** → zasilanie/kabel serwa.
- **PWM stoi w miejscu** → parametry (`SERVO7_FUNCTION`, `MNT1_TYPE`, tryb MAVLink).
- **Kąty się nie zgadzają** → `MNT1_PITCH_MIN/MAX`.
- **Reaguje na RC, nie na kod** → `MNT1_DEFLT_MODE=2`.

Kalibruj **po finalnym montażu mechanicznym** — przełożenie orczyka później unieważnia wszystko.