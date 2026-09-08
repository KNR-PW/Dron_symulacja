# Uruchomienie na realu — dwa terminale

Sciagawka na pole. Szczegoly dzialania misji: [misja.md](misja.md).
Preflight sprzetowy (serwa, gimbal, RTL_ALT): [misja_real.md](misja_real.md).

---

## 0. Raz, po zalogowaniu

```bash
ssh jetsonknr@100.84.102.43
tmux new -s suas
```

**tmux nie jest opcjonalny.** Gdy zerwie sie SSH albo Tailscale, powloka
wysyla procesom SIGHUP i gina WSZYSTKIE — nie tylko misja, ale i detektor.
Pod tmux zerwane lacze nie dociera do procesow.

Nowe okno: `Ctrl+B` `c`. Odlaczenie: `Ctrl+B` `d`. Powrot: `tmux attach -t suas`.

```bash
cd ~/Dron_symulacja
git pull
colcon build --symlink-install --packages-select \
  drone_interfaces drone_camera drone_detector drone_hardware \
  drone_bringup drone_autonomy
source install/setup.bash
```

`--symlink-install` ZAWSZE — bez tego do `install/` ida kopie i lecisz na
kodzie sprzed ostatniej zmiany. Nie mieszaj z buildem bez tej flagi.

**Po USUNIECIU albo PRZEMIANOWANIU pliku w pakiecie** wyczysc jego katalogi
budowania, inaczej colcon probuje skopiowac symlink wskazujacy donikad:

```bash
rm -rf build/<pakiet> install/<pakiet>
```

Objaw: `error: can't copy '/root/ros_ws/build/<pakiet>/...': doesn't exist`.
To NIE jest blad kodu — to stan katalogu `build/`.

---

## 1. Terminal 1 — caly stack

```bash
source ~/Dron_symulacja/install/setup.bash
ros2 launch drone_bringup suas_bringup.launch.py
```

Startuje: `drone_handler` -> kamera OAK -> YOLO -> podglad www -> GUI
oznaczania -> geolokator.

Czekaj na `Copter connected, ready to arm`.

Geolokator pisze **do repo**, tam gdzie misja szuka:

```
~/Dron_symulacja/src/drone_bringup/config/targets.json
```

To jest teraz DOMYSLNE — nie trzeba nic podawac. (Bylo `~/suas_targets`,
co bylo cichym zrodlem bledow: katalog domowy jest inny na hoscie i w
kontenerze, wiec misja czytala plik z poprzedniego lotu albo zaden.)

Obok `targets.json` geolokator zaklada katalog na KAZDY lot
(`2026-09-08_14-03/` ze zdjeciami kandydatow i `observations.csv`). Lezy to
teraz w repo, wiec `git status` bedzie je pokazywal — po locie skopiuj, co
potrzebne, i skasuj albo dopisz do `.gitignore`:

```
src/drone_bringup/config/20*/
src/drone_bringup/config/targets.json
src/drone_bringup/config/tent_target.json
```

Podglady:
- obraz z ramkami — http://100.84.102.43:8080/
- **GUI oznaczania — http://100.84.102.43:5000/** (karta musi byc otwarta,
  inaczej misja uzna, ze operatora nie ma)

---

## 2. Terminal 2 — misja

```bash
source ~/Dron_symulacja/install/setup.bash
ros2 run drone_autonomy suas_mission --ros-args \
  --params-file ~/Dron_symulacja/src/drone_bringup/config/misja.yaml
```

**Przez `ros2 run`, NIE z launcha** — misja czyta spacje z klawiatury
i potrzebuje stdin podpietego do terminala.

To okno zostaw aktywne: tu wciskasz spacje.

---

## 3. Co robisz Ty

| moment | akcja |
|---|---|
| przed startem | trasa survey w MP, uzbrojenie, start, **AUTO** |
| przelot ortofoto | **ogladasz**. Namiot zapisuje sie sam. Czlowieka klikasz w GUI (:5000) |
| koniec trasy | przelacz w MP **AUTO -> GUIDED** |
| pytanie misji | **SPACJA** = tak, ten cel. Cisza = sciezka domyslna |
| koniec | dron sam robi RTL |

**Adresy musza byc gotowe PRZED przelaczeniem na GUIDED.** `targets.json`
czytany jest RAZ, zaraz po zejsciu na 50 m. Klikniecie po przejeciu lotu
juz nie wejdzie.

Co znaczy spacja, zalezy od sytuacji — pelna tabela w [misja.md](misja.md):
- **nad waypointem, cel widoczny:** SPACJA = wycentruj i zrzuc tam;
  cisza = zrzut na wspolrzedne
- **cel znaleziony skanem:** SPACJA = podlec i zrzuc; cisza = to nie ten cel

---

## 4. Sprawdzenie w 30 sekund

```bash
ros2 topic hz /detections                    # sa detekcje?
ros2 topic echo /operator_online --once      # GUI widziane? (ma byc true)
ls -l ~/Dron_symulacja/src/drone_bringup/config/targets.json   # geo pisze?
```

W logu misji na starcie musi byc:

```
trasa skanu: /home/jetsonknr/Dron_symulacja/src/drone_bringup/config/misja_skan.waypoints
```

Jesli zamiast tego widzisz liste sprawdzonych sciezek — pliku nie ma tam,
gdzie misja go szuka.

---

## 5. Gdy cos nie gra

| objaw | co to znaczy |
|---|---|
| `Couldn't parse params file ... Error opening YAML file` | pliku NIE MA pod ta sciezka. To nie blad skladni |
| `brak ... targets.json — zadna klasa nie ma adresu` | geolokator nie wstal albo pisze gdzie indziej. Bez adresow namiot idzie skanem |
| `UWAGA: plik o tej nazwie LEZY w ...` | plik JEST, tylko pod inna sciezka (typowo w kontenerze). Misja go NIE wziela — dodaj `-p targets_json:=<ta sciezka>` |
| `targets.json: zapisany 900 s temu` | plik z POPRZEDNIEGO lotu — geolokator padl |
| misja nie pyta o spacje | karta GUI (:5000) zamknieta; misja uznaje, ze operatora nie ma |
| `error: can't copy '.../build/<pakiet>/...'` | w `build/` zostal wiszacy symlink po pliku USUNIETYM lub PRZEMIANOWANYM w `src/`. `rm -rf build/<pakiet> install/<pakiet>` i buduj jeszcze raz |
| `error: ... egg-info/PKG-INFO` przy buildzie | ogryzek po przerwanym buildzie: `find install -name "*.egg-info" -type d \| while read d; do [ -f "$d/PKG-INFO" ] \|\| rm -rf "$d"; done` |
| **dron wisi w GUIDED** | RTL z Mission Plannera. Zawsze dostepne |

`Ctrl+C` w oknie misji = przerwanie + RTL. Drugie `Ctrl+C` = twarde wyjscie
(dron zostaje w GUIDED).

---

## 6. Przed pierwszym prawdziwym zrzutem

W `config/misja.yaml` domyslnie `drop_servo_ch: 0`, czyli **zrzut tylko sie
loguje**. To celowy bezpiecznik — mozesz przelatywac caly scenariusz bez
ryzyka zgubienia ladunku.

Gdy chcesz zrzucac naprawde:

```yaml
drop_servo_ch: 13                 # AUX5
drop_pwm_by_class: [1580, 1060]   # [namiot, czlowiek]
drop_pwm_neutral: 1300
```

albo doraznie `-p drop_servo_ch:=13`.

---

## 7. Symulacja — czym sie rozni

Geolokator w Gazebo uruchamiaj OSOBNO z wlasnym katalogiem, bo domyslna
sciezka (`~/Dron_symulacja/...`) w kontenerze nie istnieje:

```bash
ros2 launch drone_bringup suas_geolocator.launch.py \
    save_dir:=/root/ros_ws/src/drone_bringup/config
```

Misja w kontenerze — **dwie** sciezki trzeba podac inaczej, bo montowany
jest tylko `src/` i `~` znaczy tam `/root`:

```bash
ros2 run drone_autonomy suas_mission --ros-args \
  --params-file /root/ros_ws/src/drone_bringup/config/misja.yaml \
  -p targets_json:=/root/ros_ws/src/drone_bringup/config/targets.json \
  -p auto_takeoff:=true
```

`auto_takeoff:=true` = misja sama uzbraja i wznosi sie, zamiast czekac na
przelaczenie AUTO -> GUIDED.

### Dlaczego `targets_json` trzeba podac RECZNIE

Trase skanu misja znajdzie sama (sprawdza kolejno `/root/ros_ws/src/...`,
`~/Dron_symulacja/src/...` i katalog `share`), ale `targets.json`
**celowo nie jest tak podmieniany**. Plik o tej nazwie w innym katalogu moze
byc z poprzedniego lotu, a zrzut pod nieaktualny adres jest gorszy niz brak
adresu — wiec misja tylko MOWI, gdzie taki plik lezy, i zostawia decyzje Tobie:

```
WARN  brak /root/Dron_symulacja/.../targets.json — zadna klasa nie ma adresu,
      namiot idzie skanem
WARN  UWAGA: plik o tej nazwie LEZY w /root/ros_ws/src/drone_bringup/config/
      targets.json. Jesli to on ma byc zrodlem adresow, popraw targets_json
```

Zobaczyles ten komunikat = misja poleciala **bez adresow**, samym skanem.
Dodaj `-p targets_json:=...` jak wyzej i uruchom jeszcze raz.

Alternatywa na stale: popraw `targets_json` w `config/misja.yaml` na sciezke
kontenerowa — ale wtedy ten sam yaml przestanie dzialac na Jetsonie.
`-p` przy uruchomieniu jest bezpieczniejsze.

### Test blednego adresu (sciezka wp_scan_on_miss)

```bash
# NA HOSCIE — wynik laduje w src/, czyli w katalogu zamontowanym w kontenerze
python3 scripts/make_targets_test.py 45
```

Robi `targets.json` z adresem namiotu przesunietym o 45 m, zeby sprawdzic, czy
dron po dolocie nie widzi celu w nadirze i odpala skan 360 st. z waypointu.
Skala bledu i co ktory oznacza — w naglowku skryptu.

Do skupienia testu na samej tej sciezce dodaj `-p search_person_after_tent:=false`,
inaczej po zrzucie dron pojdzie jeszcze szukac czlowieka.
