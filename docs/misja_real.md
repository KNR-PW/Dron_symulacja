# Misja SUAS na realu — przygotowanie i przebieg

Dokument roboczy pod test w polu. Stan: **przygotowanie, uzupelniamy przed lotem.**
Wszystko ponizej jest przetestowane w symulacji; na realu nietestowane sa
rzeczy oznaczone jako **[NIESPRAWDZONE NA REALU]**.

---

## 1. Do uzupelnienia PRZED lotem — blokujace

Bez tych trzech rzeczy nie ma sensu wozic drona na pole.

### 1.1 Kanaly i PWM serw zrzutu

JEDEN serwomechanizm na oba ladunki (u nas AUX5 = **kanal 13**): wychyla sie
w dwie strony wzgledem neutralnej — jedna strona zwalnia namiot, druga czlowieka,
neutral trzyma oba. Domyslnie `drop_servo_ch: 0`, a **kanal 0 znaczy „tylko log,
nic nie jedzie na sprzet"** — celowy bezpiecznik: misja przeleci caly scenariusz
i wypisze `ZRZUT [SYMULACJA]` zamiast zwolnic ladunek.

Do `config/suas_mission.yaml` przed lotem:

```yaml
    drop_servo_ch: 13                # <- PRAWDZIWY kanal serwa (AUX5 = 13)
    drop_pwm_by_class: [1602, 988]   # <- PWM zrzutu [namiot, czlowiek]
    drop_pwm_neutral: 1327           # <- pozycja spoczynkowa (trzyma oba)
    drop_hold_s: 1.0                 # ile trzymac wychylenie
```

PWM zmierz na stanowisku (`ros2 service call /knr_hardware/set_servo
drone_interfaces/srv/SetServo "{servo_id: 13, pwm: 1500}"`). U nas: 988 =
czlowiek, 1327 = neutral, 1602 = namiot.

Uwaga: u nas `SERVO13_FUNCTION = RCIN11` (sterowanie z radia przelacznikiem) i
`DO_SET_SERVO` z misji i tak dziala — ale RC passthrough moze nadpisac pozycje,
jesli przelacznik na radiu stoi w pozycji zrzutu. Trzymaj przelacznik w neutralu
podczas lotu autonomicznego (albo ustaw `SERVO13_FUNCTION = 0`, jesli rezygnujesz
z recznego zrzutu).

**Test na ziemi, przed lotem** (dron rozbrojony, ladunki zaladowane):

```bash
ros2 service call /knr_hardware/set_servo drone_interfaces/srv/SetServo \
    "{servo_id: 9, pwm: 1900}"      # ma zwolnic
ros2 service call /knr_hardware/set_servo drone_interfaces/srv/SetServo \
    "{servo_id: 9, pwm: 1100}"      # ma wrocic
```

### 1.2 Potwierdzenie konfiguracji gimbala

Cale rzutowanie zaklada, ze gimbal **nie jest stabilizowany** i ze pochylenia
ramy trzeba odjac rachunkiem. W symulacji jest `MNT1_TYPE=0`, `SERVO7_FUNCTION=0`.

Sprawdz w Mission Plannerze te dwa parametry na realnym dronie:

- **zgadzaja sie** -> nic nie robimy,
- **jest stabilizowany mount** (`MNT1_TYPE=1`, `SERVO7_FUNCTION=7`) -> ustaw
  `gimbal_stabilized: true` w geolokatorze, inaczej kompensacja policzy sie
  **podwojnie** i bedzie gorzej niz bez niej.

Sprawdz tez `RTL_ALT` — RTL najpierw wznosi sie na te wysokosc.

### 1.3 Kadr kamery a wykrywalnosc czlowieka

Obecny config (`oak_rgb.yaml`, `focal_px=813`, VFOV 64,4 st.):

| wysokosc | slad kadru | namiot 3 m | czlowiek 0,5 m |
|---|---|---|---|
| 80 m (ortofoto) | 101 m | 30 px | **5 px — automat nie znajdzie** |
| 50 m (zrzut) | 63 m | 49 px | **8 px — automat nie znajdzie** |

Czyli **czlowieka oznaczasz recznie w GUI**, a automat lapie namiot. Geolokator
ma na to zabezpieczenie: `person_max_alt` (50 m) — automat nie zapisuje czlowieka
z pulapu ortofoto, bo tam ma 5 px i bylyby to same falszywki. Klikniecia
operatora nie maja tego limitu.

Jesli chcesz, zeby automat lapal tez czlowieka, trzeba zejsc na natywny crop
z matrycy (patrz `docs/oak_kadr_detekcji.md`) — **[NIESPRAWDZONE NA REALU]**,
nie na jutro.

---

## 2. Uruchamianie — wszystko pod tmux

**To nie jest opcjonalne na realu.** Gdy zerwie sie SSH albo Tailscale, powloka
wysyla procesom `SIGHUP` i **gina wszystkie** — nie tylko misja, ale i detektor
z geolokatorem. Misja zostalaby wtedy bez detekcji, a dron zawisl w miejscu
w GUIDED, az dasz RTL z Mission Plannera.

`tmux` trzyma pseudoterminal po stronie Jetsona, wiec zerwane SSH nie dociera do
procesow. Po ponownym zalogowaniu wracasz do tej samej sesji.

```bash
ssh jetsonknr@100.84.102.43
tmux new -s suas
```

W jednej sesji, kolejne okna przez `Ctrl+B` `c`:

```bash
# okno 0 — drone_handler
ros2 run drone_hardware drone_handler --ros-args -p fc_ip:=/dev/ttyACM0

# okno 1 — kamera + detekcja + podglad www
ros2 launch drone_bringup suas_detect_jetson.launch.py

# okno 2 — geolokator
ros2 launch drone_bringup suas_geolocator.launch.py

# okno 3 — GUI oznaczania          -> http://100.84.102.43:5000
ros2 run drone_autonomy suas_marker_web

# okno 4 — MISJA (tu wciskasz spacje)
ros2 run drone_autonomy suas_full_mission \
    --ros-args --params-file ~/Dron_symulacja/src/drone_bringup/config/suas_mission.yaml
```

`Ctrl+B` `d` = odlacz sie, wszystko chodzi dalej.
`tmux attach -t suas` = powrot. `Ctrl+B` + numer = przelaczanie okien.

Jesli na Jetsonie nie ma tmux: `sudo apt install tmux`, albo `screen -S suas`
(odlaczenie `Ctrl+A` `d`, powrot `screen -r suas`).

---

## 3. Przebieg misji — co robisz Ty

| moment | Twoja akcja |
|---|---|
| przed startem | wgraj trase survey w MP, uzbroj, wystartuj, AUTO |
| przelot ortofoto | **ogladasz**. Namiot zapisuje sie sam. Czlowieka klikasz w GUI: LPM zamraza klatke, LPM na obiekcie, wybierz `CZLOWIEK` |
| koniec trasy | przelacz **AUTO -> GUIDED** w MP |
| cel 1 | **spacja** w oknie misji (albo nic — wtedy "to nie ten cel": nastepny kandydat, a gdy brak — grid) |
| cel 2 | to samo |
| koniec | dron sam robi RTL |

Po przelaczeniu na GUIDED misja: schodzi na `target_alt` (50 m) -> leci nad cel
-> czeka `acquire_timeout` na potwierdzenie detekcji -> pyta Cie o spacje ->
centruje -> zrzuca -> nastepny cel -> RTL.

Cele czytane sa z `~/suas_targets/targets.json` **przy kazdym wejsciu w GUIDED**.
Czyli jesli klikniesz cos po przejeciu lotu: wyjdz z GUIDED, kliknij, wroc do
GUIDED — misja wezmie nowy cel.

---

## 4. Co sie stanie, gdy cos pojdzie nie tak

| sytuacja | zachowanie |
|---|---|
| **zerwane SSH / Tailscale** | pod tmux: misja leci dalej sama. Puls z GUI znika w ~5 s, misja **przestaje pytac** i zrzuca na najlepszym kandydacie kazdej klasy. **Bez tmux: procesy gina, dron wisi w GUIDED** — trzeba dac RTL z MP |
| **nie potwierdzisz spacja (a patrzysz)** | to znaczy "nie ten cel": kandydat wypada, dron leci do nastepnego wg score, a gdy kandydatow brak — grid bez konca (konczysz spacja albo Ctrl+C) |
| **detektor nie widzi celu po dolocie** | okno akwizycji -> zamiatanie gimbalem (jesli `sweep_enabled`) -> spirala (`spiral_timeout` 45 s) -> **zrzut na waypoint** |
| **cel zgubiony w trakcie centrowania** | gimbal ZOSTAJE w pionie, cel wraca przy pierwszej dobrej klatce. Jesli zniknal na dobre: `approach_timeout` (90 s), potem zrzut na wspolrzedne |
| **brak waypointu dla klasy** | grid nad zadanym obszarem; nic nie znajdzie -> **RTL z ladunkiem na pokladzie** |
| **Ctrl+C w oknie misji** | przerwanie + RTL (obsluzone). Drugie Ctrl+C = twarde wyjscie |
| **padnie sam Jetson** | ArduPilot w GUIDED bez setpointow zatrzymuje sie i wisi. Ratunek: RTL z MP albo `FS_GCS_ENABLE` |

---

## 5. Znane wady i ograniczenia

### 5.1 Zgubienie celu w trakcie centrowania odwraca kamere — NAPRAWIONE

Gdy detekcja sie przeterminuje, kontroler wchodzi w `SEARCH` i przestawia gimbal
na `pitch_search` = **-55 st.** To ma sens w fazie dolotu, gdzie -55 patrzy przed
siebie i pomaga znalezc cel.

Ale w finalnym centrowaniu **wisisz nad celem**, czyli cel jest w nadirze.
Gimbal -55 odchyla os kamery o 35 st. od pionu, a polowa FOV to 32,2 st. — punkt
pod dronem wypada **poza kadrem**. Kamera odjezdza dokladnie od tego, czego szuka,
i przez pozostale ~85 s nie ma szans go odzyskac.

Skutek: chwilowe zgubienie celu marnuje caly `approach_timeout` i konczy sie
zrzutem na waypoint zamiast nad wycentrowanym celem.

Poprawka: parametr `search_gimbal_on_lost` (domyslnie `true`), ktory
`center_over_target` ustawia na `false` — analogicznie do istniejacego
`fresh=False`. Przy okazji `approach_timeout` moze zejsc z 90 s do 30-40 s.

### 5.2 Misja nie lapie SIGHUP — NAPRAWIONE

Obslugiwane sa `SIGINT` i `SIGTERM`. `SIGHUP` (zerwany terminal) zabija proces
**bez RTL**. tmux to obchodzi, ale nie jest to samo co naprawa.

Poprawka: dopisac `signal.SIGHUP` do `_install_signals` — jedna linijka, zamienia
„dron wisi w nieskonczonosc" na „dron wraca do domu".

### 5.3 Wysokosc jest nad punktem uzbrojenia, nie nad terenem

`Telemetry.alt` to `global_relative_frame`. Na pochylym polu skala rzutowania
(GSD = alt/focal) jest przeklamana o roznice terenu, a „50 m zrzutu" to 50 m nad
miejscem startu, nie nad celem. Przy polu 150x250 m zwykle 1-2 m bledu.

### 5.4 Symulacja nie waliduje detekcji

Wszystkie testy w Gazebo dowodza, ze **logika i geometria** sa poprawne — nie ze
cel zostanie wykryty w polu. Gazebowa postac to prymitywna siatka, a model
trenowano na prawdziwych zdjeciach. Jutro pierwszy raz sprawdzamy detekcje na
prawdziwym materiale.

---

## 6. Preflight — do odhaczenia na miejscu

- [ ] `git pull` i `colcon build` na Jetsonie, `source install/setup.bash`
- [ ] `drone_handler` mowi `Copter connected, ready to arm`
- [ ] `ros2 topic hz /detections` — sa detekcje, hz zgodne z FPS kamery
- [ ] podglad w przegladarce dziala: `http://100.84.102.43:8080/`
- [ ] GUI oznaczania dziala: `http://100.84.102.43:5000` — LPM zamraza klatke
- [ ] test serw zrzutu na ziemi (punkt 1.1), ladunki zaladowane po tescie
- [ ] `MNT1_TYPE`, `SERVO7_FUNCTION`, `RTL_ALT` sprawdzone w MP (punkt 1.2)
- [ ] gimbal reaguje: `ros2 topic pub --once /knr_hardware/gimbal_pitch std_msgs/msg/Float32 "{data: -90.0}"`
- [ ] geolokator startuje z `lock_nadir: true` (domyslnie) — misja sama zwolni blokade przy przejeciu, nic nie ustawiaj recznie
- [ ] trasa survey wgrana do drona, `FS_GCS_ENABLE` ustawione
- [ ] wszystko odpalone **pod tmux**, sesja odlaczona i sprawdzone, ze zyje
- [ ] `~/suas_targets/` istnieje i jest zapisywalne

---

## 7. Po locie — co zabrac

Katalog lotu `~/suas_targets/<data-godzina>/`:

- `targets.json` — cele z tego lotu
- `observations.csv` — **kazda przyjeta detekcja**: piksel, wysokosc, katy,
  rzutowany punkt. Z tego da sie odtworzyc geometrie i policzyc, gdzie byl blad
- `kandydat_NN_<klasa>.jpg` — klatka przy kazdym nowym kandydacie

Plus log CSV kontrolera i cala sesja tmux (`tmux capture-pane -p -S -`).
