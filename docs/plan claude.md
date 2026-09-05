# Plan: pełna misja SUAS — detekcja dwuklasowa, geolokalizacja, zrzuty

## Kontekst

Ortofotomapa 80 m nad polem 150×250 m leci wg trasy z Mission Plannera, robiona
**osobną kamerą, której w ROS nie ma**. W tym czasie OAK patrzy w dół i zbierane
są pozycje GPS **namiotu (class 0)** i **człowieka (class 1)**. Po trasie operator
przełącza AUTO→GUIDED, ROS przejmuje lot i wykonuje dwa zrzuty z **50 m** (niżej
nie wolno), po czym RTL.

Dziś to niemożliwe: detektor publikuje **jeden** box na klatkę
([yolo_detector_base.py:100](src/drone_detector/drone_detector/yolo_detector_base.py#L100))
i `TentDetection` nie ma pola klasy, więc namiot zawsze wygrywa z człowiekiem.
Geolokator zna jedną klasę i rzutuje piksel zakładając kamerę w idealnym nadirze.
`suas_simple_mission` (działa w realu) startuje z zawisu i nie umie ani przejąć
lotu po trasie AUTO, ani zejść i zrzucić ładunku.

`suas_simple_mission` i `suas_gimbal_controller` zostają **nietknięte** jako
znany-dobry fallback. Wszystko nowe powstaje obok.

---

# CZĘŚĆ I — jak to działa dla operatora

Oglądasz jednocześnie **stronę w przeglądarce** (obraz z OAK) i **terminal
z logami misji**. Zasada nadrzędna: oglądasz, klikasz rzadko. Każde Twoje
działanie tylko **nadpisuje** automat, nigdy nie jest warunkiem, żeby misja
poszła dalej. Gdy padnie łącze, dron bierze wszystko z automatu i kończy sam.

## 1. Podczas ortofoto (dron w AUTO, prowadzi Mission Planner)

W tle: **namiot wykrywa się sam** (68 px na 80 m) — geolokator zbiera obserwacje
i klastruje, nic nie musisz robić. **Człowieka automat nie znajdzie** — ma 11 px,
poniżej progu YOLO. Tu wchodzisz Ty.

```
   PPM gdziekolwiek   ->  obraz ZAMARZA (aktualna klatka, pelna jakosc)
   PPM na obiekcie    ->  male menu w miejscu kliku:  NAMIOT | CZLOWIEK
   klik w klase       ->  zapisane, podglad WRACA na zywo sam
```

Trzy kliknięcia, zero pisania. `Esc` albo LPM w puste miejsce = anuluj.
**Pomyłkę poprawiasz klikając jeszcze raz** — nowy klik tej samej klasy zastępuje
poprzedni. Na dole jedna linijka statusu: `namiot: ✓ operator | człowiek: —`,
żebyś widział, czy klik się zapisał.

Bez cofania po klatkach, bez listy, bez historii. Strona ma być szybka.

**Kliknięcie ma zawsze pierwszeństwo nad automatem.** Jeśli oznaczysz namiot
gdzie indziej niż znalazł go detektor, dron leci tam, gdzie kliknąłeś.

**Dlaczego najpierw zamrożenie, potem klik:** klikając w obraz na żywo trafiałbyś
w kadr sprzed sekundy, a serwer dostałby najnowszą klatkę — piksel wskazywałby co
innego. Zamrożona klatka niesie własny `stamp`, więc rzutowanie bierze telemetrię
z dokładnie tej chwili. Błąd opóźnienia znika, zostaje sama dokładność
rzutowania: **1–3 m**.

## 2. Przełącznik AUTO → GUIDED uruchamia całą resztę

To jedyny sygnał, jakiego misja potrzebuje. Dopóki jest AUTO, ROS jest fizycznie
bezsilny — ArduPilot ignoruje setpointy prędkości w tym trybie.

Po przełączeniu, w tej kolejności:

1. **zejście na 50 m** — natychmiast, przed czymkolwiek innym,
2. **obrót nosem na cel**, potem lot prosto do przodu (nie bokiem),
3. dalej wg scenariuszy z punktu 4.

Gimbal w locie do celu stoi w pozycji szukania **-55°**, więc cel może zostać
złapany już w trakcie dolotu.

Kolejność celów: bliższy pierwszy.

## 3. Nad celem — jedno potwierdzenie w terminalu

Dron dolatuje nad waypoint, zawisa na 50 m i przez **5 s** czeka, aż detektor
potwierdzi klasę — tym samym mechanizmem M trafień z ostatnich N klatek, który
już masz w `suas_flight_controller`. Jeśli potwierdzi, w terminalu pojawia się:

```
=== CEL 1/2: NAMIOT ===
Potwierdzony w kadrze (conf 0.71, 3.2 m od waypointu).
[SPACJA] = podejdz i wycentruj      [nic] = zrzut na waypoint za 15 s
```

- **wciśniesz spację** → centrowanie wizualne i zrzut dokładnie nad obiektem
- **nie zrobisz nic** → po 15 s zrzut na współrzędne, bez centrowania

Domyślne działanie jest bezpieczne: bez Ciebie dron nie goni detekcji, która może
być fałszywa. Przy padniętym SSH zawsze idzie ścieżką domyślną.

## 4. Trzy scenariusze

| sytuacja | co robi dron |
|---|---|
| **A. Jest waypoint, detektor potwierdził w oknie 5 s** | pyta w terminalu → spacja: centrowanie i zrzut nad obiektem; nic: zrzut na waypoint |
| **B. Jest waypoint, okno 5 s minęło bez potwierdzenia** | **zamiatanie gimbalem** (-90 → -55 w krokach, dron stoi) → jeśli nic: spirala wokół waypointu (skok 20 m, limit 45 s) → znajdzie: jak A; nie znajdzie: **zrzut na waypoint** |
| **C. Nie ma waypointu** | grid nad zadanym obszarem na 50 m → pierwsza potwierdzona detekcja: zawis i pytanie; brak spacji: ignoruje i szuka dalej. **Grid bez trafienia → RTL z ładunkiem** |

Reguła w jednym zdaniu: **jest adres → zrzut zawsze; nie ma adresu i nic nie
znaleziono → RTL.** Tak przeczytałem Twoje dwa zdania („zrzuca na wskazany
waypoint" i „jak nic nie wykryje to rób RTL"); gdyby miało być inaczej, to jeden
parametr.

Grid dla brakującej klasy startuje **po zrzucie na pierwszy obiekt**, nie przed.

## 5. Ile razy działasz w całej misji

| moment | akcja |
|---|---|
| przelot ortofoto | 3 kliknięcia na człowieka, 0 na namiot |
| koniec trasy | 1 przełącznik AUTO→GUIDED w MP |
| cel 1 | 1 spacja (albo nic) |
| cel 2 | 1 spacja (albo nic) |

**Razem ok. 6 akcji**, z czego żadna nie jest wymagana.

---

# CZĘŚĆ II — kamera, geometria, znaleziska

## Kamera

OAK jest pasażerem: trasę wyznacza kamera ortofoto, więc FOV OAK-a dobieramy
swobodnie, pod wykrywalność. Kamera daje **jeden** strumień 1024×1024; crop robi
samo urządzenie (`setVideoSize` w depthai — `oak_publisher.py` już tego używa),
więc przez USB idzie tyle samo danych **niezależnie od rozdzielczości ISP**.
Zacinanie przy `oak_rgb_4k.yaml` brało się z `i_output_isp: true`, czyli
z wysyłania całej klatki 4056×3040 (~19 MB), nie z samej matrycy.

Podgląd na żywo zostaje na `jpeg_quality: 8` — to zmierzona granica łącza.
Zamrożona klatka idzie osobno, w q90 (~200 KB, raz na żądanie).

Wybór kadru to jedna liczba — handel śladem za piksele na celu:

| ISP | `focal_px` | ślad @80 m | ślad @50 m | namiot @80 m | człowiek @80 m | człowiek @50 m |
|---|---|---|---|---|---|---|
| dziś (skalowany wide) | 813 | 101 m | 63 m | 30 px | 5 px ✘ | 8 px ✘ |
| 1/2 → 2028×1520 | 1207 | 68 m | 42 m | 45 px | 7 px ✘ | 12 px |
| **3/4 → 3042×2280 ← tymczasowo** | **1810** | **45 m** | **28 m** | **68 px** | **11 px** | **18 px** |
| 1/1 → 4056×3040 | 2414 | 34 m | 21 m | 91 px | 15 px | 24 px |

### Otwarta liczba: odstęp galsów

Ślad OAK-a musi pokryć odstęp galsów, inaczej między pasami zostają dziury.
Odstęp wynika z kamery ortofoto (Surveyor 24L v2, 6000×4000) — **czekam na
rozmiar matrycy i ogniskową.** `6000×4000` pasuje i do APS-C 24 MP (23,5×15,6 mm),
i do pełnej klatki 24 MP (35,8×23,9 mm), a to różnica 1,5×.

```
ślad_w_poprzek = 80 m × wymiar_matrycy_w_poprzek / ogniskowa
odstęp galsów  = ślad_w_poprzek × 0,4          (sidelap 60%)
```

| matryca | 16 mm | 20 mm | 24 mm | 35 mm |
|---|---|---|---|---|
| APS-C 23,5×15,6 | odstęp 47 m | 38 m | 31 m | 21 m |
| pełna klatka 35,8×23,9 | 72 m | 57 m | 48 m | 33 m |

Dla APS-C `focal_px=1810` (ślad 45 m) wystarcza wszędzie poza skrajnym 16 mm.
Dla pełnej klatki z krótką ogniskową trzeba zejść na 1509 albo 1207, płacąc
pikselami na człowieku. **Jedna liczba w configu, nie blokuje żadnego etapu.**

## Znaleziska z kodu

**1. Gimbal nie jest stabilizowany.** `SITL_param/gazebo_iris.parm`: `MNT1_TYPE=0`,
`SERVO7_FUNCTION=0`; [drone_handler.py:593](src/drone_hardware/drone_hardware/drone_handler.py#L593)
steruje surowym `DO_SET_SERVO`. Kamera pochyla się z ramą. Na 80 m przy stałym
pitchu 10° (lot 8 m/s) to **14 m błędu** rzutowania. Bramka `max_tilt=10°`
w geolokatorze odrzuca dziś prawie każdą klatkę z przelotu — to jedyny powód,
dla którego problem jeszcze nie wybuchł.

**2. AUTO/GUIDED jest wymuszone przez firmware.** `velocity_control_callback`
wysyła `send_global_velocity`, które ArduPilot ignoruje w AUTO.

**3. Brak serwisu `dropper`** — `Dropper.srv` istnieje, serwera nie ma nigdzie
w `src/`. Idziemy przez `knr_hardware/set_servo`, zero nowego kodu w handlerze.

**4. `GotoGlobal.action` ma lat/lon jako `float32`** (~1 m kwantyzacji). Zmieniamy
na `float64` — i tak przebudowujemy `drone_interfaces`.

**5. Powód wolnego centrowania.** W `HOVER` uchyb jest **znormalizowany**:
`vx = -kp_vy * ey`, `kp_vy=1.0`. Ten sam `ey=0.2` to 3,8 m na 30 m i 10 m na 80 m —
wzmocnienie zależy od wysokości, a w ogonie zbiegania prędkość spada do
centymetrów na sekundę. Przeliczenie uchybu na **metry** to usuwa.

**6. Flask + rclpy w jednym procesie już działa** w `drone_web/app.py`
(MultiThreadedExecutor + `Response` z MJPEG). GUI budujemy na tym wzorcu.

---

## Jak to się uruchamia — dwa terminale

Wszystko poza misją nie potrzebuje klawiatury, więc idzie do jednego launcha.
Misja musi być `ros2 run`, bo potrzebuje stdin na spację — węzeł odpalony
z launcha nie ma terminala i raw `termios` rzuciłby wyjątkiem.

```bash
# T1  percepcja                     ->  GUI na http://<ip>:5000
ros2 launch drone_bringup suas_field.launch.py
#   -> drone_handler, detektor, web_video_server, geolokator, GUI markera

# T2  cala misja: etapy, sterowanie, monit o spacje
ros2 run drone_autonomy suas_full_mission \
    --ros-args --params-file ~/Dron_symulacja/src/drone_bringup/config/suas_mission.yaml
```

**Co widać gdzie:**

| | T1 | T2 |
|---|---|---|
| detektor: FPS i detekcje per klasa | ✓ | |
| geolokator: kandydaci, lat/lon, odrzucenia | ✓ | |
| etapy misji | | ✓ |
| sterowanie: stan, `ex`/`ey`, gimbal, prędkości | | ✓ |
| monit o potwierdzenie (spacja) | | ✓ |

Podział jest naturalny: T1 to **co dron widzi**, T2 to **co dron robi**.
Sterowanie i etapy siedzą w jednym węźle (`suas_full_mission` dziedziczy po
`SuasFlightController`), więc lądują w tym samym terminalu bez żadnych zabiegów.

Parametry misji siedzą w `config/suas_mission.yaml`, żeby komenda w T2 została
krótka mimo kilkunastu nastaw. Na realu to dwa okna SSH; w symulacji dochodzą
Gazebo i SITL z `scripts/start_sim/`.

---

# CZĘŚĆ III — etapy

Każdy etap ma jedno mierzalne kryterium. Nigdy nie testujesz dwóch nowych rzeczy
naraz. Ja piszę kod, Ty uruchamiasz i mierzysz.

## Etap 0 — świat testowy

- Nowy `gazebo/worlds/suas_field.sdf` na bazie `terrain_tent_overcast.sdf`:
  namiot (`model://tent`) i postać (`model://Standing person`) ~80 m od siebie,
  plus drzewa jako źródło false positives. **Pozycje modeli zapisane w komentarzu** —
  to prawda odniesienia do mierzenia błędu.
- W `gimbal_small_3d/model.sdf`: `horizontal_fov` 1.123992 → **0.5548** (31,8°),
  czyli optycznie natywny crop przy ISP 3/4 (`focal_px=1810`).
- `scripts/run_ap_gazeboo_sitl.sh` + `--out=udp:172.17.0.1:14550` dla MP.

**Kryterium:** świat wstaje, w MP widać drona, da się wgrać trasę survey.

## Etap 1 — detektor dwuklasowy

**Interfejsy:** `TentDetection.msg` + `int32 class_id`; nowy `TentDetections.msg`
(`Header` + `TentDetection[]`); nowy `OperatorMark.msg`; `GotoGlobal.action`
lat/lon → `float64`.

```
# OperatorMark.msg
std_msgs/Header header   # stamp = stamp KLATKI, z ktorej kliknieto
uint8 class_id           # 0 = namiot, 1 = czlowiek
float32 u                # piksel w oryginalnej klatce
float32 v
```

**`yolo_detector_base.py`** — jedna inferencja, trzy wyjścia:
- `/detections` (`TentDetections`) — wszystkie boxy + `header.stamp` z klatki
- `/tent_detections` — najpewniejszy `class_id==0`, **każdą klatkę** (także pusty)
- `/people_detections` — to samo dla `class_id==1`
- overlay: wszystkie boxy w kolorach per klasa; log FPS per klasa

`/tent_detections` zachowuje dotychczasową semantykę, więc okno potwierdzania
w `suas_flight_controller._det_cb` działa **bez zmiany linijki**.

**Kryterium:** w kadrze z namiotem i człowiekiem `/detections` ma dwa wpisy
o różnych `class_id`, oba topici per-klasa publikują z hz kamery.

```bash
colcon build --packages-select drone_interfaces drone_detector drone_bringup drone_autonomy
ros2 launch drone_bringup suas_detect_gazebo.launch.py conf:=0.25
ros2 topic echo /detections --once ; ros2 topic hz /tent_detections /people_detections
```

> `install/` to kopie, nie symlinki — bez `colcon build` zmiany w launchach nie zadziałają.

## Etap 2 — geolokator dwuklasowy i poprawne rzutowanie

**2a.** `git mv tent_geolocator.py suas_geolocator.py` — sam rename, zero logiki.

**2b. Poprawne rzutowanie — najważniejsza zmiana w planie.** Kierunek promienia
z `(roll, pitch, yaw)` + kąt montażu, przecięcie z płaszczyzną ziemi. Znika błąd
14 m przy locie z prędkością; `max_tilt` idzie z 10° na ~25°, więc z przelotu
zostaje wielokrotnie więcej obserwacji. Funkcja przyjmuje **dowolny piksel** —
tej samej użyje kliknięcie operatora.

**2c. Dwie klasy + znaczniki operatora.** `Candidate` + `class_id`, dopasowanie
tylko w obrębie klasy, bramka rozmiaru per klasa (`tent_size_m=3.0`,
`person_size_m=0.6`), wejście `/detections`, subskrypcja `/operator_mark`
(rzutowanie `(u,v)` telemetrią z `header.stamp` — bufor `_telem_at` już istnieje).
**Priorytet: operator > automat**, znacznik ręczny jest `best` niezależnie od
`n_obs`. Wyjście `~/suas_targets/targets.json` z sekcjami `tent` i `people`.

**Testujesz:** przelot prostą 8 m/s na 80 m nad namiotem. **Przed 2b** błąd
względem prawdy ze świata rzędu 10–15 m, **po 2b** poniżej ~3 m. Plus znacznik
ręczny z konsoli: `ros2 topic pub --once /operator_mark ...`.

**Kryterium:** namiot z przelotu < 3 m błędu.

## Etap 3 — GUI oznaczania (lekkie)

Nowy węzeł `suas_marker_web`: Flask + rclpy w jednym procesie, jeden plik HTML,
bez frameworka, bez JS-owych bibliotek.

**Zero obciążenia Jetsona w spoczynku — to jest główne kryterium projektowe:**

- węzeł **nie subskrybuje obrazu w ogóle**. Podgląd na żywo to `<img>` wskazujący
  wprost na istniejący `web_video_server`, więc przez Flaska nie przechodzi ani
  jedna klatka wideo;
- PPM robi **jednorazowy** odbiór jednej klatki z topicu kamery
  (`wait_for_message`, subskrypcja tworzona i niszczona na miejscu), jeden
  `cv2.imencode` q90 i koniec;
- strona **nie odpytuje serwera w pętli** — status pobiera raz, po zapisaniu kliku;
- w spoczynku proces śpi na `accept()`.

**Łapiemy surowy obraz z kamery (1024×1024), nie podgląd detektora** — ten jest
przeskalowany do 960 px (`DEBUG_MAX_WIDTH`), więc piksele nie przekładałyby się
1:1 i klik trafiałby obok.

**Endpointy:** `GET /` (statyczny HTML) · `GET /grab` (jedna klatka q90 + jej
`stamp`) · `POST /mark` `{u, v, class}` → publikacja `OperatorMark` ·
`GET /status` (dwie linijki: zapisane cele). Nic więcej.

**Węzeł nie subskrybuje niczego.** Ani obrazu, ani telemetrii, ani statusu misji.
Ma jeden publisher (`OperatorMark`) i jednorazowy odbiór klatki przy PPM.
W spoczynku nie robi dosłownie nic.

Cała matematyka rzutowania zostaje w geolokatorze — GUI podaje tylko piksel
i stamp. Jeśli GUI padnie albo nikt go nie otworzy, misja leci na automacie.

**Kryterium:** klikasz postać w zamrożonej klatce, `people.best` w `targets.json`
jest < 5 m od prawdziwej pozycji modelu. Ponowny klik w inne miejsce nadpisuje.
Z zamkniętą stroną obciążenie węzła w `top` jest nieodróżnialne od zera.

## Etap 4 — przejęcie lotu

Misja czeka, aż `Telemetry.flight_mode` zmieni się `AUTO` → `GUIDED`, i dopiero
wtedy cokolwiek wysyła. Bez zmian w `drone_handler`. `takeover_timeout` (20 min)
→ RTL, żeby zawieszona misja nie wisiała w nieskończoność.

**`targets.json` odczytywany przy KAŻDYM wejściu w GUIDED** (zbocze narastające
`flight_mode`), nie raz na starcie. Dzięki temu klik wykonany po przejęciu lotu
wchodzi do gry: wychodzisz z GUIDED, klikasz, wracasz do GUIDED — misja bierze
nowy cel.

**Kryterium:** w AUTO misja nie drgnie; po przełączeniu rusza natychmiast.

## Etap 5 — centrowanie w metrach, zejście, zrzut, potwierdzanie

Rozbudowa `suas_flight_controller.py` (klasa bazowa — `suas_simple_mission`
skorzysta za darmo).

- **`detection_topic`** jako parametr (dziś zaszyte `/tent_detections`
  w [suas_flight_controller.py:178](src/drone_autonomy/drone_autonomy/suas_flight_controller.py#L178)).
- **Uchyb w metrach:** `err_m = e * (img/2) * alt / focal_px`; `hover_deadzone_m`
  (0,5 m), `center_tol_m` (1,0 m). To jest naprawa „za długo się centruje".
- **`vfov_deg` 64,4 → 31,8, `focal_px` 813 → 1810** w `suas_flight_controller`,
  `suas_gimbal_controller` i launchach. Obowiązkowe przy zmianie kadru — inaczej
  korekta gimbala jest 2× za duża i wpada w oscylacje (`docs/oak_kadr_detekcji.md`).
- **Zejście na 50 m natychmiast po GUIDED**, przed przelotem — przez istniejącą
  pętlę altitude-hold (`self.target_alt`).
- **Lot nosem do celu, nie bokiem:** przed `send_goto_global` liczymy namiar
  z bieżącego GPS na cel i ustawiamy kurs przez istniejące `send_set_yaw(bearing,
  relative=False)`, dopiero potem lecimy. Nie polegamy na `WP_YAW_BEHAVIOR`
  w ArduPilocie, żeby zachowanie było takie samo w SITL i na realu. Finalne
  centrowanie zostaje translacyjne (bez obrotu) — tam obracanie tylko psuje.
- **`drop(idx)`**: jeden serwomechanizm `drop_servo_ch`, `set_servo` na
  `drop_pwm_by_class[idx]` (per klasa) → pauza → powrot na `drop_pwm_neutral`.
  W symulacji (`drop_servo_ch=0`) log + zapis GPS.
  Zrzut następuje w zawisie, więc prędkość pozioma jest już w martwej strefie
  regulatora — nie dokładamy osobnego warunku zatrzymania.
- **`pitch_search` -45 → -55** w `suas_flight_controller`, `suas_gimbal_controller`
  i launchach. Ten kąt działa tylko wtedy, gdy dron **nic nie widzi** — po
  pierwszej świeżej detekcji gimbal i tak koryguje się na cel, a `vx` narasta
  przez EMA. Bardziej w dół = lepsze pokrycie terenu tuż pod dronem i wokół
  waypointu, czyli tam, gdzie cel faktycznie ma być. `pitch_max` bez zmian.
- **Okno akwizycji:** po ustabilizowaniu na 50 m nad waypointem `acquire_timeout`
  (5 s) na potwierdzenie klasy istniejącym oknem M z N klatek. Potwierdzi →
  scenariusz A, nie → zamiatanie gimbalem.
- **Zamiatanie gimbalem** (`sweep_pitches`, domyślnie `[-90, -75, -65, -55]`,
  `sweep_dwell` 2 s na krok): dron stoi, gimbal przejeżdża zakres. Na 50 m
  odpowiada to punktom na ziemi 0 / 13 / 23 / 35 m przed dronem, czyli skanuje
  pas wzdłuż nosa **bez ruszania maszyną** — szybciej i taniej niż spirala.
  Rzutowanie detekcji z pochylonego gimbala jest już poprawne dzięki etapowi 2b.
  Dopiero gdy to nic nie da, wchodzi spirala.
- **Bramka potwierdzenia:** `wait_confirm(prompt, timeout)` — nasłuch klawisza
  w terminalu (raw `termios`), `confirm_timeout` 15 s. Timeout zwraca `False`
  i misja idzie ścieżką domyślną. Tylko terminal — GUI zostaje minimalne.

**Testujesz:** `suas_simple_mission` nad namiotem — dosuwanie od HOVER do
`center_tol_m` ma trwać krócej i nie oscylować. Potem `target_alt:=50`. Potem
**pomiar, który decyduje o całej misji:** zawis nad postacią na 50 m — czy przy
18 px YOLO w ogóle publikuje `class_id=1`.

**Kryterium:** centrowanie < 15 s bez przelotu przez cel; twarda odpowiedź, czy
zrzut na człowieka będzie wizualny, czy z GPS.

## Etap 6 — `suas_full_mission`

Nowy `drone_autonomy/suas_full_mission.py`, dziedziczy po `SuasFlightController`
(jak `suas_simple_mission`), plus launch. Realizuje Część I punkty 2–4.

Grid dla scenariusza C liczy odstęp sam: `spacing = ślad_kadru(alt) × (1 - overlap)`.
Przy `search_alt=50`, `focal_px=1810` i `overlap=0.3` to 20 m, czyli ~8 galsów
na 150 m i ~2 km trasy. Obszar z pliku `.poly` wyeksportowanego z Mission Plannera.

RTL zawsze — także po każdym błędzie i po Ctrl+C (wzorzec `_install_signals`
z `suas_simple_mission` przenoszony 1:1).

**Testujesz małymi krokami:**
1. Sam PLAN na spreparowanym `targets.json`, dron na ziemi, `finish_action:=none`
2. Samo przejęcie: GUIDED → zejście na 50 m → obrót nosem → dolot, bez zrzutu
3. Scenariusz A z potwierdzeniem
4. Scenariusz A bez potwierdzenia (zrzut na waypoint)
5. Scenariusz B: okno 5 s mija → zamiatanie gimbalem → dopiero potem spirala
6. Scenariusz C (grid + potwierdzenie), oraz grid bez trafienia → RTL
7. Dwa cele po kolei
8. Pełny przebieg od trasy AUTO w MP

## Etap 7 — real

**7a. Natywny crop z OAK** — jedyna część nietestowalna w symulacji (tam zastępuje
ją `horizontal_fov`). W `drone_camera/oak_publisher.py`: ISP wg ustalonego szczebla
+ `setVideoSize(1024, 1024)`. `depthai_ros_driver` nie wystarczy — nie ma tam
pewnego sposobu na natywny crop (`docs/oak_kadr_detekcji.md`). Mierzysz
`ros2 topic hz` i robisz test z okręgiem (crop czy rozciągnięcie). Stamtąd też
twarde ostrzeżenie: **wymuś manualną ekspozycję**, inaczej rozmycie ruchem zje
cały zysk rozdzielczości.

**7b.** Build → detektor sam → geolokator pieszo, spacer po trawie (weryfikacja
rzutowania bez latania) → GUI z ręki → **pomiar, czy `MODEL4.engine` w ogóle łapie
ludzi** → lot ortofoto bez zrzutu → pełna misja.

Przed pierwszym lotem potrzebuję: **numery kanałów serw zrzutu i PWM
otwarte/zamknięte**, oraz potwierdzenie, że na realu też jest `SERVO7_FUNCTION=0`
i `MNT1_TYPE=0`.

---

## Słabe punkty i ryzyka

**1. Symulacja nie waliduje detekcji.** Gazebowa postać to prymitywna siatka,
a YOLO trenowano na prawdziwych zdjęciach. Zielony etap 6 dowodzi, że **logika
i geometria** są poprawne — nie że cel zostanie wykryty w polu. Detekcję
walidujemy wyłącznie na realu (etap 7). Model jest w toku dopracowywania,
więc liczby pikseli z tabeli kamery są warunkiem koniecznym, nie wystarczającym.

**2. Wysokość jest nad punktem uzbrojenia, nie nad terenem.** `Telemetry.alt` to
`global_relative_frame`. Na pochyłym polu skala rzutowania (GSD = alt/focal) jest
przekłamana o różnicę terenu, a „50 m zrzutu" to 50 m nad domem, nie nad celem.
Przy polu 150×250 m zwykle 1–2 m błędu; wart odnotowania, nie naprawiania.

**3. Znacznik czasu klatki to moment publikacji, nie ekspozycji.**
`oak_publisher.py` stempluje `now()` przy wysyłce, po transferze USB. Przy 8 m/s
100 ms opóźnienia to 0,8 m. To jest podłoga dokładności całego rzutowania.

**4. Grid liczy waypointy sam.** Błąd w wielokącie obszaru = dron leci w pole.
Sprawdzamy programowo, że każdy wyliczony punkt leży w zadanym wielokącie,
a fence w ArduPilocie zostaje twardym bezpiecznikiem niezależnym od ROS.
Baterię pilnujesz Ty w Mission Plannerze i w razie czego dajesz RTL.

**5. Zależność od jednego pliku.** `targets.json` jest jedynym kanałem między
geolokatorem a misją. Odczyt przy każdym wejściu w GUIDED to załatwia, ale plik
musi być zapisywany atomowo — `_atomic_write` już to robi, nie zepsuć tego.

## Szczegóły operacyjne

- **Regresja po każdym etapie:** `suas_simple_mission` w symulacji nad namiotem ma
  nadal działać. To jedyny test, który złapie, że coś zepsuliśmy po drodze.
- **Spirala na velocity control, grid na `goto_global`** — spirala jest lokalna
  i ciągła, grid to długie odcinki.
- **`RTL_ALT` do sprawdzenia w MP** — RTL najpierw wznosi się na tę wysokość.
- **`docs/SUAS.md` aktualizowany w każdym etapie**, nie na końcu. To ściągawka
  w polu; nieaktualna jest gorsza niż żadna.
- **Commit na etap**, żeby regresję w polu dało się wybisektować.
- Zrzut zmienia masę i wytrąci regulator wysokości na chwilę — alt-hold to
  wyrówna, ale nie dziw się skokowi w logu.

## Weryfikacja końcowa (symulacja)

```bash
# świat + SITL              (skrypty z scripts/start_sim/)
ros2 launch drone_bringup gazeboo_ap_sim.launch.py world:=suas_field.sdf

# T1  wszystko poza misja   ->  GUI na http://<ip>:5000
ros2 launch drone_bringup suas_field.launch.py

# T2  misja
ros2 run drone_autonomy suas_full_mission \
    --ros-args --params-file src/drone_bringup/config/suas_mission.yaml

# Mission Planner: survey grid 80 m / 150x250 m / sidelap 60%, AUTO,
# po trasie przełącz na GUIDED
```

Zaliczone, gdy: `targets.json` ma oba cele (namiot < 3 m, człowiek < 5 m od
prawdy ze świata), po GUIDED dron najpierw schodzi na 50 m, leci nosem do celu,
oba zrzuty padają nad właściwymi obiektami, RTL. Powtórka **bez dotykania GUI
i bez spacji** — pełna ścieżka autonomiczna.

---

## Pliki

**Nowe:** `msg/TentDetections.msg`, `msg/OperatorMark.msg`,
`drone_autonomy/suas_marker_web.py` + szablon HTML,
`drone_autonomy/suas_full_mission.py`, `launch/suas_field.launch.py`,
`config/suas_mission.yaml`,
`gazebo/worlds/suas_field.sdf`

**Zmieniane:** `msg/TentDetection.msg`, `action/GotoGlobal.action`,
`drone_interfaces/CMakeLists.txt`, `drone_detector/yolo_detector_base.py`,
`launch/suas_detect_{gazebo,jetson}.launch.py`,
`drone_autonomy/tent_geolocator.py` → `suas_geolocator.py`,
`drone_autonomy/suas_flight_controller.py`, `drone_autonomy/suas_gimbal_controller.py`,
`drone_autonomy/drone_comunication/drone_controller.py`,
`gazebo/models/gimbal_small_3d/model.sdf`, `drone_camera/oak_publisher.py`,
`setup.py` (drone_autonomy, drone_detector), `scripts/run_ap_gazeboo_sitl.sh`,
`docs/{SUAS,tent_geolocator,oak_kadr_detekcji}.md`

**Nietykane:** `suas_simple_mission.py`, `drone_hardware/drone_handler.py`.
