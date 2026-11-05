# Scenariusze misji SUAS — przebieg, czasy, slabe punkty

Rozpisane na podstawie kodu `suas_full_mission.py` i `suas_flight_controller.py`
oraz parametrow z `config/suas_mission.yaml`. Kazdy czas jest wyliczony, nie
zgadniety — zrodlo podane przy liczbie.

Cel dokumentu: **zaden przypadek nie ma byc niespodzianka w polu.**

---

## 0. Dwie zasady, z ktorych wynika cala reszta

### Zasada 1: spacja znaczy zawsze "tak, zrzucamy tutaj"

Gdy cel jest widoczny, dron najpierw sie wycentruje. Gdy nie jest — zrzuci na
wspolrzedne. **Milczenie operatora, gdy on patrzy, znaczy "to nie ten cel"**
i wysyla drona do kolejnego kandydata.

### Zasada 2: pytamy TYLKO wtedy, gdy operator naprawde patrzy

`suas_marker_web` publikuje `/operator_online`, dopoki przegladarka przysyla puls
(`GET /ping` co 2 s). Misja wymaga **swiezej** wiadomosci — starsza niz 5 s
znaczy "operatora nie ma".

Dlaczego puls od przegladarki, a nie liczenie widzow po stronie serwera: gdy
padnie Tailscale, przegladarka **nie zamyka polaczenia**, a TCP bez keepalive
zauwaza martwy drugi koniec dopiero po minutach. Serwer liczylby widza, ktorego
nie ma, i dron pytalby w prozni. Puls znika w 2 s.

Gdy operatora nie ma, misja **nie pyta w ogole** i zawsze konczy cel zrzutem —
zeby bez nadzoru nie krecic sie w kolko.

### Skad biora sie czasy

| wielkosc | wartosc | zrodlo |
|---|---|---|
| predkosc dolotu | **do ustalenia** | `WPNAV_SPEED` w ArduPilocie, NIE `max_vel` z naszego yamla |
| predkosc opadania | ~1,5 m/s | `WPNAV_SPEED_DN`, domyslnie 150 cm/s |
| `acquire_timeout` | 5 s | yaml |
| `confirm_timeout` | 15 s | yaml |
| `hover_hold_time` | 5 s | yaml |
| `approach_timeout` | 90 s | `suas_full_mission.py` |
| `spiral_timeout` | 45 s | `suas_full_mission.py` |
| `search_timeout` | 120 s | yaml (kod ma 300) |
| `operator_ping_ttl` | 5 s | `suas_marker_web.py` |
| `det_confirm_gap` | **1,5 s** | wszedzie; 0,5 zerowalo okno przy jitterze detektora |
| `center_lost_timeout` | 15 s | `suas_full_mission.py` |
| `target_alt` (zrzut) | 50 m | yaml |
| centrowanie zmierzone | 5,6 m zamkniete w 10 s | pomiar w symulacji |

**`max_vel: 3.0` z yamla NIE dotyczy dolotu.** Ogranicza wektory predkosci
naszego kontrolera w APPROACH/HOVER. Dolot idzie przez `send_goto_global`, czyli
akcje ArduPilota, i tam rzadzi `WPNAV_SPEED`.

**Do zrobienia przed lotem:** odczytaj `WPNAV_SPEED` w MP i wpisz tutaj. Czasy
nizej licze przy **5 m/s**. Odleglosci przyjete do rachunku: koniec ortofoto ->
cel 1 **100 m**, cel 1 -> cel 2 **60 m**, powrot **150 m**.

---

## 1. Pula kandydatow zamiast jednego celu na klase

To jest najwazniejsza zmiana w stosunku do pierwszej wersji misji.

Geolokator liczy **ranking kandydatow** dla kazdej klasy, nie tylko zwyciezce.
Misja czyta cala liste i buduje z obu klas jedna **pule**. Zawsze leci do
**NAJLEPSZEGO kandydata** (najwyzszy `score` = obs x conf) sposrod klas, ktore
nie dostaly jeszcze zrzutu. Odleglosc NIE decyduje o kolejnosci.

```
pula = kandydaci NAMIOTU + kandydaci CZLOWIEKA  (przefiltrowani)
dopoki sa ladunki i pula niepusta:
    wez NAJLEPSZEGO (score) z klasy bez zrzutu
    lec, zapytaj (jesli operator patrzy)
      spacja      -> centruj, zrzuc, klasa zalatwiona
      brak spacji -> wyrzuc kandydata z puli, wez nastepnego wg score
klasy bez zrzutu i bez kandydatow -> GRID
```

**Filtr jest konieczny.** Geolokator wypisuje takze kandydatow z 3-4
obserwacjami, czyli krzaki i cienie. Do puli wchodza tylko ci, ktorzy przeszli
prog `min_obs` swojej klasy (10 namiot / 5 czlowiek) albo zostali **wskazani
recznie** przez operatora. Bez tego dron leciałby 60 s do krzaka.

**Dlaczego od najlepszego, a nie od najblizszego:** kandydat z 300 zbieznych
obserwacji jest pewniejszy niz plandeka z 15, nawet jesli plandeka lezy blizej.
Kolejnosc po odleglosci mialaby sens tylko przy odrzucaniu przez operatora —
a bez operatora zrzut poszedlby na najblizszego, czyli potencjalnie na plandeke,
i klasa bylaby "zalatwiona" na zle. Score chroni oba przypadki.

Ladunek jest przypisany do klasy (namiot -> 0, czlowiek -> 1), wiec po zrzucie
na klase jej pozostali kandydaci wypadaja z gry.

---

## Scenariusz 1 — wszystko idzie dobrze

Dwa waypointy z ortofoto, lacze caly czas, dwa potwierdzenia spacja, dwa zrzuty
nad wycentrowanymi celami.

| krok | czas | skad |
|---|---|---|
| przelaczenie AUTO -> GUIDED | 0 s | Twoja akcja |
| zejscie 80 -> 50 m | **20 s** | 30 m / 1,5 m/s |
| odczyt `targets.json`, budowa puli | 0 s | plik |
| dolot nad najlepszego kandydata | **20 s** | 100 m / 5 m/s |
| okno akwizycji | **1-5 s** | namiot 49 px, potwierdza sie od razu |
| czekanie na spacje | **~2 s** | wciskasz od razu |
| centrowanie + trzymanie | **15 s** | 10 s dojazdu + 5 s `hover_hold_time` |
| zrzut 1 | **2 s** | `drop_hold_s` + serwis |
| dolot nad drugi cel | **12 s** | 60 m / 5 m/s |
| akwizycja + spacja + centrowanie + zrzut | **24 s** | jak wyzej |
| **RAZEM do konca zrzutow** | **~1 min 40 s** | |
| RTL | **60-90 s** | zalezy od `RTL_ALT` |

To przypadek wzorcowy i jednoczesnie **najszybszy mozliwy**.

---

## Scenariusz 2 — brak jakiegokolwiek polaczenia, pelny automat

Zerwane SSH/Tailscale. Wszystko chodzi pod tmux, wiec procesy zyja (bez tmux —
patrz `docs/misja_real.md`, dron zawisa i trzeba RTL z MP).

Co to zmienia:

- **nie ma klikniec operatora**, wiec czlowiek nie ma waypointu (automat nie
  zapisuje go z pulapu ortofoto — bramka `person_max_alt`),
- **misja nie pyta w ogole**, bo puls z GUI nie przychodzi. Najlepszy kandydat
  klasy jest od razu zatwierdzony: dron centruje i zrzuca,
- namiot ma waypoint z automatu (zmierzony blad 0,45 m).

| krok | czas |
|---|---|
| zejscie 80 -> 50 m | 20 s |
| dolot nad namiot | 20 s |
| okno akwizycji | 5 s |
| **bez pytania** — centrowanie | 15 s |
| zrzut | 2 s |
| **czlowiek: brak kandydatow -> GRID** | **120 s (timeout)** |
| ladunek 2 zostaje na pokladzie | — |
| RTL | 60-90 s |
| **RAZEM** | **~3 min 30 s** |

Zysk wzgledem poprzedniej wersji: znika **caly `confirm_timeout` na kazdym celu**
(2 x 15 s czekania w prozni), a namiot dostaje zrzut **nad wycentrowanym celem**
zamiast na wspolrzedne.

**Twarda prawda o tym scenariuszu:** grid dla czlowieka ma przy obecnej kamerze
szanse bliskie zeru, jesli czlowiek STOI — na 50 m ma wtedy 8 px, ponizej progu
YOLO. Jesli LEZY, ma 30 x 22 px i grid ma sens (patrz komentarz w
`gazebo/worlds/suas_field.sdf`). Bez lacza realnie wracamy z jednym ladunkiem,
chyba ze cel lezy.

---

## Scenariusz 3 — ATRAPA, czyli automat zlapal false positive

Najciekawszy przypadek i ten, pod ktory powstala pula kandydatow.

**Dlaczego klastrowanie tu nie chroni:** setki zbieznych obserwacji odsiewaja
szum losowy, ale obiekt **systematycznie** brany za namiot zbuduje rownie pewny,
ciasny klaster. "Waypoint z automatu + 300 obserwacji" nie dowodzi wiec, ze to
wlasciwy obiekt — dowodzi tylko, ze model konsekwentnie cos tam widzi.

**Dlaczego Ty widzisz lepiej:** waypoint powstal z **80 m**, gdzie namiot ma
30 px. Pytanie dostajesz na **50 m**, gdzie ma 49 px, na ostrym obrazie w GUI.
Twoje dane sa lepsze niz te, z ktorych powstal waypoint.

| krok | czas |
|---|---|
| zejscie + dolot nad atrape | 40 s |
| akwizycja (potwierdza atrape) | 1-5 s |
| **nie wciskasz spacji** | 15 s |
| dolot do kandydata #2 | zalezy od odleglosci |
| akwizycja + spacja + centrowanie + zrzut | 24 s |

Gdy kandydaci sie skoncza, wchodzi grid.

**Przypadek gorszy: wciskasz spacje przez pomylke.** Dron centruje sie na atrapie
i tam zrzuca. Nie ma na to zabezpieczenia w kodzie i **nie da sie go dodac** — to
jest z definicji Twoja decyzja. Zasada w polu: **spacja tylko wtedy, gdy widzisz
na podgladzie, ze ramka siedzi na wlasciwym obiekcie.** W watpliwosci nie rob
nic — dron poleci sprawdzic nastepnego kandydata.

---

## Scenariusz 4 — zgubienie detekcji

### 4a. Zgubienie PRZED potwierdzeniem

`wait_acquire` nie zbiera M trafien w `acquire_timeout` (5 s). Wtedy zamiatanie
gimbalem (domyslnie **wylaczone**) i spirala (45 s). Jesli nadal nic:

- **operator patrzy** -> pyta: `[SPACJA] = zrzuc na wspolrzedne mimo to`,
  `[nic] = szukam dalej`. Ty decydujesz, czy waypoint jest wart slepego zrzutu,
- **operatora nie ma** -> zrzut na wspolrzedne.

Narzut: **+50 s** (5 s akwizycji + 45 s spirali).

### 4b. Zgubienie W TRAKCIE centrowania — NAPRAWIONE

Bylo: przy zgubieniu detekcji kontroler odstawial gimbal z pionu na -55 st.
Nad celem to wyrzucalo punkt pod dronem poza kadr (35 st. odchylenia przy
polowie FOV 32,2 st.) i cel stawal sie nieodzyskiwalny — dron palil caly
`approach_timeout` (90 s) i zrzucal na wspolrzedne.

Teraz: `center_over_target` ustawia `search_gimbal_on_lost=false`, wiec przy
zgubieniu gimbal **zostaje w pionie**. Chwilowa utrata (przeslonieciecie, zla
klatka) konczy sie przy pierwszej dobrej detekcji — 0,07-0,14 s przy 7-14 FPS.
Faza dolotu bez zmian (tam -55 st. pomaga cel znalezc).

| przypadek | narzut |
|---|---|
| cel wrocil | **~0 s** |
| cel zniknal na dobre | **15 s** (`center_lost_timeout`) + zrzut na wspolrzedne |

Ciagly SEARCH dluzszy niz `center_lost_timeout` konczy centrowanie od razu —
`approach_timeout` (90 s) zostaje tylko jako gorne ograniczenie calosci.

---

## 5. Kiedy misja przechodzi do gridu — jednoznacznie

Grid uruchamia sie **dopiero wtedy, gdy dla danej klasy skoncza sie wszyscy
kandydaci** — albo dlatego, ze `targets.json` zadnego nie mial, albo dlatego, ze
operator odrzucil po kolei wszystkich.

```
kandydaci danej klasy w puli?
├── TAK  -> lec do NAJLEPSZEGO (score) -> akwizycja -> decyzja
│           ├── spacja / brak operatora -> centruj, zrzuc, KONIEC KLASY
│           └── brak spacji (operator patrzy) -> wyrzuc z puli, nastepny
└── NIE  -> GRID nad obszarem
            ├── znalazl -> decyzja jak wyzej
            │               └── brak spacji -> GRID JESZCZE RAZ (bez konca)
            └── nie znalazl
                ├── operator patrzy  -> GRID JESZCZE RAZ (bez konca)
                └── operatora nie ma -> koniec, ladunek zostaje
```

**Grid kreci sie bez konca, dopoki operator patrzy.** Konczy go spacja albo
Ctrl+C (abort i RTL). To jest bezpieczne wlasnie dlatego, ze warunkiem petli jest
obecnosc operatora — zawsze jest ktos, kto moze ja przerwac. Gdy operatora nie
ma, grid idzie **raz**, bo bez nadzoru nikt by drona nie zatrzymal.

---

## 6. Pole przeszukiwania — jak je zaplanowac

Grid liczy odstep galsow sam, ze sladu kadru i overlapu:

```
slad    = img_w * wysokosc / focal_px  =  1024 * 50 / 813  =  63 m
odstep  = slad * (1 - overlap)         =  63 * 0,7         =  44 m
galsy   = search_w / odstep + 1        =  150 / 44 + 1     =  4
trasa   = galsy * search_h             =  4 * 250          =  1000 m
czas    = trasa / predkosc             =  1000 / 5         =  200 s
```

`search_timeout` w yamlu jest juz **300 s** (bylo 120, co ucinalo grid po
niecalych trzech galsach). Dalsze opcje skrocenia:

| opcja | efekt |
|---|---|
| `search_timeout: 300` (wartosc z kodu) | grid sie domyka, kosztuje 3,5 min |
| `search_overlap: 0.1` -> odstep 57 m, 3 galsy, 750 m, 150 s | szybciej, wieksze ryzyko dziury |
| podniesc `WPNAV_SPEED` do 8 m/s | 1000 m w 125 s |
| zmniejszyc `search_w`/`search_h` do realnego obszaru | najlepsze, jesli wiadomo gdzie szukac |

**Rekomendacja:** ustalcie realny obszar po obejrzeniu pola. 150x250 to liczby
z planu zawodow, nie z pomiaru.

Uwaga: grid jest wysrodkowany na **pozycji zapamietanej przy przejeciu lotu**
(`self.home`), a nie na srodku pola. Jesli przejmiesz lot na skraju obszaru,
polowa gridu poleci poza teren.

---

## 7. Slabe punkty — gdzie to moze sie wysypac

### 7.1 Spirala prawie nic nie wnosi, a kosztuje 45 s

Na 50 m kadr obejmuje 63 m, czyli **promien 31 m wokol drona**. Spirala ma
`spiral_step` 20 m, wiec pierwszy pierscien dokłada zasieg do ~51 m — margines,
ktory w duzej czesci byl juz widoczny z zawisu. Przy 5 m/s jeden pierscien to
113 m i 23 s, wiec w 45 s mieszcza sie niecale dwa.

Najwazniejsze: **spirala nie adresuje prawdziwej przyczyny.** Jesli cel nie
zostal potwierdzony, to prawie na pewno dlatego, ze ma za malo pikseli, a nie
dlatego, ze wypadl z kadru. Przesuniecie drona o 20 m nie dodaje ani jednego
piksela.

**Rekomendacja: wylaczyc spirale** (`spiral_timeout: 0`) i isc od razu do
pytania. Oszczedza 45 s na kazdym celu, ktory sie nie potwierdzil.

### 7.2 `wait_acquire` wyrzuca to, co widzial w locie

`wait_acquire` zaczyna od `_reset_det_window()`, wiec **wszystko, co detektor
zobaczyl podczas dolotu, jest kasowane**. Callbacki chodza (akcja `goto` spinuje
node), wiec dane sa — tylko sa wyrzucane.

Skutek najgorszy w gridzie: dron przelatuje nad celem w polowie odcinka, detektor
go widzi, a misja sprawdza okno dopiero po dolocie do naroznika. **Grid moze
przeleciec dokladnie nad czlowiekiem i tego nie zauwazyc.**

**Rekomendacja:** sprawdzac okno detekcji w trakcie lotu, nie po dolocie. Wymaga
przerywania akcji `goto` w polowie — najwieksza robota z tej listy.

### 7.3 Dwie dlugie akcje blokujace

`goto` i `descend` ida przez akcje ArduPilota z timeoutem **180 s**. Jesli serwer
akcji nie domknie celu (`yaw_callback` w `drone_handler` ma udokumentowany blad
tego typu — dlatego `yaw_to_target` jest wylaczony), misja stoi 3 minuty.
**Rekomendacja:** 60 s dla `goto`.

### 7.4 `approach_timeout` 90 s — ZALATWIONE INACZEJ

Zamiast obnizac gorny limit calego centrowania, dodany `center_lost_timeout`
(15 s ciaglego SEARCH = cel zniknal na dobre). `approach_timeout` zostaje 90 s
jako bezpiecznik na przypadek, gdy cel jest widoczny, ale dron nie moze sie
domknac w tolerancji.

### 7.5 Gimbal odjezdza od celu przy zgubieniu — NAPRAWIONE, patrz 4b

### 7.9 Geolokator i misja bily sie o gimbal — NAPRAWIONE

Geolokator trzymal gimbal w pionie co 2 s (`lock_nadir`), a misja w APPROACH
koryguje ten sam kanal — dwoch piszacych na jeden silownik. Teraz misja przy
przejeciu publikuje `/geolocator/lock_nadir=false` i od tej chwili gimbal
nalezy do niej. Geolokator dodatkowo **sledzi zadany kat gimbala** i rzutuje
nim, zamiast zakladac sztywno -90 — wiec pochylenie w APPROACH nie psuje juz
jego obserwacji.

### 7.10 `descend()` wracal natychmiast — NAPRAWIONE

Akcja `goto_global` konczy sie po odleglosci POZIOMEJ < 2 m, a `descend` podaje
te sama lat/lon — wracal wiec w 0 s, a zejscie dzialo sie dopiero w dolocie.
Przy krotkim dolocie okno akwizycji otwieralo sie na ~70 m zamiast 50.
Teraz `descend` czeka, az `|alt - cel| < 2 m` (timeout 60 s).

### 7.6 Grid liczony od punktu przejecia lotu — patrz 6

### 7.7 Sygnalem obecnosci jest strona na porcie 5000

Jesli ogladasz obraz tylko przez `web_video_server` (port 8080), misja uzna, ze
Cie nie ma, i przestanie pytac. **Karta GUI musi byc otwarta.**

### 7.11 `det_confirm_gap` zerowal okno potwierdzania — NAPRAWIONE

Zmierzone 2026-09-02: dron doleciał nad lezacego czlowieka z bledem **0,1 m**
i mimo to `okno akwizycji 5s minelo bez potwierdzenia`. W logu dwie linie
`Przerwa w detekcjach 0.51s / 0.71s (> 0.5s) — okno potwierdzania wyzerowane`.

Detektor publikuje KAZDA klatke, takze pusta, wiec `det_confirm_gap` mierzy
jitter detektora, nie utrate celu. W symulacji detektor chodzi ~5 Hz
z przerwami do 0,7 s, wiec prog 0,5 s zerowal okno w kolko i 4 z 8 klatek
nigdy sie nie uzbieralo. Namiot przechodzil, bo jest wykrywany w kazdej
klatce; czlowiek 31x9 px nie mial zapasu.

Koszt tego bledu w zmierzonym przebiegu: 5 s akwizycji + 8 s spirali
+ centrowanie z 19,5 m zamiast z 1 m = **25 s straty na celu**. Cala misja
52 s zamiast oczekiwanych 27 s.

Naprawione przez ujednolicenie na **1,5 s** w czterech miejscach (domyslna
w kodzie, `suas_mission.yaml`, oba launche) — wczesniej yaml mial jedna
wartosc, a launche druga.

### 7.12 `require_same_track` nie dzialal przy track_id = -1 — NAPRAWIONE

Warunek brzmial `if good and self.require_same_track and msg.track_id >= 0:`,
wiec przy braku ID **cala kontrola sciezki byla pomijana**, a trafienie i tak
wpadalo do okna M z N. Zabezpieczenie nie dzialalo dokladnie w przypadku,
przed ktorym mialo chronic.

Zmierzone 2026-09-02: grid na punkcie oddalonym 14,6 m od `tree_4_a` potwierdzil
cel jako `4/6 klatek, ID=-1`. Centrowanie gonilo ducha — pozycja skakala o 20 m
miedzy probkami, `vx` wysycal sie na 3 m/s. Prawdziwe cele dostaja ID od razu
(namiot 53, lezacy czlowiek 151), wiec `track_id = -1` to podpis migoczacej
falszywki.

Naprawione: brak sciezki = trafienie NIE liczy sie do okna (`good = False`),
wiec detekcja bez ID nie odswieza tez `last_det_time` i nie wplywa na sterowanie.

### 7.13 Grid zrzucal ladunek po nieudanym centrowaniu — NAPRAWIONE

W trybie gridu nie ma waypointu, na ktory mozna zrzucic w ciemno. Mimo to po
nieudanym centrowaniu leciala galaz `zrzut w miejscu wykrycia` — czyli ladunek
szedl tam, gdzie dron akurat stal, obok falszywki.

Teraz: cel, ktory znika w trakcie centrowania, jest traktowany jak falszywka.
Operator patrzy -> szukamy dalej. Operatora nie ma -> koniec klasy, ladunek
zostaje na pokladzie (zgodnie z regula "nie ma adresu -> nie zrzucamy").

### 7.8 Pojedynczy plik jako kanal miedzy geolokatorem a misja

`targets.json` to jedyne polaczenie. Zapis jest atomowy, wiec nie ma ryzyka
odczytu polowki pliku — ale jesli geolokator padnie, misja zobaczy stary plik
i nie zauwazy problemu.

---

## 8. Plan testow — po kawalku

| # | co testujemy | jak wywolac | kryterium |
|---|---|---|---|
| 1 | budowa puli i kolejnosc | spreparowany `targets.json` z kilkoma kandydatami, dron na ziemi, `finish_action:=none` | log pokazuje kolejnosc wg odleglosci i filtr `min_obs` |
| 2 | obecnosc operatora | otworz i zamknij karte GUI | `/operator_online` przechodzi na false w ~5 s |
| 3 | zerwanie lacza | `iptables -I INPUT -p tcp --dport 5000 -j DROP` | to samo, mimo ze przegladarka nie zamknela polaczenia |
| 4 | przejecie + zejscie + dolot | GUIDED, `drop_servo_ch:0` | schodzi na 50 m, leci nad cel, nie zrzuca |
| 5 | scenariusz 1 | potwierdzasz spacja oba cele | dwa `ZRZUT [SYMULACJA]` nad celami |
| 6 | scenariusz 2 | zamknij GUI przed przejeciem | zero pytan, oba cele obsluzone automatycznie |
| 7 | **scenariusz 3 (atrapa)** | dwa kandydaty namiotu, odrzuc pierwszy | leci do drugiego, nie do gridu |
| 8 | scenariusz 4b | zaslon cel po potwierdzeniu | **teraz: 90 s zawisu. Po poprawce 7.5: odzyskanie** |
| 9 | grid bez konca | odrzuc wszystkich kandydatow | grid startuje i powtarza sie, spacja konczy |
| 10 | calosc | trasa AUTO w MP -> GUIDED | pelny przebieg |

Testy 1, 2 i 7 robi sie **bez latania** — to jest najlepszy stosunek wartosci do
ryzyka i od nich warto zaczac.

---

## 9. Co zmienic przed lotem — lista decyzji

| zmiana | dlaczego | ryzyko |
|---|---|---|
| ~~`search_gimbal_on_lost` w centrowaniu~~ | ZROBIONE | |
| ~~`SIGHUP` w handlerze misji~~ | ZROBIONE | |
| ~~geolokator zwalnia gimbal, rzutuje zadanym katem~~ | ZROBIONE | |
| ~~`descend` czeka na wysokosc~~ | ZROBIONE | |
| ~~pula od najlepszego (score), nie od najblizszego~~ | ZROBIONE | |
| `approach_timeout` 90 -> 40 | oszczedza 50 s na celu | mniej czasu na trudne centrowanie |
| ~~`search_timeout` 120 -> 300~~ | ZROBIONE | |
| spirala wylaczona | oszczedza 45 s na celu | tracimy szukanie przy blednym waypoincie |
| `WPNAV_SPEED` ustalone i zapisane | czasy staja sie przewidywalne | — |
| realne `search_w`/`search_h` | grid nie lata poza teren | trzeba obejrzec pole |
| kanaly i PWM serw zrzutu | bez tego zrzut jest tylko logiem | patrz `docs/misja_real.md` |
