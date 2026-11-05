# Geolokator — czy daje dobry adres

Tylko real. Najwazniejsza jest dokladnosc **znacznika operatora**, bo to on
jest adresem czlowieka (automat go nie zapisze — na 50 m ma 8 px).

Dlaczego akurat klik: automat rzutuje klatke, ktora wlasnie przyszla, a klik
dotyczy klatki **zamrozonej kilka sekund temu**. Przez ten czas dron leci
dalej. Wszystko zalezy od tego, czy klik zostanie policzony telemetria
z chwili POWSTANIA klatki. Gdy nie jest — nie ma zadnego bledu, jest tylko
zly punkt.

---

## 1. Na ziemi, przed startem (2 minuty)

Dron uzbrojony na ziemi, gimbal w dol, GUI otwarte na :5000.

### Klik w SRODEK kadru

Zamroz klatke (LPM), kliknij dokladnie w srodek. W logu geolokatora:

```
ZNACZNIK OPERATORA [namiot] piksel (512,512) z 12.0 m ->
    +0.0 m wschod, +0.0 m polnoc od drona
```

| wynik | co znaczy |
|---|---|
| piksel ~(512,512), offset ~**0,0** | dobrze, GUI i rzutowanie sie zgadzaja |
| piksel inny niz 512,512 | przegladarka zle przelicza klik na oryginalna klatke |
| piksel dobry, offset niezerowy | gimbal nie jest w nadirze |

Potem kliknij **gorna krawedz**: offset ma wyjsc `alt * tan(32.2 st.)`
do przodu. To sprawdza ogniskowa — jedyna liczba, ktorej nie widac inaczej.

### KTORY ZEGAR — sprawdzenie numer jeden

Geolokator loguje to raz, przy starcie detekcji:

```
czas detekcji brany z: stamp klatki                                <- DOBRZE
czas detekcji brany z: det_latency=0.20s (stamp w innym zegarze)   <- ZLE
```

Drugi wariant znaczy, ze klik jest liczony telemetria z chwili KLIKNIECIA,
nie z chwili klatki. Caly czas patrzenia na zamrozony obraz zamienia sie
wtedy w blad: **8 m/s x 3 s = 24 m**, bez ostrzezenia.

Jesli widzisz `det_latency`, a klatki maja sensowny stamp, wyrownaj progi —
GUI dopuszcza 30 s rozjazdu zegara, geolokator tylko 5:

```bash
ros2 launch drone_bringup suas_geolocator.launch.py -p stamp_max_skew:=30.0
```

(W symulacji `det_latency` jest normalne — most Gazebo podaje czas od zera.
Dlatego sciezki operatora nie da sie zwalidowac w Gazebo.)

### Ile masz czasu od zamrozenia do klika

Geolokator trzyma **6,4 s** historii telemetrii (64 probki, 10 Hz). Klik
w starsza klatke nie ma czym byc policzony:

```
znacznik operatora bez telemetrii — pomijam
```

Ten komunikat idzie do logu ROS, **nie do przegladarki** — operator jest
pewien, ze oznaczyl cel.

Sprawdz raz: zamroz, odczekaj 10 s, kliknij (ma polecic warn), potem powtorz
z 3 s (ma przejsc). **Regula w polu: od zamrozenia do wyboru klasy ok. 5 s.**

---

## 2. W locie — dokladnosc klika

### Test na ruch (ten rozstrzyga)

Lec **prosto, ze stala predkoscia** obok znanego obiektu i oznacz go dwa razy:

1. klik **natychmiast** po zamrozeniu,
2. klik po **swiadomym odczekaniu 3-4 s**.

Oba punkty maja wyjsc w tym samym miejscu z dokladnoscia do paru metrow.
Jesli ten drugi jest przesuniety **w kierunku lotu** o mniej wiecej
`predkosc * czas patrzenia` — stamp nie jest uzywany, wroc do sekcji 1.

### Pomiar bledu

Wpisz wspolrzedne z logu jako waypoint w Mission Plannerze i zmierz
odleglosc do prawdziwego obiektu.

```
ZNACZNIK OPERATORA ... = 52.123456 21.123456          <- surowy punkt z klika
   wpadl do klastra #3 (171 obs) -> 52.123460 21.123458   <- adres dla misji
```

| co porownujesz | budzet |
|---|---|
| surowy punkt vs prawdziwy obiekt | **< 5 m** |
| surowy punkt vs srodek klastra | **< 10 m** (`cluster_radius`) |

Rozjazd wiekszy niz 10 m znaczy, ze klik zalozyl NOWY klaster obok
prawdziwego celu — i to on pojdzie do misji, bo znacznik operatora ma
pierwszenstwo nad automatem.

---

## 3. Automat (namiot) — jedno sprawdzenie

**Zawis nic nie dowodzi.** Cel jest wtedy pod dronem, przesuniecie bliskie
zeru i bledna skala nie ma na czym zadzialac — wynik wychodzi idealny takze
przy zlej geometrii.

Test to **przelot OBOK celu**. Trzy warunki naraz:

```
 #1  -35.363258  149.165570   obs= 171  conf=0.94  przeloty=2  rozrzut=1.9m
```

1. jeden obiekt = **jeden** kandydat (dwa = blad skali albo `det_latency`),
2. `rozrzut` w okolicach metra,
3. kandydat **nie wedruje** razem z dronem miedzy raportami (co 5 s).

---

## 4. Gdy nie wychodzi

| objaw | przyczyna |
|---|---|
| klik przesuniety w kierunku lotu, rosnie z czasem patrzenia | stamp ignorowany (sekcja 1) |
| klik przesuniety w kierunku lotu, ale staly | `det_latency` za male |
| `znacznik operatora bez telemetrii` | klik po ponad 6,4 s od zamrozenia |
| `odrzucone: rozmiar=...` masowo w locie | zla `focal_px` albo zla `alt` |
| `odrzucone: promien=...` | gimbal patrzy w horyzont, nie w dol |
| blad rosnie z predkoscia | przechyl ramy nie kompensowany — sprawdz `MNT1_TYPE` i `SERVO7_FUNCTION` w MP; jesli mount JEST stabilizowany, ustaw `gimbal_stabilized: true` |
| blad rosnie z wysokoscia | zla `focal_px` |

`observations.csv` z lotu trzyma dane surowe (piksel, telemetria, wynik
rzutowania), wiec kazde z powyzszych da sie przeliczyc offline z innym
`det_latency` — bez powtarzania lotu.

---

## Minimum przed lotem

- [ ] klik w srodek kadru daje offset 0,0
- [ ] log mowi `czas detekcji brany z: stamp klatki`
- [ ] klik po 3 s i klik natychmiastowy daja ten sam punkt
- [ ] `targets.json` powstaje w `src/drone_bringup/config/`
