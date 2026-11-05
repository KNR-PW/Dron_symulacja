# Misja SUAS — jak ma dzialac

Ten dokument opisuje JEDNA misje, ktora ma zastapic `suas_full_mission`
i `suas_grid_mission`. Czytaj tylko ten plik.

Zastepuje: `misja_scenariusze.md` (rozpisany stary przebieg) i czesc
`misja_real.md` (zostaje tam preflight i sprzet). `SUAS.md` zostaje jako
sciagawka komend, `suas_geolocator.md` jako opis geolokatora.

Stan: **projekt, kod jeszcze nie napisany.** Sekcja 8 to rzeczy do
rozstrzygniecia przed kodowaniem.

---

## 1. Jedna zasada, z ktorej wynika cala reszta

Dla kazdej klasy osobno misja zadaje jedno pytanie: **czy mam adres?**

```
NAMIOT   ma waypoint z geolokatora?  TAK -> DOWIEZ      NIE -> SZUKAJ
CZLOWIEK ma waypoint z geolokatora?  TAK -> DOWIEZ      NIE -> SZUKAJ (od miejsca,
                                                               w ktorym stoi)
```

To jest cala logika wysokiego poziomu. Nie ma puli kandydatow, nie ma
rankingu po score, nie ma odrzucania kandydata #1 na rzecz #2, nie ma gridu
bez konca.

**Czlowiek bez adresu tez jest szukany** — ale dopiero po namiocie i inaczej
zaczyna. Powod jest prosty: w trakcie skanu na namiot NIKT nie obserwowal
klasy CZLOWIEK (obserwator sluchal tylko `/tent_detections`), wiec teren jest
pod tym katem nieogladany, mimo ze dron nad nim przelecial.

Skan dla czlowieka zaczyna sie **obrotem 360 st. W MIEJSCU**, tam gdzie dron
akurat stoi — czyli po zrzucie nad namiotem. Ten obrot jest darmowy, bo dron
juz tam wisi. Dopiero potem leci do **najblizszego** punktu trasy i dalej
kolejnoscia najblizszego sasiada. Powrot na WP1 tylko po to, zeby zaczac
"od poczatku", bylby czystym przelotem bez zysku.

Kolejnosc liczy sie zachlannie, a nie cyklicznym przesunieciem listy, bo
trasa jest LINIA, a nie petla: przy trzech punktach i dronie przy WP3
cykliczne WP3 -> WP1 -> WP2 to 425 m, a WP3 -> WP2 -> WP1 tylko 300 m.

Kazdy punkt zachowuje przy tym **swoj** luk i kurs z `scan_arc_deg` /
`scan_heading_deg` — sa przypisane do punktu, nie do miejsca w kolejce.

**Ile to realnie daje:** na 50 m czlowiek STOJACY ma 8 px, czyli ponizej progu
YOLO (sekcja 5). Ta sciezka ma szanse tylko przy czlowieku LEZACYM (30x22 px).
Wylaczyc: `search_person_after_tent: false`.

**Dlaczego tak.** Adres z geolokatora to albo klaster z setek zbieznych
obserwacji (zmierzony blad 0,45 m), albo klik operatora, ktory patrzyl na
obraz. W obu przypadkach ktos juz podjal decyzje. Zadaniem misji jest
**dowiezc ladunek pod ten adres**, a nie prowadzic sledztwo od nowa.

Kolejnosc klas jest sztywna: **najpierw NAMIOT, potem CZLOWIEK.** Namiot ma
na 50 m 49 px i wykrywa sie sam, czlowiek 8 px i praktycznie nie. Robimy
najpierw to, co na pewno wyjdzie.

---

## 2. Dwie sciezki, wspolny ogon

Nie ma jednego ciagu na wszystko. Sciezki sa dwie i roznia sie GEOMETRIA:
raz cel lezy PRZED dronem, raz POD nim, a to zmienia i sterowanie, i to,
kiedy detektor w ogole ma glos.

### Kiedy detektor moze przerwac lot

**Detektor przerywa lot tylko wtedy, gdy dron nie ma dokad lepiej leciec.**

| sytuacja | okno M z N |
|---|---|
| dolot na waypoint z geolokatora | **WYLACZONE** — wlacza sie dopiero po dolocie |
| przelot miedzy punktami skanu | wlaczone |
| obrot na punkcie skanu | wlaczone |

Adres z geolokatora to klaster z setek zbieznych obserwacji albo klik
operatora, ktory patrzyl na obraz. Pojedyncze okno M z N zlapane w locie jest
przy tym slaba przeslanka — krzak po drodze zatrzymywalby drona i odciagal go
od punktu, ktory ktos juz zweryfikowal. Gorzej: ladunek jest przypisany do
klasy, wiec zrzut na taka detekcje to zrzut ladunku NAMIOTU na krzak, a dobry
adres namiotu zostaje niewykorzystany.

W trybie SZUKAJ jest dokladnie odwrotnie: detektor to JEDYNE zrodlo wiedzy,
wiec musi moc przerwac lot w dowolnym momencie.

### Sciezka A — cel PRZED dronem (skan, wykrycie w locie)

```
  1. STOP          hamowanie tam, gdzie jestem
  2. SPRAWDZ       okno M z N jeszcze raz, na stojaco
  3. CELUJ         obrot nosem na cel + gimbal, BEZ ruchu do przodu
  4. SPACJA        operator potwierdza nieruchomy, wycentrowany obraz
  5. PODLOT        APPROACH — prosto przed siebie, gimbal domyka sie w dol
  6. ZAWIS + ZRZUT
```

**STOP jest konieczny**, bo przerwanie dolotu NIE zatrzymuje drona —
ArduPilot w GUIDED trzyma ostatni zadany punkt i przy 5 m/s ucieka
kilkadziesiat metrow, zanim ktokolwiek zacznie centrowanie.

**SPRAWDZ na stojaco jest filtrem.** Cel potwierdzony w locie, ktory nie
potwierdza sie po zatrzymaniu, jest podejrzany — rozmycie ruchem robi
z krzakow dziwne rzeczy. Nie potwierdzil sie -> klasa dostaje `det_cooldown`
ciszy i wracamy do tego, co robilismy. Bez tej ciszy ta sama falszywka
zatrzymywalaby drona w kolko.

**CELUJ to nowy krok.** Dron stoi i obraca sie nosem na cel (yaw z uchybu
poziomego), gimbal domyka sie w pionie. Wychodzimy, gdy OBA uchyby sa
w tolerancji przez `aim_hold`. Dopiero potem rusza podlot — dzieki temu jest
on linia prosta i sprowadza sie do jednej osi, a operator dostaje do oceny
obraz nieruchomy i wysrodkowany, a nie kadr uciekajacy w bok.

**Gimbal ma dwa rezimy i to jest wazniejsze niz jakikolwiek pojedynczy kat:**

| stan | kto rzadzi gimbalem |
|---|---|
| nie widzi celu | **misja** prowadzi skan: -90/-65/-40 na postoju, -55 w przelocie |
| widzi cel | **regulator** — jedzie tam, gdzie cel, az do limitu sprzetowego (~-18 st.) |

Te katy to KOMENDY w stanie "nic nie widze", a nie limity. Od chwili
wykrycia gimbal obraca sie dowolnie, wiec CELUJ domyka **obie osie**: yaw
ustawia nos, gimbal ustawia pochylenie, i cel laduje w srodku kadru.
Tam rzutowanie piksela jest najdokladniejsze, wiec podlot zaczyna sie
z najlepszego mozliwego pomiaru.

Warunek wyjscia liczymy w **pikselach** (`ex`, `ey`), nie w metrach. Powod:
przy gimbalu wysoko podniesionym gorne wiersze kadru moga patrzec ponad
horyzont, a rzutowanie zwraca wtedy `None` — i uchyb metrowy odczytalby sie
jako zero, czyli "wycentrowany", dokladnie wtedy, gdy nie jest.

### Sciezka B — cel POD dronem (dolot na waypoint)

```
  1. DOLOT         detektor MILCZY przez cala droge
  2. STOP          nad waypointem, gimbal w pion       <- start budzetu 20 s
  3. OKNO M z N    max 10 s — czy jest co centrowac
     widzi   -> 4. SPACJA 10 s, zadane RAZ -> ZAWIS + ZRZUT / na wspolrzedne
     nie widzi -> 3b. SKAN 360 st. Z WAYPOINTU (wp_scan_on_miss)
                     znalazl -> sciezka A: CELUJ -> SPACJA -> PODLOT -> ZAWIS
                     nie     -> ZRZUT NA WSPOLRZEDNE
```

**Krok 3b istnieje, bo adres bywa przesuniety.** Slaba geolokalizacja albo cel,
ktory ruszyl sie miedzy przelotem ortofoto a misja, daja waypoint obok celu.
Dotad jedyna reakcja na "nie widze" byl zrzut w ciemno na wspolrzedne — a cel
oddalony o 40 m jest DALEJ W ZASIEGU, tylko nie w nadirze. Skan tymi samymi
katami co w trybie SZUKAJ siega ok. 78 m wokol waypointu, czyli obejmuje caly
realny blad adresu.

Cel znaleziony tym skanem jest **OBOK, nie pod dronem**, wiec dalej idzie
sciezka A (z krokiem CELUJ), a nie sam ZAWIS — ten zaklada cel w nadirze.

**Pytania nie zadajemy, gdy celu nie widac.** Wczesniej misja pytala takze
wtedy, ale monit brzmial "[SPACJA] = nic nie zmieni, i tak zrzucam na
wspolrzedne" — obie odpowiedzi konczyly sie tak samo, a czekanie kosztowalo
pelne `confirm_timeout`. Teraz ten czas idzie na skan, ktory moze cos zmienic.

**Budzet 20 s od dolotu i pytanie zadane RAZ** — to sa dwa zabezpieczenia
przed ta sama petla: detekcja przychodzi, pytamy, cel znika, detekcja wraca,
pytamy znowu, i tak bez konca nad jednym waypointem.

Struktura jest linia prosta, zadna galaz nie wraca do tylu:

```
t=0     dolot, gimbal w pion
t<=10   czekam na okno M z N
        domknelo sie      -> pytam (z monitem "widze cel")
        minelo 10 s       -> pytam (z monitem "NIE widze celu")
t<=20   pytanie, 10 s, RAZ
        SPACJA + cel widoczny  -> ZAWIS
        SPACJA bez celu        -> ZRZUT NA WSPOLRZEDNE
        cisza                  -> ZRZUT NA WSPOLRZEDNE
```

Migoczaca detekcja nie ma wiec jak zapetlic misji: okno M z N ma jedno
podejscie o dlugosci 10 s, a odpowiedz operatora (albo jej brak) jest
ostateczna, nawet jesli cel w miedzyczasie zniknie i wroci.

ZAWIS ma juz wlasny limit `center_lost_timeout` (15 s ciaglego SEARCH = cel
zniknal na dobre), po ktorym schodzimy na ZRZUT NA WSPOLRZEDNE. Najgorszy
przypadek nad waypointem to wiec 20 + 15 = 35 s i **kazda sciezka konczy sie
zrzutem pod dobry adres.**

**W sciezce B nie ma krokow CELUJ i PODLOT** — i nie jest to oszczednosc,
tylko geometria. Dron stoi NAD celem, gimbal patrzy w pion. Namiar na cel
oddalony o 2 m jest nieokreslony, wiec obracanie sie nosem "w jego strone"
dawaloby dzikie komendy yaw z drobnych bledow piksela. Nie ma tez czego
podlatywac: zostaje sama korekta polozenia w metrach, czyli ZAWIS.

### Wspolny ogon

```
  ZAWIS   gimbal w pionie, korekta w metrach
          az cel blizej niz center_tol_m przez hover_hold_time
  ZRZUT
```

**SPACJA bramkuje PODLOT (A) albo CENTROWANIE (B), nigdy sam zrzut.**
Decyzja zapada wtedy, gdy operator ma co ocenic. Od tej chwili dron pracuje
sam — gdyby druga bramka stala przed samym zrzutem, utrata SSH w polowie
centrowania zostawilaby drona nad celem z ladunkiem i bez zgody.

Jesli cel zniknie w trakcie ZAWISU: w sciezce A ladunek zostaje (nie ma na co
zrzucic w ciemno), w sciezce B schodzimy na ZRZUT NA WSPOLRZEDNE — bo tam
adres caly czas jest.

---

## 3. Tryb DOWIEZ — mam waypoint

Sciezka B z sekcji 2.

```
lec nad waypoint            <- DETEKTOR MILCZY przez cala droge
zatrzymaj sie, gimbal w pion            <- rusza budzet 20 s
okno M z N, max 10 s
zapytaj RAZ, 10 s:
    SPACJA        -> ZAWIS na detekcji i zrzut tam
    nic przez 10s -> ZRZUT NA WSPOLRZEDNE
brak detekcji -> SKAN 360 st. z waypointu
    znalazl   -> CELUJ, SPACJA, PODLOT, ZAWIS, zrzut nad celem
    nie       -> ZRZUT NA WSPOLRZEDNE
nastepna klasa
```

Okno M z N otwiera sie DOPIERO nad waypointem i sluzy tylko do jednego:
odpowiedzi na pytanie "czy jest co centrowac". Nie jest bramka — przy braku
detekcji ladunek i tak idzie, na wspolrzedne.

**Cisza operatora znaczy teraz "zrzuc na wspolrzedne".** W `suas_full_mission`
znaczyla dokladnie odwrotnie — "to nie ten cel, szukaj dalej". To jest
najwieksza zmiana semantyczna w calej misji.

Uzasadnienie: skoro adres uznajemy za wiarygodny (sekcja 1), to domyslnym
dzialaniem ma byc jego wykonanie. Spacja jest **ulepszeniem** ("widze cel na
obrazie, wycentruj sie na nim zamiast lecec na sam punkt"), a nie zgoda na
cokolwiek. Zerwane lacze konczy sie wtedy zrzutem na dobry adres, a nie
gridem bez konca.

Dla czlowieka to jest jedyna sensowna sciezka: na 50 m ma 8 px, detektor go
nie potwierdzi, wiec centrowanie i tak sie nie odbedzie. **Waypoint JEST
odpowiedzia, detekcja jest premia.**

---

## 4. Tryb SZUKAJ — nie mam waypointu (tylko namiot)

Dwa punkty w pliku `.waypoints`:

```
WP1   srodek brzegu obszaru przeszukiwania   -> skan 180 st.
WP2   srodek obszaru przeszukiwania          -> skan 360 st.
WP1   POWROT — jeszcze jeden skan 180 st.    (scan_return_to_first)
```

Powrot na WP1 doklada misja sama, nie ma go w pliku. Wartosc jest przede
wszystkim w samym przelocie: druga droga nad polem to druga szansa detektora,
tym razem z przeciwnego kierunku.

```
na kazdym punkcie:  ZATRZYMAJ SIE i skanuj — obrot co 60 st.,
                    na kazdej pozycji gimbal gora-dol
miedzy punktami:    gimbal na -55 st., detektor patrzy caly czas
potwierdzenie w dowolnym momencie -> sciezka A z sekcji 2
nic po obu punktach -> RTL z ladunkiem
```

**Dron ma wykryc namiot STOJAC, nie w locie.** Stojac obraz nie jest rozmyty
ruchem, dron nie ucieka celowi z kadru, a okno M z N ma spokojnie czas sie
domknac. Lot miedzy punktami jest wtedy tylko przejazdem, a nie jedyna szansa
na detekcje.

### Krok obrotu: 60 stopni

Kadr jest kwadratowy (64,4 st.), ale jego **szerokosc w AZYMUCIE nie rowna sie
64,4 st.** — pochylony kadr rozszerza sie z odlegloscia, wiec azymut zalezy
i od kata gimbala, i od tego, jak daleko patrzymy:

| gimbal | pas terenu | srodek kadru | polowa azymutu na 40 m / 64 m / 78 m |
|---|---|---|---|
| -90 | -31 do +31 m | 0 m | caly obwod (kadr otacza nadir) |
| -65 | -6 do +78 m | 23 m | 44,4 / **35,5** / — |
| -55 | +2 do +119 m | 35 m | 45,2 / 37,4 / 34,7 |
| -40 | +16 do +365 m | 60 m | 44,7 / 38,6 / 36,6 |

Azymut jest **najwezszy na najdalszym zasiegu przy najnizszym pochyleniu** —
i to jest przypadek wiazacy: gimbal -65 na 64 m daje 2 x 35,5 = **71 st.**

```
krok 60 st.  ->  71 - 60 = 11 st. zapasu, czyli 18% zakladki
```

Zapas jest z tej strony, z ktorej trzeba: liczony na granicy pewnego wykrycia
(64 m, namiot 30 px — sekcja 5). Blizej jest jeszcze szerzej (88 st. na 40 m),
a dalej i tak nic nie widac.

- **WP1, 180 st.** -> 3 pozycje yaw. Pokrycie 3 x 71 = 213 st. > 180 st.
- **WP2, 360 st.** -> 6 pozycji yaw. Pokrycie 6 x 71 = 426 st. > 360 st.

### Kroki gimbala: -90, -65, -40

Na kazdej pozycji yaw gimbal przechodzi gora-dol, zeby **kazda odleglosc
trafila raz w SRODEK kadru**, a nie tylko na jego skraj:

| gimbal | srodek kadru | pas terenu |
|---|---|---|
| -90 | 0 m | -31 do +31 m |
| -65 | 23 m | -6 do +78 m |
| -40 | 60 m | +16 do +365 m |

Odstep 25 st. jest mniejszy niz polowa FOV (32 st.), wiec kolejne kadry
zachodza na siebie — nie ma szczeliny miedzy pierscieniami.

**Nadir robimy RAZ na punkt, bez obracania.** Przy -90 kadr otacza punkt pod
dronem ze wszystkich stron (kwadrat 63 m wysrodkowany na nadirze), wiec
powtarzanie go na kazdej pozycji yaw byloby fotografowaniem tego samego
terenu szesc razy.

### Ile to trwa

```
WP1:  1 nadir + 3 yaw x 2 pochylenia =  7 komorek x 1,5 s + 2 obroty = 13 s
WP2:  1 nadir + 6 yaw x 2 pochylenia = 13 komorek x 1,5 s + 5 obrotow = 25 s
                                                              razem   ~37 s
```

Jedna wartosc na symulacje i real, liczona pod **najgorszy przypadek 5 FPS**.

Skrocenie ma dwa zrodla i tylko jedno z nich to dwell. Przy `max_yaw_rate`
0,5 rad/s (29 st./s) jeden krok 60 st. trwa **2,1 s** — dluzej niz postoj.
Stad `scan_yaw_rate: 1.0` rad/s (57 st./s), osobny od `max_yaw_rate`, ktory
dalej ogranicza regulator w podlocie.

| dwell | yaw | razem |
|---|---|---|
| 2,0 s | 29 st./s | 55 s |
| 1,5 s | 29 st./s | 48 s |
| **1,5 s** | **57 st./s** | **37 s** |
| 1,0 s | 57 st./s | 27 s |

### Dlaczego 1,5 s, a nie 1,0

5 FPS nie znaczy rowno co 0,2 s. Detektor ma jitter do 0,7 s, wiec **efektywnie
miesci sie 2-5 klatek na sekunde** — to jest zmierzone i z tego powodu
`grid_acquire_timeout` w obecnym kodzie podniesiono z 1,0 na 3,0 s.

| dwell | klatek nominalnie | klatek najgorzej | 4 z 8 | 3 z 6 |
|---|---|---|---|---|
| 1,0 s | 5,0 | 2,0 | NIE | NIE |
| **1,5 s** | 7,5 | 3,0 | NIE | **TAK** |
| 2,0 s | 10,0 | 4,0 | TAK | TAK |

Dwell 1,0 s odpada niezaleznie od bramki. Zostaje wybor: **1,5 s z czulsza
bramka** albo **2,0 s ze standardowa**. Biore pierwsze — jest o 10 s szybsze
i lepiej ustawione wzgledem tego, co ta bramka naprawde robi.

### Bramka skanu jest inna niz bramka zrzutu: 3 z 6

Okno 4 z 8 jest kalibrowane pod sytuacje, w ktorej domkniecie okna **cos
uruchamia**. W skanie domkniecie okna nie uruchamia niczego oprocz sprawdzenia:
po nim ida jeszcze cztery bramki — SPRAWDZ na stojaco (pelne 4 z 8 kontrolera),
CELUJ, SPACJA i udane centrowanie.

Koszty sa wiec skrajnie niesymetryczne:

```
falszywe wyzwolenie skanu  ->  kilka sekund na ponowne sprawdzenie
przegapiony cel w komorce  ->  misja wraca z ladunkiem
```

Dlatego skan dostaje **3 z 6**, a `det_confirm_frames`/`det_window_frames`
kontrolera zostaja **4 z 8** i dalej bramkuja wszystko, co ma konsekwencje.
Zadaniem skanu jest nie przegapic; zadaniem sprawdzenia na stojaco jest nie
dac sie oszukac.

**Ciasna bramka na komorke jest bezpieczna, bo komorki na siebie zachodza:**

- azymut: zakladka 11 st. na 64 m, czyli 18% obwodu widziane dwukrotnie,
- pochylenia -65 (-6..78 m) i -40 (16..365 m) zachodza na **16..78 m** — cel
  w tym pasie jest w kadrze na DWOCH komorkach tej samej pozycji yaw,
- dyski WP1 i WP2 tez na siebie zachodza.

Redundancja siedzi w geometrii skanu, a nie w dlugosci postoju. Pojedyncza
przegapiona komorka nie znaczy przegapionego celu.

Jesli test w polu pokaze, ze komorki jednak wypadaja — podnies `scan_dwell`
do 2,0 s. Kosztuje to 10 s na cala misje.

### Dwie rzeczy do zrobienia w kodzie, zeby krotki dwell byl bezpieczny

**1. Zerowac okno M z N przy KAZDEJ zmianie komorki.** Okno jest krocacym
buforem 8 klatek, wiec trafienia z poprzedniej komorki zostaja w nim po
przestawieniu gimbala albo obrocie. Przy dwellu 2 s to sie samo wyplukiwalo,
przy 1 s juz nie: dwa trafienia z komorki k i dwa z k+1 domykaja okno na celu,
ktorego nie ma w zadnej z nich. Im krotszy dwell, tym wazniejsze.

**2. Chowac ruch gimbala w obrocie.** Kolejnosc pochylen na przemian
(-65 -> -40, potem na nastepnej pozycji -40 -> -65) sprawia, ze gimbal nigdy
nie wraca na pusto, a jego przejazd miedzy pozycjami yaw dzieje sie w trakcie
obrotu. Inaczej doliczasz ~0,4 s na komorke, czyli tyle, ile wlasnie
zaoszczedziles na dwellu.

### Kat przelotowy -55 i dlaczego `pitch_max` zostaje -30

`-55` miedzy punktami to ostatni kat, przy ktorym **caly kadr lezy na ziemi
z zapasem 23 st.** Powyzej -32 st. gorne wiersze kadru pokazuja niebo,
a rzutowanie piksela zwraca dla nich `None` (promien nie trafia w ziemie).

To jest argument za `pitch_transit: -55`, a **nie** za obnizeniem `pitch_max`.
Kat maksymalny zostaje -30, bo po wykryciu celu gimbal musi moc podniesc sie
na tyle, zeby go dogonic — cel 70 m przed dronem wymaga -35 st. Ograniczenie
do -55 uniemozliwialoby sledzenie czegokolwiek dalej niz 35 m, czyli polowy
wlasnego zasiegu wykrywania. Skan uzywa zreszta -40, wiec limit -55 wykluczylby
takze najwyzszy krok samego skanu.

---

## 5. Ile realnie widac — to decyduje o liczbie punktow skanu

Liczba pikseli celu zalezy od odleglosci SKOSNEJ, nie od wysokosci. Przy
`focal_px` = 813 i 50 m nad ziemia:

| odleglosc pozioma | namiot 3 m | czlowiek 0,5 m |
|---|---|---|
| 0 m (pod dronem) | 49 px | 8 px |
| 35 m | 40 px | 7 px |
| 64 m | **30 px** | 5 px |
| 100 m | 22 px | 4 px |
| 150 m | 15 px | 3 px |
| 250 m | 10 px | 2 px |

Punktem odniesienia jest **30 px** — tyle ma namiot z 80 m w nadirze, czyli
z pulapu, na ktorym geolokator normalnie pracuje i buduje klastry.

```
namiot 30 px (pewny)     -> promien poziomy  64 m
namiot 20 px (marginalny)-> promien poziomy 111 m
```

**Zalozenie "z dwoch punktow zobacze cale pole 150x250" nie domyka sie przy
30 px.** Pole ma 3,75 ha, a punkt skanu o promieniu 64 m pokrywa 1,29 ha —
i co gorsza 64 m to mniej niz polowa szerokosci pola (75 m), wiec rogi nie sa
pokryte z osi w ogole. Pelne pokrycie przy 30 px to okolo **6 punktow**
(dwie kolumny po trzy).

Przy 20 px rachunek wyglada inaczej i dwa punkty sa **prawie** wystarczajace:
z brzegu widac do 111 m, ze srodka kolejne 111 m w kazda strone, zostaje
pas ~14 m na przeciwleglym koncu.

**Wniosek praktyczny: to nie jest decyzja projektowa, tylko pomiar.** Przed
lotem zrob jeden test: postaw namiot, ustaw kamere na 50 m (albo odsun sie
o zmierzona odleglosc skosna) i sprawdz, z jakiej odleglosci detektor go
JESZCZE potwierdza oknem M z N. Wynik podstaw do tabeli i wyjdzie z niej
liczba punktow. Punkty sa w pliku `.waypoints`, wiec dolozenie trzeciego to
edycja pliku, nie zmiana kodu.

Obnizenie pulapu skanu pomaga mniej, niz sie wydaje: piksele zaleza od
odleglosci skosnej, wiec z 30 m widac dalej w poziomie przy tej samej liczbie
pikseli — ale cel jest wtedy ogladany pod katem ~68 st. od pionu, czyli
prawie z boku, a model byl uczony na ujeciach z gory.

---

## 6. Czym to sie rozni od `suas_full_mission`

| | `suas_full_mission` (teraz) | nowa misja |
|---|---|---|
| kolejnosc celow | pula obu klas, sortowana po `score` | sztywno: namiot, potem czlowiek |
| **cisza operatora** | "to nie ten cel" -> nastepny kandydat -> grid | **"zrzuc na wspolrzedne"** -> koniec klasy |
| kandydaci #2, #3 | cala lista z rankingiem, odrzucanie | jeden adres na klase |
| brak waypointu | grid liczony/offsety/plik, **bez konca** dopoki operator patrzy | 2 punkty skanu z pliku, RAZ, potem RTL |
| przed podlotem | dolot -> okno akwizycji -> spacja -> APPROACH | dolot -> STOP -> sprawdzenie na stojaco -> **CELUJ** -> spacja -> APPROACH |
| obrot na cel | w trakcie lotu, razem z `vx` (luk) | osobny krok na postoju (linia prosta) |
| szukanie z miejsca | `sweep_for_target` + spirala (obie domyslnie OFF) | obrot yaw na jednym kacie gimbala |
| gimbal bez celu | `pitch_search: -55` (jeden kat) | skan gora-dol `[-90,-65,-40]` na postoju, `pitch_transit: -55` w przelocie |
| szukanie na postoju | `sweep_for_target` (sam gimbal, domyslnie OFF) | skan yaw x gimbal, 60 st. krok — **glowny sposob wykrycia** |
| czlowiek bez waypointu | grid, potem zrzut | nie rusza, ladunek wraca |
| zrodla przerwania | `grid_det_interrupt` + `watch_sources` + polling `targets.json` | tylko detektor |
| parametry w yamlu | ~35 | ~15 |

Z `suas_grid_mission` nowa misja bierze to, co sie sprawdzilo: **rownolegla
obserwacje obu klas** (`ClassWatcher` — kontroler ma jedno okno i jeden topic
naraz, wiec grid musi liczyc wlasne), **hamowanie w miejscu z ponownym
sprawdzeniem** i **cisze dla klasy po falszywce**.

### Co przestaje istniec

`search_offsets`, `search_w/h/overlap`, `spiral_*`, `sweep_*`, `watch_sources`,
`auto_after_grid`, `watch_interval`, `grid_passes`, pula kandydatow i cale
czytanie rankingu z `targets.json` (zostaje samo `best` per klasa).

---

## 7. Gimbal: co zostaje bez zmian

`pitch_max` **zostaje -30.** Kuszace bylo obnizenie go do -55, skoro tyle
wynosi kat przelotowy — ale to zepsuloby dwie rzeczy naraz.

Po pierwsze, sledzenie: cel 70 m przed dronem wymaga gimbala na -35 st.
Limit -55 pozwalalby sledzic tylko to, co blizej niz 35 m, czyli polowe
wlasnego zasiegu wykrywania (sekcja 5).

Po drugie, predkosc podlotu. Kontroler liczy ja z POLOZENIA gimbala
w zakresie:

```python
span = self.pitch_max - self.pitch_min      # -30 - (-90) = 60
forward_ratio = (self.pitch_deg - self.pitch_min) / span
vx_target = self.kp_vx * forward_ratio
```

Zwezenie zakresu z 60 na 35 st. przesunelaby cala charakterystyke: gimbal
-70 dawal `ratio` 0,33 (1,3 m/s), a dawalby 0,57 (2,3 m/s) — ten sam kat,
prawie dwa razy szybciej. `kp_vx` trzeba by przestroic od nowa.

Skoro `pitch_max` zostaje, **`kp_vx` tez zostaje** i nic w regulatorze podlotu
nie wymaga dotykania.

---

## 8. Do rozstrzygniecia przed kodowaniem

1. **Ktory brzeg?** WP1 na srodku brzegu 150 m (pole ciagnie sie 250 m przed
   dronem) czy 250 m (pole ciagnie sie 150 m w bok)? Zakladam **150 m**, bo
   wtedy oba punkty leza na dluzszej osi i pokrywaja pole rownomiernie.
2. **Ile punktow skanu?** Wynik pomiaru z sekcji 5. Dwa to minimum, ktore
   dziala tylko przy optymistycznym progu 20 px.
3. **Obrot: 360 czy 180 st.?** Na WP1 (brzeg) pole jest w calosci z przodu,
   wiec wystarczy +-90 st. Na WP2 (srodek) potrzebne jest pelne 360 st.
   Przy 15 st./s pelny obrot to 25 s, polowka 13 s.
4. **Skan takze na WP2** — zakladam TAK (to samo zachowanie na obu punktach).
   Napisz, jesli WP2 mial byc tylko punktem przelotowym.
5. **Nie ma operatora w trybie SZUKAJ** (zerwane lacze, brak pulsu z GUI).
   Nie ma waypointu, wiec nie ma na co zrzucic w ciemno. Proponuje regule
   z `suas_grid_mission`: bez operatora zrzut **wylacznie po udanym
   wycentrowaniu**, a cel, ktory zniknie w trakcie centrowania, nie dostaje
   ladunku. To jest sprawdzone — 2026-09-02 grid potwierdzil w ten sposob
   drzewo.
6. **Co po zrzucie namiotu w trybie SZUKAJ?** Zakladam RTL od razu (czlowiek
   bez waypointu i tak jest poza gra), czyli wracamy z jednym ladunkiem.
7. **Tryb mieszany.** Namiot ma adres, czlowiek nie (to najbardziej
   prawdopodobny przypadek na zawodach, bo automat nie zapisuje czlowieka
   z pulapu ortofoto). Regula z sekcji 1 obsluguje go bez zadnego trybu:
   namiot idzie sciezka DOWIEZ, czlowiek nie rusza. **Nie ma przelacznika
   A/B — jest pytanie o adres, zadawane osobno dla kazdej klasy.**
8. **Nazwa wezla.** Proponuje `suas_mission` — ma zastapic `full` i `grid`,
   wiec zasluguje na nazwe bez przymiotnika.

---

## 9. Pliki i uruchomienie

```
src/drone_autonomy/drone_autonomy/suas_mission.py    wezel
src/drone_bringup/config/misja.yaml                  parametry
src/drone_bringup/config/misja_skan.waypoints        trasa skanu
src/drone_bringup/config/targets.json                adresy (pisze geolokator)
```

Wszystkie cztery leza w `src/`, czyli w katalogu **zamontowanym w kontenerze
Dockera** — plik jest ten sam na hoscie i w kontenerze. W `~/suas_targets`
nie byl, co bylo cichym zrodlem pomylek.

### Build

```bash
cd ~/Dron_symulacja
colcon build --packages-select drone_autonomy drone_bringup
source install/setup.bash
```

### Misja

Musi isc przez `ros2 run`, a **nie z launcha** — `wait_confirm` czyta
klawiature i potrzebuje stdin podpietego do terminala.

**Sciezka do yamla zalezy od tego, gdzie jestes.** Docker montuje tylko
katalog `src/`, wiec `~` znaczy co innego po kazdej stronie:

```bash
# Jetson / host
ros2 run drone_autonomy suas_mission --ros-args \
    --params-file ~/Dron_symulacja/src/drone_bringup/config/misja.yaml

# kontener Docker (~ to /root, repo jest pod /root/ros_ws/src)
ros2 run drone_autonomy suas_mission --ros-args \
    --params-file /root/ros_ws/src/drone_bringup/config/misja.yaml
```

Blad `Couldn't parse params file ... Error opening YAML file` znaczy
DOKLADNIE tyle, ze pliku nie ma pod ta sciezka — nie ze yaml jest zly.

Sciezki WEWNATRZ yamla maja ten sam problem, dlatego `scan_waypoints` jest
podana sama nazwa pliku: misja sprawdza po kolei `/root/ros_ws/src/...`,
`~/Dron_symulacja/src/...` i katalog `share` pakietu, po czym loguje, ktory
plik wziela. `targets_json` **nie** jest tak podmieniany — plik z innego
srodowiska moze byc z poprzedniego lotu, a zrzut pod nieaktualny adres jest
gorszy niz brak adresu. Misja tylko powie, gdzie taki plik lezy, i przy
kazdym odczycie loguje jego wiek.

Test bez trasy AUTO (sam uzbraja i wznosi sie na `target_alt`):

```bash
ros2 run drone_autonomy suas_mission --ros-args \
    --params-file ~/Dron_symulacja/src/drone_bringup/config/misja.yaml \
    -p auto_takeoff:=true
```

Pojedyncze parametry nadpisuje sie przez `-p nazwa:=wartosc` PO
`--params-file`. Przydatne:

| parametr | domyslnie | po co |
|---|---|---|
| `auto_takeoff` | `false` | `true` = test bez trasy AUTO |
| `drop_servo_ch` | `0` | `0` = zrzut tylko w logu; **na realu 13** |
| `scan_dwell` | `1.5` | podnies do `2.0`, jesli komorki wypadaja |
| `scan_return_to_first` | `true` | `false` = bez powrotu na WP1 |
| `finish_action` | `rtl` | `rtl` / `land` |

### Zeby geolokator pisal do config/

```bash
ros2 launch drone_bringup suas_geolocator.launch.py \
    save_dir:=/home/jetsonknr/Dron_symulacja/src/drone_bringup/config
```

`targets.json` laduje wtedy wprost w `config/`, a katalogi z danymi lotu
(`observations.csv`, `kandydat_NN.jpg`) obok niego. **Brak pliku nie jest
bledem** — znaczy tyle, ze zadna klasa nie ma adresu i namiot idzie skanem.

### Parametry, ktore wprowadza ta misja

```yaml
# sciezka B — budzet nad waypointem
wp_budget:      20.0     # od dolotu do decyzji
wp_acquire:     10.0     # okno M z N nad waypointem
confirm_timeout:10.0     # pytanie zadawane RAZ
wp_scan_on_miss: true    # nie widze pod dronem -> skan zamiast zrzutu w ciemno
wp_scan_arc_deg: 360.0
wp_scan_timeout: 120.0

# CELUJ
aim_tol_px:     0.06     # kiedy cel jest w srodku kadru
aim_hold:        1.0
aim_timeout:    15.0

# skan
scan_yaw_step:  60.0                    # 18% zakladki na 64 m
scan_arc_deg:   [180.0, 360.0]          # luk per punkt
scan_pitch_nadir: -90.0                 # raz na punkt, bez obracania
scan_pitches:   [-65.0, -40.0]          # na kazdej pozycji yaw
scan_dwell:      1.5                    # ta sama wartosc sim i real
scan_yaw_rate:   1.0                    # rad/s TYLKO na skan
scan_confirm_frames: 3                  # czulsza bramka skanu (M)
scan_window_frames:  6                  # (N)
scan_return_to_first: true
search_person_after_tent: true          # skan na czlowieka po zrzucie na namiot
person_scan_timeout: 300.0
pitch_transit:  -55.0                   # kat w przelocie
```

**Regulatory sa przepisane 1:1 z `suas_simple_mission`** — tam caly lancuch
SEARCH -> APPROACH -> HOVER jest oblatany, wiec nie ma powodu ich ruszac.
Zgadza sie wszystkie 24: `kp_vx` 4.0, `kp_hover` 0.2, `kp_alt` 0.5, `kp_yaw`
0.3, `max_vel` 3.0, `max_vz` 1.5, `max_yaw_rate` 0.5, `ema_alpha` 0.15,
`damping` 0.6, `gimbal_deadzone` 0.06, `hover_deadzone_m` 0.5, `lost_timeout`
3.0, katy `pitch_*` i okno 4 z 8 z `det_confirm_gap` 1.5.

Bez zmian zostaja: `pitch_max` -30 i `kp_vx` (sekcja 7), `target_alt` 50,
okno KONTROLERA `det_confirm_frames` 4 z `det_window_frames` 8,
`det_confirm_gap` 1.5, `require_same_track` true, `center_tol_m` 1.5,
`hover_hold_time` 5, `center_lost_timeout` 15, `det_cooldown` 20,
`brake_settle_time` 3 i cala sekcja zrzutu.
