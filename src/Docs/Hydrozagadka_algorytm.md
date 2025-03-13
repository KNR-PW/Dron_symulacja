
# Hydrozagadka - algorytm misji

1. Przygotowanie do lotu
2. Takeoff
3. Lokalizacja zbiornika na wode oraz miejsca zrzutu
4. Podejscie do zbiornika
5. Pobranie probki
6. Powrot do punktu zrzutu
7. Wroc do punktu 4 dopoki nie spelniles zalozen zadania (liczba przeniesionych probek)
8. Powrot do miejsca ladowania

## Wiadomosci MAVLINK do wykorzystania w poszczegolnych krokach

Najwazniejsze wiadomosci:
A -> setMode(GUIDED)
B -> arm/disarm
C -> takeoff(alt)
D -> goToPosition(x,y,z)
E -> setVelocity(vx,vy,vz)
F -> goToLocalization(lat,lon,alt)
G -> goToLocalizationSmooth(lat,lon,alt)
H -> 