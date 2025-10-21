# Poradnik jak skąfigurować Qgroundcontrol z symulacja
## krok 1 
Musisz stworzyć nowy kontener z symulacją w ten sam sposób jaki jest opisany w poradniku **README.md**, musisz to zrobić ponieważ do skryptu dodałem nową flagę która pozwala na forwardowanie portu TCP przez Dockera
## krok 2
Zgodnie z dokumentacją PX4 pobierz QGroundControl na swoją maszynie [link](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
## krok 3
Włącz Qgroundcontrol oraz symulacje w kontenerze, następnie kliknij w lewym górnym rogu w logo aplikacji wejdź w ***Aplication Settings*** następnie w ***Comm Links*** i tam naciśnij guzik **add**
## krok 4
Wpisz dowolną nazwe połączenia np simulation, dalej zmień typ połączenia z **Serial** na **TCP**\
Następnie w **Serwer Address** wpisz ten link: 
```bash
127.0.0.1
```
A w **Port** wpisz te cyfry:
```bash
5763
```
Możesz włączy opcje automatycznego Startu by aplikacja zawsze próbowała podłączyć się do tego portu
## krok 5
Ciesz się skonfigurowanym gcs