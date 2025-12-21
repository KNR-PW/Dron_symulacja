# Poradnik jak skąfigurować Qgroundcontrol z symulacja
## krok 1 
Musisz stworzyć nowy kontener z symulacją w ten sam sposób jaki jest opisany w poradniku **README.md**, musisz to zrobić ponieważ do skryptu dodałem nową flagę która pozwala na forwardowanie portu TCP przez Dockera
## krok 2
Zgodnie z dokumentacją PX4 pobierz QGroundControl na swoją maszynie [link](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)
## krok 3
Włącz Qgroundcontrol oraz symulacje w kontenerze, następnie kliknij w lewym górnym rogu w logo aplikacji wejdź w ***Aplication Settings*** następnie w ***Comm Links*** i tam naciśnij guzik **add**
## krok 4

Wpisz dowolną nazwę połączenia, np. **simulation**, następnie zmień typ połączenia z **Serial** na **TCP**.  
W polu **Server Address** wpisz:


```bash
127.0.0.1
```

a w polu **Port**:


```bash
5763
```

**Alternatywnie** możesz wybrać typ połączenia **UDP** – wtedy wystarczy zostawić domyślne ustawienia i QGroundControl zazwyczaj **połączy się automatycznie** z symulacją.

Dodatkowo możesz włączyć opcję **Automatically Connect on Start**, aby aplikacja zawsze próbowała połączyć się z symulacją przy starcie.

## krok 5
Ciesz się skonfigurowanym gcs
