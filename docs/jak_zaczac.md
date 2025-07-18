# Poradnik jak połapać się w naszym projekcie

## Podstawowe umiejętności wymage do pracy w projekcie:
* Podstawy pythona
* Podstawowa wiedza na temat programowania obiektowego
* Podstawy ROS2
* Podstawy Git'a
## Jak działać w symulacji
1. Wykonać tutorial z pliku README.md **Uwaga** do odpalenia symulacji w sposób w niej opisany potrzebujecie komputera z linuxem jeżeli ktoś ma ochotę zachęcam do próby konfiguracji na Windowsa.
2. Jeżeli został wykonany punkt 1 i symulacja wam działa to stwórzcie nowy branch.
3. Napiszcie testową misję w folderze **src/drone_autonomy/drone_autonomy** pod nazwą **test_mission.py**
4. Zapiszcie zmiany i stwórzcie 3 nowe terminale.
5. W pierwszym z nich włączcie konetener **knr_drone**, następnie przedcie do katalogu **docker** i wpiszcie tą komendę (odpala ona emulator flight controlera w dokerze)
```bash
sudo ./run_ardupilot_sitl.sh
```
6. W drugim terminalu wpiszcie tą komendę (builduje cały projekt, oraz uruchamia ona webootsa i **drone_simulation.launch.py**)
```bash
sudo ./build_and_run.sh
```
7. W trzecim terminalu wpiszcie tą komendę (odpala waszą misję napisaną w **test_mission.py**) 
```bash
sudo ./run_test_mission.sh
```
8. **Uwaga** podczas uruchamiana misji sitl może stwierdzić że z jakiejś przyczyny nie zarmuje on drona w tedy w 1 terminalu(ten który ma sitla odpalonego) wpiszcię tę komendę. Jeżeli ta komenda nie pomaga zgłoście problem na grupie.
```bash
arm throttle 
``` 
9. Jeżeli wasz kod działa tak jak tego oczekujemy możecie zpuszować wasze zmiany na repo i zrobić pull requesta (oczywiście podczas pushowania robicie nowy branch, jeżeli wykonaliście punkt 2 git sam wam podpowie jak to zrobić)
