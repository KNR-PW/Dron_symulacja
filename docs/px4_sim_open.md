# Jak odpalić symulacje PX4
## krok 1
By odpalić symulacje px4 musicie pobrac docker huba image obrazu:
```bash
docker image pull dierust/knr_px4_sim:latest 
```
## krok 2
W katalogu Dron_symulacja musicie wykonać komende
```bash
git submodule update --init --recursive
```
Oraz w katalogu KNR_Drone_PX4_Autopilot
```bash
cd KNR_Drone_PX4_Autopilot && git submodule update --init --recursive
```
Teraz musisz zainicjalizować pakiet PX4
```bash
./Tools/setup/ubuntu.sh
```
Po czym **zrestartuj komputer!!!**
Następnie musicie wejść do folderu **docker/docker_px4** i tam jest specjalnie przygotowany skrypt do stworzenia konteneru
```bash
cd docker/docker_px4
./setup_container_gpu.sh dierust/knr_px4_sim:latest 
```
### Uwaga teraz każdą kolejną komende odpalamy w koleinych terminalach
## krok 3
Teraz będziemy musieli potrzebować włączyć symulacje w gazeboo i sitl px4
do właczenia modelu drona w trybie multiwirnikowca
```bash
./run_px4_quad_sitl.sh
```
do właczenia modelu drona w trybie vtola
```bash
./run_px4_vtol_sitl.sh
```
## krok 4
W tym kroku będziemy uruchamiać aplikacje microxrcdds która tłumaczy wewnętrzne wiadomości (uORB) PX4 na rosowe wiadomości
```bash
./run_microxrce.sh
```

## krok 5
teraz musisz odpalić qgoundcontrol by dron mógł być zarmowany co było wytłumaczone w **gcs_config.md**

## krok 6
Możemy teraz odpalic nasze nody rosowe
```bash
./build_and_run.sh
```
## krok 7
Możemy teraz odpalic naszą misje testową
```bash
./run_test_mission.sh
```
