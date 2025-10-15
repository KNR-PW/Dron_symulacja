# Jak odpalić symulacje PX4
## krok 1
By odpalić symulacje px4 musicie pobrac docker huba image obrazu:
```bash
docker image pull dierust/knr_px4_sim:latest 
```
## krok 2
Następnie musicie wejść do folderu **docker/docker_px4** i tam jest specjalnie przygotowany skrypt do stworzenia konteneru
```bash
cd docker/docker_px4
./setup_container_gpu.sh dierust/knr_px4_sim:latest 
```
## krok 3
Włącz colcon builda w folderze rosowym konteneru następnie wpisz 
```bash
source install/setup.bash
MicroXRCEAgent udp4 -p 8888
```
Oraz odpal skrypt o nazwie **./run_px4_sitl** i zobacz czy widzisz topici nadawne przez drona jezeli widzisz je i symulacje to znaczy że wszystko wykonało się poprawnie