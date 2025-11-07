1. Musisz byc w sieci knr
2. Sprawdz ip wejdz na strone  192.168.77.1
3. Połącz się z ssh ( user: knr, host: ip, hasło:(dostępne u rafała) do rasberry pi
4. Sprawdz czy rpicam-hello działa ( jeżeli używasz rpicama)
5. wejdz do Dron_symulacja    source install/setup.bash
6. W zależności od drona (MAMBA-OAK, WROBELEK-RPI CAM) 
* WROBELEK: ros2 launch drone_bringup drone_wrobelek.launch.py
* MAMBA: ros2 launch drone_bringup drone_mamba.launch.py

