#!/bin/bash
# Sprzatanie po nieudanym uruchomieniu symulacji.
#
# Objaw, ktory to leczy: SITL wypisuje "Waiting for heartbeat" i "link 1 down",
# Gazebo zalewa "Duplicate input frame", a drone_handler dostaje
# "Connection refused" na porcie 5762.
# Przyczyna: zostawiony proces arducopter trzyma porty FDM (9002/9003), wiec
# nowa instancja nie moze sie zainicjalizowac i nigdy nie wysyla heartbeatow.
docker exec knr_drone bash -c "pkill -9 -f arducopter; pkill -9 -f mavproxy; pkill -9 -f 'gz sim'; pkill -9 -f ros_gz_bridge; pkill -9 -f drone_handler; sleep 1; echo '--- co zostalo ---'; ps aux | grep -E 'arducopter|mavproxy|gz sim' | grep -v grep || echo 'czysto'"
