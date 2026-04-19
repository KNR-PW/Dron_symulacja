
# Przykład: ./run_px4_vtol_sitl.sh small_city

WORLD=${1:-aruco}

cd ../KNR_Drone_PX4_Autopilot

# Wyczyść stare instancje PX4/GZ wewnątrz kontenera, żeby uniknąć:
# "PX4 server already running for instance 0"
docker exec knr_drone_px4 bash -c "pkill -9 -x px4 2>/dev/null || true; pkill -9 -f 'gz sim' 2>/dev/null || true; rm -f /tmp/px4-*.lock /tmp/px4_instance_*.lock 2>/dev/null || true"


# Odśwież CMake tylko jeśli cel nie istnieje
if ! ninja -C build/px4_sitl_default -t targets 2>/dev/null | grep -q "gz_knr_tiltrotor_$WORLD"; then
    echo "Nowy świat - odświeżam CMake..."
    touch src/modules/simulation/gz_bridge/CMakeLists.txt
fi

# Ustaw domyślną pozycję spawnu drona (x,y,z)
export PX4_GZ_MODEL_POSE="0,0,45"
#export PX4_GZ_NO_FOLLOW=0

docker exec -it -e PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE" -e PX4_GZ_NO_FOLLOW="$PX4_GZ_NO_FOLLOW" knr_drone_px4 bash -c "cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && make px4_sitl gz_knr_tiltrotor_$WORLD"