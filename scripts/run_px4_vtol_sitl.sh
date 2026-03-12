
# Przykład: ./run_px4_vtol_sitl.sh small_city

WORLD=${1:-aruco}

cd ../KNR_Drone_PX4_Autopilot


# Odśwież CMake tylko jeśli cel nie istnieje
if ! ninja -C build/px4_sitl_default -t targets 2>/dev/null | grep -q "gz_knr_tiltrotor_$WORLD"; then
    echo "Nowy świat - odświeżam CMake..."
    touch src/modules/simulation/gz_bridge/CMakeLists.txt
fi

# Ustaw pozycję spawnu drona (x,y,z)
export PX4_GZ_MODEL_POSE="0,0,4"
export PX4_GZ_NO_FOLLOW=1

docker exec -it -e PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE" -e PX4_GZ_NO_FOLLOW="$PX4_GZ_NO_FOLLOW" knr_drone_px4 bash -c "cd ~/Dron_symulacja/KNR_Drone_PX4_Autopilot && make px4_sitl gz_knr_tiltrotor_$WORLD"