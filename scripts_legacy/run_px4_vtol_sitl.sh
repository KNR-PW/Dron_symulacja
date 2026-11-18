
# Przykład: ./run_px4_vtol_sitl.sh small_city

WORLD=${1:-aruco}

cp -r ../knr_tiltrotor ../KNR_Drone_PX4_Autopilot/Tools/simulation/gz/models
cd ../KNR_Drone_PX4_Autopilot


# Odśwież CMake tylko jeśli cel nie istnieje
if ! ninja -C build/px4_sitl_default -t targets 2>/dev/null | grep -q "gz_knr_tiltrotor_$WORLD"; then
    echo "Nowy świat - odświeżam CMake..."
    touch src/modules/simulation/gz_bridge/CMakeLists.txt
fi

# Ustaw pozycję spawnu drona (x,y,z)
export PX4_GZ_MODEL_POSE="0,0,4"

make px4_sitl "gz_knr_tiltrotor_$WORLD"