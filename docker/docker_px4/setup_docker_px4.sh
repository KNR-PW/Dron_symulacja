docker exec -it knr_drone_px4 bash -c "/root/Dron_symulacja/KNR_Drone_PX4_Autopilot/Tools/setup/ubuntu.sh --no-nuttx"
docker exec -it knr_drone_px4 bash -c "git config --global --add safe.directory /root/Dron_symulacja/KNR_Drone_PX4_Autopilot/platforms/nuttx/NuttX/nuttx"
docker exec -it knr_drone_px4 bash -c "git config --global --add safe.directory /root/Dron_symulacja/KNR_Drone_PX4_Autopilot/src/modules/mavlink/mavlink"
docker exec -it knr_drone_px4 bash -c "git config --global --add safe.directory /root/Dron_symulacja/KNR_Drone_PX4_Autopilot"
docker exec -it knr_drone_px4 bash -c 'echo "export GZ_CONFIG_PATH=/usr/share/gz" >> /root/.bashrc'