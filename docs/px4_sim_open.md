# Jak odpalić symulacje PX4
docker build -t knr_image_px4 .

```bash
cd docker/docker_px4
./setup_container_gpu.sh knr_image_px4
```

docker exec -it knr_drone_px4 bash -c "source /opt/ros/jazzy/setup.bash && MicroXRCEAgent udp4 -p 8888"
