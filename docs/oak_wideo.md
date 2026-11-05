# Wideo z OAK

domyslnie :
zapis z topicu `/oak/rgb/image_raw`
zapis do `~/oak_video/1`, `~/oak_video/2`...
nagrywanie startuje samo, `Ctrl+C` konczy i domyka plik

## Oak_raw
```bash
ros2 launch drone_bringup oak_video.launch.py
ros2 launch drone_bringup oak_video.launch.py camera_topic:=/oak/rgb/image_raw fps:=15.0
```

## Detekcja:
```bash
ros2 launch drone_bringup oak_video.launch.py camera_topic:=/tent_detections/image fps:=8.0
```