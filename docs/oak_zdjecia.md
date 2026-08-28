# Zdjecia z OAK

domyslnie :
zapis z topicu `/oak/rgb/image_raw` 
zapis do `~/oak_photos/1`, `~/oak_photos/2`... 

```bash
ros2 launch drone_bringup oak_photos.launch.py

ros2 launch drone_bringup oak_photos.launch.py camera_topic:=/oak/rgb/image_raw fps:=0.5
```

## Wybor 1 topicu

```bash
ros2 launch drone_bringup oak_photos.launch.py camera_topic:=/tent_detections/image fps:=2.0
```

## Dwa zapisy naraz, kazdy z innego topicu



```bash
ros2 launch drone_bringup oak_photos.launch.py second:=true \
  camera_topic:=/oak/rgb/image_raw      fps:=2.0 \
  camera_topic2:=/tent_detections/image  fps2:=2.0
```
