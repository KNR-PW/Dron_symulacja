# Wideo z OAK

Nagrywa `.mp4` z `/oak/rgb/image_raw` — tego samego surowego topicu, ktory czyta
detektor. Kamera musi juz chodzic; nagrywanie wlacza sie i wylacza serwisami, a
plik zamyka sie dopiero po `turn_off_video`.

```bash
ros2 launch drone_bringup oak_video.launch.py

ros2 service call knr_video/turn_on_video  drone_interfaces/srv/TurnOnVideo  "{}"
ros2 service call knr_video/turn_off_video drone_interfaces/srv/TurnOffVideo "{}"
```

Zapis do `~/oak_video/1`, `~/oak_video/2`... Ustaw `fps:=` na realna czestotliwosc
topicu (`ros2 topic hz /oak/rgb/image_raw`), inaczej wideo odtworzy sie za szybko.
