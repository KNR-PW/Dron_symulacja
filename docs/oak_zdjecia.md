# Zdjecia z OAK

Zapisuje pojedyncze klatki z `/oak/rgb/image_raw` — tego samego surowego topicu,
ktory czyta detektor, wiec bez strat JPEG z podgladu. Kamera musi juz chodzic
(np. z `suas_detect_jetson.launch.py`), ten launch jej nie uruchamia.

```bash
ros2 launch drone_bringup oak_photos.launch.py
ros2 launch drone_bringup oak_photos.launch.py fps:=2.0 save_dir:=/home/jetsonknr/zdjecia
```

Zapis do `~/oak_photos/1`, `~/oak_photos/2`... — nowy podkatalog przy kazdym starcie.
