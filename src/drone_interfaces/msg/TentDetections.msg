# Komplet detekcji z JEDNEJ klatki - zrodlo prawdy dla geolokalizacji.
#
# W odroznieniu od pojedynczego TentDetection (ktory niesie tylko najpewniejszy
# box danej klasy) tutaj sa WSZYSTKIE boxy z klatki, obu klas naraz.
#
# header.stamp = stamp klatki kamery, nie moment publikacji detekcji. Dzieki temu
# odbiorca moze siegnac po telemetrie z chwili, w ktorej klatka powstala, zamiast
# zgadywac opoznienie inferencji.
std_msgs/Header header
uint32 img_w
uint32 img_h
TentDetection[] detections
