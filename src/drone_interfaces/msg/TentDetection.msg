# Detekcja namiotu na obrazie z kamery
# detected: true jesli namiot wykryty, false jesli brak
# bounding_box: [x, y, w, h] w pikselach oryginalnego obrazu (lewy gorny rog + rozmiar)
# confidence: pewnosc detekcji (0.0-1.0), 0.0 gdy brak detekcji
bool detected
float32[4] bounding_box
float32 confidence
