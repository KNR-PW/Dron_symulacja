# Detekcja namiotu na obrazie z kamery
# detected: true jesli namiot wykryty, false jesli brak
# bounding_box: [x, y, w, h] w pikselach oryginalnego obrazu (lewy gorny rog + rozmiar)
# confidence: pewnosc detekcji (0.0-1.0), 0.0 gdy brak detekcji
# track_id: ID sciezki z trackera (ByteTrack); -1 = brak detekcji lub tracker
#           jeszcze nie przypisal ID. Ten sam obiekt trzyma to samo ID miedzy
#           klatkami -> pozwala odrzucic falszywki "migajace" w losowych miejscach.
bool detected
float32[4] bounding_box
float32 confidence
int32 track_id
