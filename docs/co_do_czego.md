# Poradnik co każdy package robi?
## Wstęp
Jak widać nasz projekt rozsłusł sie do sporych rozmiarów i na początek, jest dość przytłaczający dla nowego użytkownika. Ten poradnik to zmieni.
## Czemu jest tak dużo folderów w /src?
Każdy folder w **/src** to osobny package rosowy który został od zera stworzony przez nas, lub zmodyfikowany. Każdy osobny package ma inne zadanie i może mieć wiele nodów, które mają wspólny cel.
## Rozumienie naszych package
### drone_autonomy
Pakiet odpowiedzialny za przechowywanie wysokopoziomowych misji na dronie. To w nim zapisujemy misje które ma wykonać dron. Wszystkie obecne misje wykorzystują klasę **DroneController** która znajduję się w folderze **drone_comunication**. W tej klasie są opisane wszystkie działające serwisy, akcje i topici które wykorzystujemy podczas lotu.
### drone_bringup
Pakiet odpowiedzialny za przechowywanie launch files które wykorzystujemy podczas latania dronem, bądź symulacji. Obowiązkowo zachęcam do zapoznania się z **drone_dev.launch.py**, który służy do testowania drona, oraz **drone_simulation.launch.py** który odpala całą symulację.
### drone_camera
Pakiet odpowiedzialny za nody związane z kamerą. Każdy z nodów powinien posiadać parametr odpowiedzialny za wybór subskrybowanego topica. Najważniejsze obecnie nody to **images_recoreder.py** i **video_recoreder.py**.
### drone_detector
Pakiet odpowiedzialny za nody związane z detekcją najróżniejszych rzeczy.   
### drone_gui
Pakiet odpowiedzialny za interaktywną aplikację do łatwiejszego przeprowadzania testów, aktualnie będzie to jeden z głównych celów by postawić to na nogi.
### drone_hardware
Pakiet odpowiedzialny za komunikację pomiędzy komputerem pokładowym a flight controlerem, oraz używaniem portów GPIO przez rasberry pi. Dla ciekawych jak wysyłamy polecenia przez MavLinka zachęcam do przeglądu nodu **drone_handler.py**.
### drone_interfaces
Pakiet odpowiedzialny za przechowywanie wszystkich customowych interfacesów, jakie tworzymy w naszym projekcie.
### drone_web
Pakiet odpowiedzialny za komunikację pomiędzy rosem a stroną web.
### webots_simulation
Pakiet odpowiedzialny za przechowywanie światów weboots, oraz parametrów do SITLa.
### reszta
Reszta pakietów nie jest aż tak ważna i została zostawiona w ramach historii bądź potencjalnego ponownego wykorzystania.