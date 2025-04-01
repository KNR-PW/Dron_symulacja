# Tutorial jak zrobić kontener na rasberce
## Krok 1
Pobierz najnowszą wersje naszego repo komendą
```bash
git clone https://github.com/KNR-PW/Dron_symulacja.git
```
## Krok 2
Wejdź w folder **Dron_symulacja/docker_rpi**\
i uruchom **build_image.sh** za pomocą komendy
```bash
sudo ./build_image.sh
```

Następnie urucho skrypt **make_container.sh** za pomocą komendy:
```bash
sudo ./make_container
```
Ciesz sie działającym dockerem