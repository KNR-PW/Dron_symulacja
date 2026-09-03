# Plan na jutro — łączność Jetson przez LTE/Tailscale (lag wideo, zamarzanie terminali)

> Status: **propozycja diagnostyczna**, spisana bez Jetsona pod ręką.
> Objaw: podgląd web tnie się, czasem zamarzają WSZYSTKIE terminale naraz.
> Setup: Jetson → internet przez modem LTE SIM7600 → Tailscale → laptop/telefon.

## Na co stawiam (hipotezy, od najbardziej prawdopodobnej)

1. **Uplink LTE wysycony wideo + bufferbloat.** MJPEG z `web_video_server` zjada
   cały (cienki) upload modemu. Bufor się zapycha, opóźnienie skacze do sekund,
   a interaktywny SSH na tym samym łączu głodnieje → „zamarzają wszystkie
   terminale naraz". To jest podpis wspólnego, wysyconego łącza bez kolejkowania.
2. **Tailscale przez DERP (relay), nie direct** — LTE zwykle za CGNAT operatora,
   więc dziura NAT się nie przebija i ruch leci przez serwer pośredniczący:
   +opóźnienie, +wąskie gardło.
3. **Słaby sygnał / zły tryb SIM7600** — spadek na 3G, brak anteny diversity,
   niski SINR → jitter, retransmisje, chwilowe zrywy.
4. (mniej prawdopodobne) MTU/fragmentacja na WireGuard po LTE, zasilanie modemu.

Kolejność bloków niżej idzie od „potwierdź główną przyczynę" do „załóż obejścia".

---

## Blok 0 — przygotowanie (2 min)

Na laptopie i na Jetsonie:
```bash
tailscale status            # obie strony online? jaki adres 100.x ma Jetson?
```
Zapisz adres Tailscale Jetsona jako `JET=100.84.102.43` (albo aktualny).

---

## Blok A — direct czy DERP? (5 min) [potwierdza hipotezę 2]

Na laptopie:
```bash
tailscale ping $JET
```
- **`pong ... via DERP(xxx)`** → idzie przez RELAY. To jest problem — direct
  by zdjął część opóźnienia i wąskie gardło.
- **`pong ... via 100.x.x.x:port` / adres publiczny** → masz direct, hipoteza 2
  odpada, skup się na A/B.

Dodatkowo opóźnienie i straty:
```bash
ping -c 100 $JET            # patrz na avg, mdev (jitter), % packet loss
```
**PASS orientacyjnie:** avg < 80 ms, mdev < 30 ms, loss < 1%.
Duży `mdev` lub loss > 2% = łącze niestabilne (hipoteza 1/3).

---

## Blok B — przepustowość i bufferbloat (10 min) [potwierdza hipotezę 1]

Zainstaluj `iperf3` po obu stronach (`sudo apt install iperf3`).

Na Jetsonie:
```bash
iperf3 -s
```
Na laptopie — **upload Jetsona to kluczowa liczba** (Jetson wysyła wideo):
```bash
iperf3 -c $JET -R -t 20      # -R: serwer (Jetson) wysyła do nas = jego UPLINK
iperf3 -c $JET    -t 20      # bez -R: nasz upload do Jetsona (mniej istotny)
```
Zapisz Mbit/s uplinku Jetsona. **Jeśli < ~5 Mbit/s** — wideo MJPEG go zapcha.

**Test bufferbloatu (to jest sedno „zamarzania"):**
W jednym terminalu leci `ping -c 100 $JET`, a w drugim równolegle odpal
`iperf3 -c $JET -R -t 30`. Patrz co robi ping PODCZAS transferu:
- ping skacze z ~60 ms na **500–3000 ms** → **bufferbloat potwierdzony**.
  To dokładnie to, co ścina Ci SSH, gdy leci wideo.

---

## Blok C — sygnał i tryb modemu SIM7600 (10 min) [hipoteza 3]

Znajdź port AT modemu (zwykle `/dev/ttyUSB2` lub `ttyUSB3`):
```bash
ls /dev/ttyUSB*
mmcli -L                     # jeśli jest ModemManager
```

Przez ModemManager (najprościej):
```bash
mmcli -m 0 --signal-setup=5
mmcli -m 0 --signal-get      # RSSI, RSRP, RSRQ, SNR
```
Albo AT-komendami (np. `sudo minicom -D /dev/ttyUSB2` lub `picocom`):
```
AT+CSQ        # RSSI: 0-31; >15 dobrze, <10 słabo, 99 = brak
AT+CPSI?      # tryb (LTE vs WCDMA/3G!), band, RSRP, RSRQ, SINR
AT+COPS?      # operator + typ dostępu
```
**Interpretacja LTE:**
- RSRP: > -90 dBm dobrze, -100..-110 słabo, < -110 kiepsko.
- RSRQ: > -10 dB dobrze, < -15 słabo.
- SINR: > 10 dB dobrze, < 3 dB szum > sygnał → tu leżą zrywy.
- **Jeśli `AT+CPSI?` pokazuje WCDMA/HSPA zamiast LTE** — modem zszedł na 3G,
  upload dramatycznie spada. Wymuś LTE (`AT+CNMP=38` = tylko LTE) i sprawdź.

**Fizyczne:** obie anteny wpięte (MAIN + AUX/diversity na SIM7600)? Zasilanie
modemu stabilne (SIM7600 przy nadawaniu ciągnie skoki prądu — słaby 5 V =
resety)? To najczęstsze przyczyny słabego uplinku.

---

## Blok D — czy jesteś za CGNAT? (3 min) [wyjaśnia hipotezę 2]

Na Jetsonie:
```bash
ip -4 addr show             # adres na interfejsie modemu (wwan0/usb0/ppp0)
curl -s ifconfig.me; echo   # publiczny adres widziany ze świata
```
- Adres na interfejsie w zakresie **100.64.0.0/10** (albo 10.x) i inny niż
  publiczny z `ifconfig.me` → **jesteś za CGNAT operatora**. Wtedy Tailscale
  direct po LTE jest bardzo trudny → zostaje DERP (hipoteza 2 wyjaśniona) i
  pragmatyczne obejścia z Bloku E.

---

## Blok E — obejścia do wdrożenia (rób nawet bez pełnej diagnozy)

Uszeregowane od „największy zysk najmniejszym kosztem":

### E1. mosh zamiast ssh — NAJWIĘKSZY zysk na terminalach
`mosh` jest zaprojektowany pod łącza z lagiem i stratą: przeżywa zamarznięcia,
lokalne echo pisania, wznawia się po zerwaniu. Terminale przestaną „umierać".
```bash
# raz, po obu stronach:
sudo apt install mosh
# łączenie (zamiast jet):
mosh jetsonknr@$JET
```
Alias do `~/.bashrc` na laptopie:
```bash
alias jetm='mosh jetsonknr@100.84.102.43'
```

### E2. Kolejkowanie na łączu (cake/fq_codel) — leczy bufferbloat z Bloku B
Priorytetyzuje ruch interaktywny (SSH) nad masowym (wideo), więc terminale nie
zamarzają, gdy leci podgląd. Na Jetsonie, na interfejsie modemu (`IFACE=wwan0`):
```bash
sudo tc qdisc replace dev $IFACE root cake bandwidth 4Mbit
# jeśli brak 'cake':
sudo tc qdisc replace dev $IFACE root fq_codel
```
`bandwidth` ustaw ~90% zmierzonego uplinku z Bloku B. Weryfikacja: powtórz test
bufferbloatu — ping pod obciążeniem powinien zostać niski.

### E3. Mniej danych z wideo — odciąża uplink u źródła
Podgląd jest najcięższy. Opcje (bez zmian w kodzie, przez parametry launcha):
```bash
# rzadsze klatki + niższa jakość JPEG:
ros2 launch drone_bringup suas_bringup.launch.py debug_jpeg_quality:=10
# w suas_detect_jetson jest też debug_every_n (publikuj co N-tą klatkę):
ros2 launch drone_bringup suas_detect_jetson.launch.py debug_every_n:=3 debug_jpeg_quality:=10
```
Zasada: podgląd włączaj TYLKO gdy patrzysz (marker_web i tak subskrybuje na
żądanie). Do oznaczania celu nie trzeba płynnego 12 FPS — wystarczy klatka.
Nie oglądaj web_video_server (:8080) i marker_web (:5000) jednocześnie.

### E4. Tailscale — spróbuj wymusić direct
Jeśli Blok D pokazał, że jednak NIE za CGNAT:
```bash
sudo tailscale up --reset          # świeże ustanowienie ścieżek
tailscale netcheck                 # co blokuje: UDP? porty?
```
Upewnij się, że UDP 41641 nie jest zablokowany. Za CGNAT direct raczej nie
wyjdzie — wtedy zostają E1–E3.

### E5. Drobne
- MTU: na łączu LTE czasem pomaga obniżyć MTU interfejsu WireGuard/modemu
  (test: `ping -M do -s 1400 $JET` — jeśli duże pakiety giną, zejdź z MTU).
- Zasilanie modemu i anteny (patrz Blok C) — bez tego reszta nie pomoże.

---

## Tabela decyzyjna

| Co zobaczysz | Wniosek | Zrób |
|---|---|---|
| `tailscale ping` → DERP + Blok D = CGNAT | direct niemożliwy | E1 + E2 + E3 |
| ping pod obciążeniem skacze do >1 s | bufferbloat | **E2** (cake), potem E3 |
| iperf uplink < 3 Mbit/s | za mały upload na wideo | **E3** (tnij wideo), E1 na terminale |
| `AT+CPSI?` = 3G / SINR < 3 dB | słaby radiowy | antena/band/pozycja, `AT+CNMP=38` |
| terminale giną, wideo działa | interaktywny głodzony | **E1 (mosh)** + E2 |

---

## Jeśli masz mało czasu — 3 ruchy, które zwykle wystarczą

1. **mosh** zamiast ssh (E1) — terminale przestają zamarzać.
2. **cake** na interfejsie modemu (E2) — leczy „wszystko naraz".
3. **Tnij wideo** (E3) — `debug_every_n:=3 debug_jpeg_quality:=10`, podgląd tylko
   gdy patrzysz.

Diagnostykę (Bloki A–D) rób, żeby wiedzieć KTÓRA przyczyna dominuje, ale E1–E3
możesz wdrożyć od razu — nie zaszkodzą.
