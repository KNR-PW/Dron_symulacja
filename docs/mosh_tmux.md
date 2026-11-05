# mosh + tmux — łączenie z Jetsonem przez LTE

Host: `jetsonknr@100.84.102.43` (Tailscale).

## Logowanie bez hasła (raz)

```bash
ssh-keygen -t ed25519            # jeśli nie masz jeszcze klucza
ssh-copy-id jetsonknr@100.84.102.43
```
Alias na laptopie — dopisz do `~/.bashrc`, potem `source ~/.bashrc`:
```bash
alias jet='mosh jetsonknr@100.84.102.43'
```

## mosh

```bash
sudo apt install -y mosh         # na laptopie I na Jetsonie
jet                              # łączenie (albo: mosh jetsonknr@100.84.102.43)
```

## tmux (na Jetsonie)

```bash
sudo apt install -y tmux
tmux new -s dron                 # start
tmux attach -t dron              # powrót po rozłączeniu
```
Skróty (prefix = `Ctrl-a`):

| Skrót | Efekt |
|---|---|
| `Ctrl-a v` | podział obok siebie |
| `Ctrl-a -` | podział pod spodem |
| `Alt+strzałki` | skok między panelami |
| kółko myszy | przewijanie historii |
| `Ctrl-a d` | odepnij (procesy lecą dalej) |
| `Ctrl-a x` | zamknij panel |

## Po co to (zamiast zwykłego ssh)

- **mosh** — przeżywa lag i zrywy LTE, pisanie natychmiastowe.
- **tmux** — historia, panele i procesy żyją, nawet gdy łącze padnie.
