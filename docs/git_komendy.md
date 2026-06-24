# Git – ściąga z komend

Praktyczny zbiór komend do kopiowania. Nie opis wszystkiego, tylko to, co realnie się przydaje.

---

## Stan i podgląd

```bash
git status                      # co zmienione / staged / untracked
git status -s                   # skrócony widok
git diff                        # zmiany niezastage'owane
git diff --staged               # zmiany dodane do commita (git add)
git diff HEAD~1                 # różnica względem poprzedniego commita
git diff main...HEAD            # co dorzuciła moja gałąź względem main
git log --oneline -20           # ostatnie 20 commitów, zwięźle
git log --oneline --graph --all # drzewko wszystkich gałęzi
git log --stat                  # commity + lista zmienionych plików
git log --author="DieRust"      # commity konkretnego autora
git show <hash>                 # pełny diff jednego commita
git show <hash> --stat          # tylko lista plików w commicie
git blame <plik>                # kto i kiedy zmienił każdą linię
```

---

## Dodawanie i commit

```bash
git add <plik>                  # dodaj konkretny plik
git add .                       # dodaj wszystko z bieżącego katalogu
git add -p                      # interaktywnie wybieraj fragmenty (hunki)
git commit -m "tytul"           # commit z tytułem
git commit -m "tytul" -m "opis" # tytuł + dłuższy opis (drugi -m)
git commit -am "tytul"          # add (śledzonych) + commit w jednym
git commit --amend              # popraw OSTATNI commit (treść lub pliki)
git commit --amend --no-edit    # dorzuć zmiany do ostatniego commita, bez zmiany opisu
```

---

## Gałęzie (branch)

```bash
git branch                      # lista lokalnych gałęzi
git branch -a                   # lokalne + zdalne
git switch <galaz>              # przełącz na istniejącą
git switch -c <nowa>            # utwórz i przełącz
git switch -c <nowa> origin/main# nowa gałąź z origin/main
git checkout <galaz>            # starszy odpowiednik switch
git branch -d <galaz>           # usuń gałąź (bezpiecznie, jeśli zmerge'owana)
git branch -D <galaz>           # usuń na siłę (nawet niezmerge'owaną)
git branch -m <nowa_nazwa>      # zmień nazwę bieżącej gałęzi
```

---

## Synchronizacja ze zdalnym (fetch / pull / push)

```bash
git remote -v                   # lista zdalnych repo (origin, my_fork...)
git fetch origin                # pobierz zmiany BEZ scalania
git fetch --all                 # pobierz ze wszystkich remote
git pull                        # fetch + merge bieżącej gałęzi
git pull --rebase               # fetch + rebase (czystsza historia)

git push                        # wypchnij na skonfigurowany upstream
git push -u my_fork <galaz>     # pierwszy push + ustawienie śledzenia
git push my_fork <galaz>        # push na konkretny remote
git push --force-with-lease     # nadpisz zdalną gałąź bezpiecznie (po rebase/amend)
```

> `--force-with-lease` jest bezpieczniejsze niż `--force` – odmówi, jeśli ktoś
> w międzyczasie wypchnął coś, czego nie masz.

---

## Cofanie zmian (najczęstsze ratunki)

```bash
git restore <plik>              # wyrzuć niezapisane zmiany w pliku
git restore .                   # wyrzuć wszystkie niezapisane zmiany
git restore --staged <plik>     # cofnij 'git add' (zostaw zmiany w plikach)
git checkout -- <plik>          # starszy odpowiednik restore

git reset --soft HEAD~1         # cofnij commit, ZOSTAW zmiany staged
git reset HEAD~1                # cofnij commit, zmiany jako niestaged
git reset --hard HEAD~1         # cofnij commit i WYRZUĆ zmiany (uważaj!)
git revert <hash>               # nowy commit cofający wskazany (bezpieczne na wypchnięte)

git clean -n                    # POKAŻ, które nieśledzone pliki by usunął
git clean -fd                   # usuń nieśledzone pliki i katalogi
```

---

## Stash (schowek na zmiany)

```bash
git stash                       # schowaj zmiany, wróć do czystego stanu
git stash -u                    # schowaj też nieśledzone pliki
git stash push -m "opis"        # schowek z nazwą
git stash list                  # lista schowków
git stash show -p stash@{0}     # podgląd zawartości schowka
git stash apply                 # przywróć (zostaw na liście)
git stash pop                   # przywróć i usuń z listy
git stash drop stash@{0}        # usuń konkretny schowek
```

---

## Łączenie gałęzi (merge / rebase)

```bash
git merge <galaz>               # wlej <galaz> do bieżącej
git merge --no-ff <galaz>       # zawsze twórz commit scalający
git rebase main                 # przenieś commity bieżącej gałęzi na czubek main
git rebase --abort              # przerwij rebase przy konflikcie
git rebase --continue           # kontynuuj po rozwiązaniu konfliktu
git merge --abort               # przerwij merge przy konflikcie
```

---

## Submoduły (są w tym repo, np. KNR_Drone_PX4_Autopilot)

```bash
git submodule update --init --recursive   # pobierz/zainicjuj submoduły
git submodule status                       # stan submodułów
git submodule update --remote              # podciągnij submoduły do ich nowszych commitów
git clone --recurse-submodules <url>       # klon od razu z submodułami
```

---

## Tagi

```bash
git tag                         # lista tagów
git tag v1.0                    # lekki tag na bieżącym commicie
git tag -a v1.0 -m "opis"       # tag z opisem (annotated)
git push my_fork v1.0           # wypchnij tag
git push my_fork --tags         # wypchnij wszystkie tagi
```

---

## Ten projekt – typowy przepływ na my_fork

```bash
# nowa gałąź z aktualnego maina
git fetch origin
git switch -c moja_funkcja origin/main

# ...praca, edycje...
git add <pliki>
git commit -m "opis zmiany"

# wypchnięcie na swój fork
git push -u my_fork moja_funkcja

# aktualizacja gałęzi o najnowszy main
git fetch origin
git rebase origin/main          # albo: git merge origin/main
```

---

## Szybkie sztuczki

```bash
git log --oneline -- <plik>     # historia jednego pliku
git diff --name-only            # tylko nazwy zmienionych plików
git checkout <hash> -- <plik>   # przywróć plik z konkretnego commita
git reflog                      # historia HEAD (ratunek po 'zgubionym' commicie)
git config --global alias.st status   # własny skrót: 'git st'
```
