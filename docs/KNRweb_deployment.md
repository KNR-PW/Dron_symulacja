# Wdrożenie aplikacji na hosting Render
### Potrzebne są:
1. konto Render
2. link do repozytorium strony na githubie
3. requirements.txt dla projektu - musi być sensowny

### Na hostingu (bardzo intuicyjne)
1. Zaloguj się
2. Kliknij new +
3. Wybierz **web service**
4. Podajesz link do publicznego repozytorium
5. Wybierz region *ja dałem EU East*, branch, Root directory zostaw puste
6. Runtime ustaw na **Python 3**
7. ``` pip install -r requirements.txt``` - powinno być domyślnym build command
8. ```gunicorn app:app``` - defaultowy start command ale jak główny plik to np ```manin.py``` a instancja to np server: wtedy powinno być ```gunicorn main:server```
9.  Plan prawdopodobnie Free
10.  Klikasz "Create Web Service"
    #### śledzenie postępu procesu
      problemy:
     -  Build fails zazwyczaj błąd w ```requirements.txt```
     -  Deploy Fails/ Application crashing najczęściej błąd w start command:
         1. gunicorn musi być w ```requirements.txt```
         2. ewentualnie błędy w kodzie Flaska
11. Jeżeli build i deploy przeszły wejdź w link i zobacz stronę
12. Na darmowym pakiecie najpierw pokazuje reklama Rendera - trzeba poczekać 
