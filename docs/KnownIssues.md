<<<<<<< HEAD


=======
# 📋 Lista znanych problemów z symulacją

Poniżej znajduje się lista znanych problemów oraz ich rozwiązań.

---

## ❗ Gazebo Sim nie uruchamia GUI — komunikat *"Gazebo world already loaded"*

**Przyczyna:**  
Gazebo nie zostało poprawnie zamknięte i proces nadal działa w tle.

**Rozwiązanie:**  
Zabij wszystkie procesy Gazebo:

```bash
sudo pkill -f "gz sim"
>>>>>>> origin/dev
