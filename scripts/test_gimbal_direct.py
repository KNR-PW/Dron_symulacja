#!/usr/bin/env python3
import subprocess
import sys

def set_pitch(rad):
    # Limit z SDF drona knr_tiltrotor: 0.0 do 1.05 rad
    rad = max(0.0, min(1.05, rad))
    print(f"Ustawiam CameraJoint na {rad:.2f} rad...")
    
    cmd = [
        "gz", "topic", 
        "-t", "/model/knr_tiltrotor_0/servo_7", 
        "-m", "gz.msgs.Double", 
        "-p", f"data: {rad}"
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

if __name__ == '__main__':
    print("--- INTERAKTYWNY TEST GIMBALA (Gazebo) ---")
    print("Zakres: 0.0 (prosto) do 1.05 (maks. w dół)")
    print("Wpisz 'q' aby wyjść.")
    print("------------------------------------------")
    
    while True:
        try:
            val = input("Podaj kąt w radianach (0.0 - 1.05): ").strip().lower()
            if val == 'q':
                break
            
            rad = float(val)
            set_pitch(rad)
            
        except ValueError:
            print("BŁĄD: Podaj liczbę lub 'q'!")
        except KeyboardInterrupt:
            print("\nKoniec.")
            break

