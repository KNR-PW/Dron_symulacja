# Servo Calibration Node

## Opis

Node `drone_handler_px4` z service'iem `preflight_calibration_control_service` umożliwia kalibrację serwomechanizmów drona poprzez wysyłanie poleceń kalibracji do Orange Cube (Flight Controller). On otrzymuje je a potem sam wyłącza kalibracje zgodnie z napisanym softem w firmwarze.

### Co robi?

- Wysyła wiadomości `PreflightCalibrationControl` na topic `/fmu/in/preflight_calibration_control`
- Obsługuje dwa działania:
  - **action=1** - START kalibracji serwomechanizmu
  - **action=0** - STOP kalibracji

### Wymagania

- Orange Cube podłączony do Raspberry Pi
- Serwomechanizm podłączony do PWM portu na Orange Cube


## Sposoby wywołania

### Sposób 1: Poprzez Python (test_mission.py)

Utwórz lub zmodyfikuj plik `vtol_servo_test.py`:

```python
import rclpy
import time
from drone_comunication import DroneController
from drone_interfaces.srv import PreflightCalibrationControlService

class TestClient(DroneController):
    def __init__(self):
        super().__init__()
        self.cli = self.create_client(PreflightCalibrationControlService, 'knr_hardware/preflight_calibration_control_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for preflight_calibration_control_service...')
    
    def start_calib(self, action: int):
        req = PreflightCalibrationControlService.Request()
        req.action = action
        self.future = self.cli.call_async(req)
        self.get_logger().info(f'Sent calibration request with action: {action}')

def main(args=None):
    rclpy.init(args=args)
    try:
        print("Creating TestClient...")
        mission = TestClient()
        print("TestClient created - service is ready!")
        
        print("Sending calibration START (action=1)...")
        mission.start_calib(1)  # Valid actions: 0=STOP, 1=START
        
        print("Waiting for response...")
        time.sleep(2)
        
        print("Done!")
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        mission.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
```

**Uruchomienie:**

```bash
# Terminal 1 - uruchom handler
source /home/knr/Dron_symulacja/install/setup.bash
ros2 run drone_hardware drone_handler_px4

# Terminal 2 - Uruchom MicroXRCE
W raspberce mamy alias "xrce" wpisujesz tylko to i ci powinno wyskoczyć

# Terminal 3 - uruchom launch file 
cd /home/knr/Dron_symulacja/src/drone_bringup/drone_bringup
vtol_preflight.launch.py

# Terminal 4 - uruchom test
cd /home/knr/Dron_symulacja/src/drone_autonomy/drone_autonomy
python3 vtol_servo_test.py
```

---

### Sposób 2: Ręcznie z terminalu

**Terminal 1 - uruchom handler:**
```bash
source /home/knr/Dron_symulacja/install/setup.bash
ros2 run drone_hardware drone_handler_px4
```
**Terminal 2 - microXRCE:**
```bash
xrce
```
**Terminal 3 - wyślij żądanie kalibracji:**

START kalibracji:
```bash
ros2 service call knr_hardware/preflight_calibration_control_service drone_interfaces/srv/PreflightCalibrationControlService "{action: 1}"
```

## Troubleshooting

- **Service not found**: Upewnij się, że `drone_handler_px4` jest uruchomiony
- **Battery status not ready**: Node czeka na dane z akumulatora - to normalne w symulacji
- **Servo doesn't move**: Sprawdź połączenie serwomechanizmu z Orange Cube na PWM porcie