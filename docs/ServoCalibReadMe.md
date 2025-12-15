## Jak włączyć kalibrowanie serwomechanizmu w test misji
# Co odpalić?
0. Kontener, mam nadzieje, że to oczywiste.
1. Odpal drone_handler_px4, mozna za pomoca skryptu build_and_run.sh
2. Odpal MicroXRCE Agenta za pomocą skryptu run_microxrce.sh
# Co dodać do test misji?
1. : 
class TestClient(DroneController):
    def __init__(self):
        super().__init__()
        self.cli = self.create_client(VtolServoCalib, 'knr_hardware/calib_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calib_servo service...')
    def set_servo(self, angle_3: float, angle_4: float):
        req = VtolServoCalib.Request()
        req.angle_3 = angle_3  
        req.angle_4 = angle_4  
        self.future = self.cli.call_async(req)
2. : 
mission.set_servo(45.0,45.0)  # W Main argumenty w kątach 
## Jak odpalić manualnie za pomoca wezwania serwisu w jednej linijce z terminalu?
1. ros2 service call /calib_servo drone_interfaces/srv/VtolServoCalib ""{values: [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]}"
### 1.0 to 90 stopni, 0.0 to 45 stopni, -1.0 to 0 stopni działa tylko dla serw 3 i 4 póki co jak ogarniemy HITL zrobię, żeby dla wszystkich 8 możliwych działało.
