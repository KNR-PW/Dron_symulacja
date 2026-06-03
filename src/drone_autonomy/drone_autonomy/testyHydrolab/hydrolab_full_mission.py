#!/usr/bin/env python3
import argparse
import json
import os
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'drone_web')))

from drone_comunication.drone_controller import DroneController
from Hydrolab import PoolDetector, center_over_pool

from drone_web.ros_report_website import RosReportWebsite
from drone_web.ros_mission_website import RosMissionWebsite

from drone_interfaces.srv import CreateReport, UpdateReport, PostLog


ALT = 10.0 #wysokość na której latamy
SAMPLE_ALT = 3.0 # docelowa wysokosc do pobierania probki
PUMP_DURATION = 8.0 #czas pobierania
SCAN_DURATION = 5.0 
SAMPLE_VOLUME = 500 #no chyba to jest niepotrzebne jednak
ESP_UDP_PORT = 5005       # port, na ktory ESP32 rozglasza pomiary wody (UDP broadcast)
SENSOR_DURATION = 5.0 


def _parse_point(text: str):
    lat, lon = text.split(",")
    return float(lat), float(lon)


def parse_args(argv):
    p = argparse.ArgumentParser(prog="hydrolab_full_mission")
    p.add_argument("--p1", required=True, help="lat,lon basenu 1 ")
    p.add_argument("--p2", required=True, help="lat,lon basenu 2 ")
    p.add_argument("--p3", required=True, help="lat,lon basenu 3 ")
    p.add_argument("--p4", required=True, help="lat,lon basenu 4")
    p.add_argument("--alt", type=float, default=ALT)
    p.add_argument("--alt-sample", type=float, default=SAMPLE_ALT)
    p.add_argument("--pump-duration", type=float, default=PUMP_DURATION)
    args, _ = p.parse_known_args(argv)
    return args


class WebReportClient(Node):
    def __init__(self):
        super().__init__("hydrolab_web_client")
        self._create_report_cli = self.create_client(CreateReport, "create_report")
        self._update_report_cli = self.create_client(UpdateReport, "update_report")
        self._post_log_cli = self.create_client(PostLog, "post_log_to_web")

    def _call(self, client, request, timeout=15.0):

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"Serwis {client.srv_name} niedostepny")
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().warn(f"Serwis {client.srv_name}: timeout")
            return None
        return future.result()

    def create_report(self, metadata: dict) -> bool:
        req = CreateReport.Request()
        req.json_report = json.dumps(metadata)
        res = self._call(self._create_report_cli, req)
        if res is None:
            return False
        if res.success:
            self.get_logger().info(f"create_report OK: {res.message}")
        else:
            self.get_logger().warn(f"create_report blad: {res.message}")
        return bool(res.success)

    def upload_photo(self, pool_id: int, path: str, tag: str = "") -> bool:
        if not path or not os.path.exists(path):
            self.get_logger().warn(f"upload_photo: brak pliku {path}")
            return False
        req = UpdateReport.Request()
        req.json_update = json.dumps({"pool_id": pool_id, "tag": tag})
        req.images_list = [path]
        res = self._call(self._update_report_cli, req)
        if res is None:
            return False
        if res.success:
            self.get_logger().info(f"upload_photo pool={pool_id} tag={tag} OK")
        else:
            self.get_logger().warn(f"upload_photo pool={pool_id} blad: {res.message}")
        return bool(res.success)

    def upload_measurement(self, pool_id: int, ph: float, ec: float,
                           temperature: float, ml: int) -> bool:
        payload = {
            "pool_id": pool_id,
            "ph": ph,
            "conductivity": ec,
            "temperature": temperature,
            "ml": ml,
        }
        req = UpdateReport.Request()
        req.json_update = json.dumps(payload)
        req.images_list = []
        res = self._call(self._update_report_cli, req)
        if res is None:
            return False
        if res.success:
            self.get_logger().info(f"upload_measurement pool={pool_id} OK: {payload}")
        else:
            self.get_logger().warn(f"upload_measurement pool={pool_id} blad: {res.message}")
        return bool(res.success)

    def log(self, message: str, level: str = "info") -> bool:
        req = PostLog.Request()
        req.message = message
        req.level = level
        res = self._call(self._post_log_cli, req, timeout=5.0)
        return res is not None and res.result == 1


class WaterSensorReceiver:
    """Odbiera UDP broadcast z ESP32 z czujnikami wody i trzyma ostatni odczyt.
    """

    def __init__(self, port=ESP_UDP_PORT, logger=None):
        self.port = port
        self.logger = logger
        self._lock = threading.Lock()
        self._latest = None    # (ph, ec, temp)
        self._stamp = 0.0      # czas odbioru (monotonic)
        self._running = False
        self._sock = None
        self._thread = None

    def _log(self, level, msg):
        if self.logger is not None:
            getattr(self.logger, level)(msg)

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("", self.port))  
        self._sock.settimeout(1.0)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._log("info", f"WaterSensorReceiver: nasluch UDP na :{self.port}")

    def _loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
                ph = float(msg["ph"])
                ec = float(msg["cond"])
                temp = float(msg["temperatura"])
            except (ValueError, KeyError, json.JSONDecodeError):
                self._log("warn", "WaterSensorReceiver: zly pakiet, pomijam")
                continue
            with self._lock:
                self._latest = (ph, ec, temp)
                self._stamp = time.monotonic()

    def get_latest(self, max_age=SENSOR_DURATION):
        with self._lock:
            if self._latest is None:
                return None
            if max_age is not None and (time.monotonic() - self._stamp) > max_age:
                return None
            return self._latest

    def stop(self):
        self._running = False
        if self._sock is not None:
            self._sock.close()


def read_water_measurement(sensors: "WaterSensorReceiver", timeout=3.0):
    """Zwraca (ph, ec, temp) z ostatniego pakietu ESP, albo None gdy brak swiezych danych."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        reading = sensors.get_latest()
        if reading is not None:
            return reading
        time.sleep(0.1)
    return None


def capture_and_save(detector: PoolDetector, tag: str):
    det = detector.get_live_offset()
    if det is None:
        detector.get_logger().warn(f"capture_and_save[{tag}]: brak klatki")
        return None
    _, _, frame, boxes = det
    return detector.save_annotated(frame, boxes, tag=tag)


def run_mission(mission: DroneController, detector: PoolDetector,
                uploader: WebReportClient, sensors: WaterSensorReceiver, args):
    altitude = args.alt
    sample = args.alt_sample
    descent = altitude - sample
    waypoints = [_parse_point(args.p1), _parse_point(args.p2),
                 _parse_point(args.p3), _parse_point(args.p4)]

    uploader.create_report({"mission": "hydrolab_mission", "pools": 4})
    uploader.log("Start misji Hydrolab")

    mission.arm()
    mission.takeoff(altitude)

    for idx, (lat, lon) in enumerate(waypoints[:3], start=1):
        if mission._alarm:
            mission.rtl()
            return
        mission.get_logger().info(f"Pool {idx}/4  -> {lat:.7f},{lon:.7f}")
        mission.send_goto_global(lat, lon, altitude)
        time.sleep(2.0)
        fname = capture_and_save(detector, tag=f"pool{idx}")
        if fname:
            uploader.upload_photo(idx, fname, f"pool{idx}")

    if mission._alarm:
        mission.rtl()
        return

    lat4, lon4 = waypoints[3]
    mission.get_logger().info(f"Pool 4/4 -> {lat4:.7f},{lon4:.7f}")
    mission.send_goto_global(lat4, lon4, altitude)
    time.sleep(3.0)

    if detector.scan(SCAN_DURATION):
        center_over_pool(mission, detector, tag="pool4")
    else:
        mission.get_logger().warn("Pool 4: nie wykryto.")

    fname = capture_and_save(detector, tag="sample")
    if fname:
        uploader.upload_photo(4, fname, "sample")

    if descent > 0.1:
        mission.get_logger().info(f"Zniżanie do {sample} m po wodę.")
        mission.send_goto_relative(0.0, 0.0, descent)
        time.sleep(3.0)

    if mission.is_motor_available:
        mission.get_logger().info(f"Pompa: {args.pump_duration} s.")
        mission.control_dc_motor(rotation=True, duration=args.pump_duration)
    else:
        mission.get_logger().warn("Pompa niedostępna - pomijam pobór.")

    reading = read_water_measurement(sensors)
    if reading is not None:
        ph, ec, temp = reading
        uploader.upload_measurement(4, ph, ec, temp, SAMPLE_VOLUME)
    else:
        mission.get_logger().warn("Brak danych z ESP.")
        uploader.log("Brak danych z czujnikow wody", level="warn")

    if descent > 0.1:
        mission.get_logger().info(f"Wzlot z powrotem na {altitude} m.")
        mission.send_goto_relative(0.0, 0.0, -descent)
        time.sleep(2.0)

    uploader.log("Koniec misji, RTL")
    mission.rtl()


def main():
    rclpy.init(args=sys.argv)
    args = parse_args(sys.argv[1:])

    mission = DroneController("hydrolab_full_mission")
    detector = PoolDetector(mission)
    uploader = WebReportClient()

    report_web = RosReportWebsite()
    mission_web = RosMissionWebsite()

    sensors = WaterSensorReceiver(logger=mission.get_logger())
    sensors.start()

    executor = MultiThreadedExecutor()
    executor.add_node(mission)
    executor.add_node(detector)
    executor.add_node(uploader)
    executor.add_node(report_web)
    executor.add_node(mission_web)

    t = threading.Thread(target=run_mission,
                         args=(mission, detector, uploader, sensors, args),
                         daemon=True)
    t.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        t.join(timeout=2.0)
        sensors.stop()
        mission.destroy_node()
        detector.destroy_node()
        uploader.destroy_node()
        report_web.destroy_node()
        mission_web.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
