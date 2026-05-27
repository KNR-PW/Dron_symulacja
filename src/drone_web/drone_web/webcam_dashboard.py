import argparse
import asyncio
import fractions
import json
import logging
import os
import threading
import time

from aiohttp import web
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection,
    RTCSessionDescription,
)

import numpy as np
import cv2
import av

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from drone_interfaces.msg import Telemetry
from sensor_msgs.msg import Image

DASHBOARD_DIR = os.path.join(os.path.dirname(__file__), "dashboard")

pcs = set()
websockets = set()
status_lock = threading.Lock()
ros_node = None
camera_topic = '/oak/rgb/image_raw'

_static_cache = {}

_last_telemetry_json = None

status_data = {
    "altitude": 0.0,
    "speed": 0.0,
    "battery_percent": "100%",
    "battery_voltage": "0V",
    "gps_global": "0.0,0.0",
    "gps_relative": "0.0,0.0",
    "mission_time": "--",
    "flight_mode": "INIT",
    "water_temp": "--",
    "last_update": "-",
}

_no_signal_cached = None


def _get_no_signal_frame():
    global _no_signal_cached
    if _no_signal_cached is None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(img, "No camera signal", (130, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        _no_signal_cached = av.VideoFrame.from_ndarray(img, format='bgr24')
    return _no_signal_cached


class StatusNode(Node):
    def __init__(self):
        super().__init__('dashboard_status_node')
        self.create_subscription(
            Telemetry, 'knr_hardware/telemetry',
            self.telemetry_callback, 10
        )
        self.get_logger().info("StatusNode: subscribed to knr_hardware/telemetry")

    def telemetry_callback(self, msg):
        with status_lock:
            status_data["altitude"] = round(msg.alt, 1)
            status_data["speed"] = round(msg.speed, 1)
            status_data["battery_percent"] = f"{msg.battery_percentage}%"
            status_data["battery_voltage"] = f"{msg.battery_voltage:.2f}V"
            status_data["gps_global"] = f"{round(msg.global_lat, 6)},{round(msg.global_lon, 6)}"
            status_data["gps_relative"] = f"{round(msg.lat, 6)},{round(msg.lon, 6)}"
            status_data["flight_mode"] = msg.flight_mode
            status_data["last_update"] = time.strftime("%Y-%m-%d %H:%M:%S")


class ROSVideoStreamTrack(MediaStreamTrack):
    """Bridge between ROS 2 sensor_msgs/Image and WebRTC aiortc video stream."""
    kind = "video"

    _shared_frame = None
    _shared_lock = threading.Lock()
    _subscription = None

    @classmethod
    def setup_subscription(cls, node, topic):
        """Single shared subscription for all WebRTC tracks."""
        if cls._subscription is not None:
            return
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        cls._subscription = node.create_subscription(
            Image, topic, cls._on_image, qos_profile
        )
        logging.info(f"ROSVideoStreamTrack: subscribed to {topic}")

    @classmethod
    def _on_image(cls, msg):
        try:
            channels = 3 if ('8' in msg.encoding and 'mono' not in msg.encoding) else 1
            if channels == 3:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
                fmt = 'bgr24' if msg.encoding == 'bgr8' else 'rgb24'
            else:
                img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width))
                fmt = 'gray'
            frame = av.VideoFrame.from_ndarray(img_array, format=fmt)
            with cls._shared_lock:
                cls._shared_frame = frame
        except Exception as e:
            logging.error(f"Image conversion error: {e}")

    def __init__(self):
        super().__init__()
        self._start_time = None
        self._time_base = fractions.Fraction(1, 90000)
        self._target_interval = 1.0 / 30.0
        self._last_good_frame = None

    async def recv(self):
        if self._start_time is None:
            self._start_time = time.time()

        await asyncio.sleep(self._target_interval)

        with self._shared_lock:
            if self._shared_frame is not None:
                self._last_good_frame = self._shared_frame

        frame = self._last_good_frame if self._last_good_frame is not None else _get_no_signal_frame()

        elapsed = time.time() - self._start_time
        frame.pts = int(elapsed * 90000)
        frame.time_base = self._time_base
        return frame


def ros_spin():
    global ros_node
    rclpy.init()
    ros_node = StatusNode()
    try:
        rclpy.spin(ros_node)
    finally:
        if ros_node:
            ros_node.destroy_node()
        rclpy.shutdown()


def _load_static(filename):
    if filename not in _static_cache:
        with open(os.path.join(DASHBOARD_DIR, filename), "r") as f:
            _static_cache[filename] = f.read()
    return _static_cache[filename]


async def index(request: web.Request) -> web.Response:
    return web.Response(content_type="text/html", text=_load_static("index.html"))


async def stylesheet(request: web.Request) -> web.Response:
    return web.Response(content_type="text/css", text=_load_static("style.css"))


async def javascript(request: web.Request) -> web.Response:
    return web.Response(content_type="application/javascript", text=_load_static("client.js"))


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    websockets.add(ws)
    try:
        async for msg in ws:
            pass
    finally:
        websockets.discard(ws)
    return ws


async def telemetry_broadcast_task(app):
    global _last_telemetry_json
    try:
        while True:
            if websockets:
                with status_lock:
                    data = json.dumps(status_data)
                if data != _last_telemetry_json:
                    _last_telemetry_json = data
                    for ws in list(websockets):
                        try:
                            await ws.send_str(data)
                        except Exception:
                            websockets.discard(ws)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass


async def offer(request: web.Request) -> web.Response:
    global ros_node
    if ros_node is None:
        return web.Response(status=503, text="ROS node not ready")

    ROSVideoStreamTrack.setup_subscription(ros_node, camera_topic)

    params = await request.json()
    offer_desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    from aiortc import RTCConfiguration, RTCIceServer
    config = RTCConfiguration(iceServers=[
        RTCIceServer(urls=["stun:stun.l.google.com:19302"])
    ])
    pc = RTCPeerConnection(configuration=config)
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange() -> None:
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    video = ROSVideoStreamTrack()
    pc.addTrack(video)

    await pc.setRemoteDescription(offer_desc)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
        }),
    )


async def on_startup(app: web.Application):
    app['telemetry_task'] = asyncio.create_task(telemetry_broadcast_task(app))


async def on_shutdown(app: web.Application) -> None:
    app['telemetry_task'].cancel()
    await app['telemetry_task']
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()

    for ws in list(websockets):
        await ws.close()
    websockets.clear()


def main(args=None):
    global camera_topic

    parser = argparse.ArgumentParser(description="WebRTC Dashboard for drone telemetry")
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--camera-topic', default='/oak/rgb/image_raw',
                        help='ROS 2 image topic for video stream')
    parsed, _ = parser.parse_known_args()

    camera_topic = parsed.camera_topic

    logging.basicConfig(level=logging.INFO)
    logging.info(f"Dashboard starting: port={parsed.port}, camera_topic={camera_topic}")

    t = threading.Thread(target=ros_spin, daemon=True)
    t.start()
    time.sleep(1.0)

    app = web.Application()
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/style.css", stylesheet)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)
    app.router.add_get("/ws", websocket_handler)
    web.run_app(app, host=parsed.host, port=parsed.port)


if __name__ == "__main__":
    main()
