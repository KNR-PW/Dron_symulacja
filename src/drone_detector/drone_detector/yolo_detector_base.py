#!/usr/bin/env python3
"""Klasa bazowa dla detektorow YOLO (wspolna logika SIM i JETSON)."""

import time
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from ultralytics import YOLO

from drone_interfaces.msg import TentDetection, TentDetections

# ── 1. STALE ────────────────────────────────────────────────────────
DEBUG_MAX_WIDTH = 960   # szerokosc obrazu debug (px); mniejszy = tanszy encode/transfer

# Model jest dwuklasowy i taki zostanie, wiec mapowanie trzymamy na sztywno —
# parametr z wlasnym mini-jezykiem bylby przerostem formy nad trescia.
# Nazwy sluza tylko do logow i etykiet; zrodlem prawdy sa indeksy modelu,
# ktore node wypisuje przy starcie.
CLASS_TOPICS = {
    0: "/tent_detections",      # namiot
    1: "/people_detections",    # czlowiek
}
CLASS_NAMES = {0: "namiot", 1: "czlowiek"}
CLASS_COLORS = {0: (0, 255, 0), 1: (0, 165, 255)}   # BGR: zielony, pomaranczowy
UNKNOWN_COLOR = (160, 160, 160)


def _empty_detection() -> TentDetection:
    """Pusta detekcja — publikowana, gdy w klatce nie ma obiektu danej klasy.

    Publikujemy ja KAZDA klatke, takze bez trafienia: odbiorcy (okno M z N
    w suas_flight_controller, timeouty gimbala) licza klatki, nie sekundy.
    """
    det = TentDetection()
    det.detected = False
    det.bounding_box = [0.0, 0.0, 0.0, 0.0]
    det.confidence = 0.0
    det.track_id = -1
    det.class_id = -1
    return det


class YoloDetectorBase(Node):
    """Wspolna logika: subskrypcja kamery, publikacja detekcji i podglad debug."""

    def __init__(self, node_name: str, default_model: str, track_kwargs: dict = None):
        super().__init__(node_name)

        # ── 2. PARAMETRY ROS ────────────────────────────────────────
        self.declare_parameter("camera_topic", "/rgb_camera/image")
        self.declare_parameter("model_path", default_model)
        self.declare_parameter("conf", 0.5)
        # Osobny prog pewnosci dla CZLOWIEKA (klasa 1). Czlowiek z pulapu jest
        # maly i slaby w detekcji, wiec zwykle chce sie go lapac NIZSZYM progiem
        # niz namiot. <0 = brak osobnego progu (uzyj 'conf' dla obu klas).
        self.declare_parameter("conf_person", -1.0)
        self.declare_parameter("input_size", 1024)
        self.declare_parameter("debug_every_n", 1)      # 1 = podglad z kazdej klatki
        self.declare_parameter("debug_jpeg_quality", 20)
        # Filtr klas modelu, np. "0" = tylko namioty, "1" = tylko ludzie.
        # DOMYSLNIE PUSTY = obie klasy naraz. Filtr nie jest juz potrzebny do
        # rozdzielenia klas (kazda ma wlasny topic) — zostaje na wypadek, gdyby
        # ktoras klasa generowala smieci i chcialo sie ja wylaczyc.
        self.declare_parameter("classes", "")

        model_path = self.get_parameter("model_path").value
        self.conf = self.get_parameter("conf").value
        cp = self.get_parameter("conf_person").value
        # <0 = uzyj wspolnego 'conf'. Detekcje odpalamy na NIZSZYM z progow,
        # a nadmiar odsiewamy per klasa (ultralytics nie ma progu per-klasa).
        self.conf_person = cp if cp >= 0 else self.conf
        self.input_size = self.get_parameter("input_size").value
        cam_topic = self.get_parameter("camera_topic").value
        self.debug_every_n = max(1, self.get_parameter("debug_every_n").value)
        self.jpeg_quality = self.get_parameter("debug_jpeg_quality").value
        cls_str = self.get_parameter("classes").value.strip()
        # None = bez filtra; ultralytics oczekuje listy indeksow albo None
        self.classes = [int(c) for c in cls_str.split(",") if c.strip()] or None

        # ── 3. MODEL YOLO ───────────────────────────────────────────
        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(model_path, task="detect")
        self.br = CvBridge()
        self._track_kwargs = track_kwargs or {}

        # ── 4. PUB / SUB ────────────────────────────────────────────
        # /detections — komplet boxow z klatki, obie klasy, z naglowkiem czasu.
        # To jest zrodlo prawdy dla geolokalizacji.
        self.pub_all = self.create_publisher(TentDetections, "/detections", 10)
        # Topici per klasa — najpewniejszy box DANEJ klasy, kazda klatke.
        # Zachowuja semantyke sprzed rozdzielenia klas, wiec suas_flight_controller
        # i suas_gimbal_controller dzialaja bez zmian.
        self.class_pubs = {
            cls: self.create_publisher(TentDetection, topic, 10)
            for cls, topic in CLASS_TOPICS.items()
        }
        self.img_pub = self.create_publisher(Image, "/tent_detections/image", 1)
        # JPEG robimy tu, w tym samym watku. Wysylanie raw bgr8 960x720 (2.1 MB/klatke)
        # do osobnego node'a republish kosztowalo wiecej CPU niz sam encode.
        self.img_pub_c = self.create_publisher(
            CompressedImage, "/tent_detections/image/compressed", 1)
        cam_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, cam_topic, self._on_image, cam_qos)

        # ── 5. LICZNIKI ─────────────────────────────────────────────
        self._frames = 0
        self._hits = {cls: 0 for cls in CLASS_TOPICS}   # klatki z trafieniem, per klasa
        self._dbg_frames = 0
        self._t0 = time.monotonic()

        # Wypisujemy mapowanie indeks -> nazwa, zeby dalo sie zweryfikowac, ze
        # numery klas modelu zgadzaja sie z CLASS_TOPICS.
        self.get_logger().info(f"Klasy modelu: {self.model.names}")
        topics = "  ".join(f"{c}->{t}" for c, t in CLASS_TOPICS.items())
        self.get_logger().info(
            f"{node_name} ready  |  input_size={self.input_size} "
            f"debug_every_n={self.debug_every_n} "
            f"classes={self.classes if self.classes is not None else 'wszystkie'} "
            f"| /detections + {topics}")
        self.get_logger().info(
            f"Prog pewnosci: namiot={self.conf}  czlowiek={self.conf_person}"
            + ("" if self.conf_person != self.conf else " (wspolny)"))

    # ────────────────────────────────────────────────────────────────
    def _on_image(self, msg: Image):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.track(
            frame,
            conf=min(self.conf, self.conf_person),
            persist=True,
            verbose=False,
            imgsz=self.input_size,
            classes=self.classes,
            **self._track_kwargs,
        )

        # ── 6. WSZYSTKIE BOXY Z KLATKI ──────────────────────────────
        dets = TentDetections()
        # stamp klatki kamery, nie moment publikacji — odbiorca moze siegnac po
        # telemetrie z chwili, w ktorej klatka powstala
        dets.header = msg.header
        dets.img_h, dets.img_w = frame.shape[0], frame.shape[1]

        # Najpewniejszy box W OBREBIE KLASY. Bez tego namiot z 50 m zawsze
        # wygrywalby z czlowiekiem, bo jest wiekszy i latwiejszy do wykrycia.
        best = {}
        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                box = boxes[i]
                x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
                det = TentDetection()
                det.detected = True
                det.bounding_box = [x1, y1, x2 - x1, y2 - y1]
                det.confidence = float(box.conf[0])
                # ID sciezki z trackera; None dopoki tracker nie potwierdzi sciezki.
                det.track_id = int(box.id[0]) if box.id is not None else -1
                det.class_id = int(box.cls[0])
                # Prog per klasa: czlowiek (1) ma wlasny conf_person, reszta 'conf'.
                # Detekcje odpalilismy na nizszym progu, wiec tu odsiewamy nadmiar.
                thr = self.conf_person if det.class_id == 1 else self.conf
                if det.confidence < thr:
                    continue
                dets.detections.append(det)
                prev = best.get(det.class_id)
                if prev is None or det.confidence > prev.confidence:
                    best[det.class_id] = det

        self.pub_all.publish(dets)

        # ── 7. NAJPEWNIEJSZY BOX PER KLASA (kazda klatke, takze pusty) ──
        for cls, pub in self.class_pubs.items():
            pub.publish(best.get(cls, _empty_detection()))

        # ── 8. PODGLAD DEBUG (pomniejszony, co N-ta klatka) ─────────
        self._dbg_frames += 1
        want_raw = self.img_pub.get_subscription_count() > 0
        want_c = self.img_pub_c.get_subscription_count() > 0
        if (want_raw or want_c) and self._dbg_frames % self.debug_every_n == 0:
            dbg = frame
            for det in dets.detections:
                x, y, w, h = det.bounding_box
                color = CLASS_COLORS.get(det.class_id, UNKNOWN_COLOR)
                cv2.rectangle(dbg, (int(x), int(y)), (int(x + w), int(y + h)), color, 2)
                name = CLASS_NAMES.get(det.class_id, str(det.class_id))
                label = f"{name} {det.confidence:.2f}"
                if det.track_id >= 0:
                    label += f" #{det.track_id}"
                cv2.putText(dbg, label, (int(x), int(y) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            if dbg.shape[1] > DEBUG_MAX_WIDTH:
                scale = DEBUG_MAX_WIDTH / dbg.shape[1]
                dbg = cv2.resize(dbg, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

            if want_c:
                ok, buf = cv2.imencode(
                    ".jpg", dbg, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header = msg.header
                    cmsg.format = "jpeg"
                    cmsg.data = buf.tobytes()
                    self.img_pub_c.publish(cmsg)
            if want_raw:
                raw = self.br.cv2_to_imgmsg(dbg, encoding="bgr8")
                raw.header = msg.header
                self.img_pub.publish(raw)

        # ── 9. FPS ──────────────────────────────────────────────────
        self._frames += 1
        for cls in self._hits:
            if cls in best:
                self._hits[cls] += 1

        dt = time.monotonic() - self._t0
        if dt >= 2.0:
            fps = self._frames / dt
            per_class = "  ".join(
                f"{CLASS_NAMES.get(c, c)}: {n}/{self._frames}"
                for c, n in self._hits.items())
            self.get_logger().info(
                f"FPS: {fps:.1f}  ({1000/fps:.0f} ms/frame) | {per_class}")
            self._frames = 0
            self._hits = {cls: 0 for cls in CLASS_TOPICS}
            self._t0 = time.monotonic()
