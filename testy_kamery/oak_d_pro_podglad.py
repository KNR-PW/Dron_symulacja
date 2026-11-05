#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import depthai as dai
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO stream from OAK-D Pro (USB) with styled boxes")
    parser.add_argument(
        "--model",
        default="/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.pt",
        help="Model path (.pt or .engine)",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO input size")
    parser.add_argument("--cam_width", type=int, default=1280, help="Camera width")
    parser.add_argument("--cam_height", type=int, default=720, help="Camera height")
    parser.add_argument("--fps", type=float, default=20.0, help="Camera FPS")
    parser.add_argument("--device", default="cuda:0", help="Inference device: cpu or cuda:0")
    return parser.parse_args()


def get_color(cls_id: int) -> tuple[int, int, int]:
    return (
        int((37 * cls_id) % 255),
        int((17 * cls_id) % 255),
        int((29 * cls_id) % 255),
    )


def infer_with_fallback(model: YOLO, frame, args: argparse.Namespace, active_device: str):
    try:
        results = model(frame, imgsz=args.imgsz, conf=args.conf, device=active_device, verbose=False)
        return results, active_device
    except RuntimeError as exc:
        msg = str(exc).lower()
        cuda_failed = "cuda" in active_device and (
            "no kernel image" in msg or "unable to find an engine" in msg
        )
        if not cuda_failed:
            raise

        print("CUDA inference failed on this Torch build, switching to CPU.")
        results = model(frame, imgsz=args.imgsz, conf=args.conf, device="cpu", verbose=False)
        return results, "cpu"


def main() -> None:
    args = parse_args()
    model_path = os.path.expanduser(args.model)
    model = YOLO(model_path)

    current_device = args.device
    names = model.names
    last_time = time.time()
    fps = 0.0

    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setPreviewSize(args.cam_width, args.cam_height)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(args.fps)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("video")
    cam.preview.link(xout.input)

    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue("video", maxSize=4, blocking=False)
        print("YOLO stream from OAK-D Pro (USB) started. Press q to quit.")

        while True:
            in_frame = video_queue.get()
            frame = in_frame.getCvFrame()

            start = time.time()
            results, current_device = infer_with_fallback(model, frame, args, current_device)
            infer_time = time.time() - start
            infer_fps = 1.0 / infer_time if infer_time > 0 else 0.0

            annotated = frame.copy()
            r = results[0]

            if r.boxes is not None:
                boxes = r.boxes.xyxy.cpu().numpy()
                scores = r.boxes.conf.cpu().numpy()
                classes = r.boxes.cls.cpu().numpy().astype(int)

                for box, score, cls in zip(boxes, scores, classes):
                    if score < args.conf:
                        continue

                    x1, y1, x2, y2 = map(int, box)
                    color = get_color(cls)
                    label = f"{names[cls]} {score:.2f}"

                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(annotated, (x1, y1 - h - 6), (x1 + w, y1), color, -1)
                    cv2.putText(
                        annotated,
                        label,
                        (x1, y1 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                    )

            now = time.time()
            dt = now - last_time
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            last_time = now

            cv2.putText(
                annotated,
                f"PIPE FPS: {fps:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                annotated,
                f"INF FPS: {infer_fps:.1f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 0),
                2,
            )
            cv2.putText(
                annotated,
                f"DEVICE: {current_device}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 200, 255),
                2,
            )

            cv2.imshow("YOLO OAK-D Pro", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
