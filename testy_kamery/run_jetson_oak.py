#!/usr/bin/env python3
import argparse
import os
import time

import cv2
import depthai as dai
import torch
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO stream from OAK-D Pro (USB) with styled boxes")
    parser.add_argument(
        "--model",
        default="/home/jetsonknr/Dron_symulacja/testy_kamery/yolo26s.engine",
        help="Model path (.pt or .engine)",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO input size")
    parser.add_argument("--cam_width", type=int, default=1920, help="Camera width")
    parser.add_argument("--cam_height", type=int, default=1080, help="Camera height")
    parser.add_argument("--fps", type=float, default=30.0, help="Camera FPS")
    parser.add_argument("--device", default="cuda:0", help="Inference device: cpu or cuda:0")
    return parser.parse_args()


def get_color(cls_id: int) -> tuple[int, int, int]:
    return (
        int((37 * cls_id) % 255),
        int((17 * cls_id) % 255),
        int((29 * cls_id) % 255),
    )


def setup_cuda() -> bool:
    """Configure cuDNN for Jetson Orin and verify CUDA works."""
    if not torch.cuda.is_available():
        print("CUDA not available.")
        return False

    capability = torch.cuda.get_device_capability(0)
    required_arch = f"sm_{capability[0]}{capability[1]}"
    arch_list = torch.cuda.get_arch_list()
    if required_arch not in arch_list:
        print(
            f"Incompatible Torch CUDA build: GPU needs {required_arch}, "
            f"but wheel has {arch_list}."
        )
        return False

    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.backends.cuda.matmul.allow_tf32 = True

    try:
        dummy = torch.randn(1, 3, 64, 64, device="cuda:0")
        conv = torch.nn.Conv2d(3, 8, 3, padding=1).to("cuda:0")
        _ = conv(dummy)
        torch.cuda.synchronize()
        print(f"CUDA OK: {torch.cuda.get_device_name(0)}")
        return True
    except Exception as e:
        print(f"CUDA sanity check failed: {e}")
        return False


def warmup_model(model: YOLO, device: str, imgsz: int) -> bool:
    """Run a few dummy inferences so cuDNN builds its engine cache."""
    print(f"Warming up model on {device} (imgsz={imgsz})...")
    try:
        import numpy as np
        dummy_frame = np.zeros((imgsz, imgsz, 3), dtype="uint8")
        for i in range(3):
            model(dummy_frame, imgsz=imgsz, conf=0.25, device=device, verbose=False)
        torch.cuda.synchronize()
        print("Warmup done.")
        return True
    except Exception as e:
        print(f"Warmup failed: {e}")
        return False


def infer_strict(model: YOLO, frame, args: argparse.Namespace, device: str):
    return model(frame, imgsz=args.imgsz, conf=args.conf, device=device, verbose=False)


def build_pipeline(args: argparse.Namespace) -> dai.Pipeline:
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setFps(args.fps)
    cam.setVideoSize(args.cam_width, args.cam_height)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.video.link(xout.input)
    return pipeline


def connect_with_retry(pipeline: dai.Pipeline, retries: int = 20, delay_sec: float = 1.0) -> dai.Device:
    last_exc = None
    for attempt in range(1, retries + 1):
        try:
            return dai.Device(pipeline)
        except RuntimeError as exc:
            last_exc = exc
            if attempt < retries:
                print(f"Waiting for OAK-D Pro USB device ({attempt}/{retries})...")
                time.sleep(delay_sec)

    assert last_exc is not None
    raise last_exc


def main() -> None:
    args = parse_args()

    cuda_ok = setup_cuda()
    if "cuda" in args.device and not cuda_ok:
        raise RuntimeError(
            "CUDA preflight failed. Torch/CUDA stack is not usable for this model on GPU. "
            "Stopping without CPU fallback as requested."
        )

    model_path = os.path.expanduser(args.model)
    model = YOLO(model_path)

    current_device = args.device

    if "cuda" in current_device:
        warmup_ok = warmup_model(model, current_device, args.imgsz)
        if not warmup_ok:
            raise RuntimeError(
                "Model warmup on CUDA failed. Stopping without CPU fallback as requested."
            )

    names = model.names
    last_time = time.time()
    fps = 0.0

    frame_count = 0
    total_infer_time = 0.0
    session_start = time.time()

    pipeline = build_pipeline(args)
    with connect_with_retry(pipeline) as device:
        video_queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)
        print(f"YOLO stream from OAK-D Pro (USB) started on [{current_device}]. Press q to quit.")

        try:
            while True:
                in_frame = video_queue.get()
                frame = in_frame.getCvFrame()

                start = time.time()
                results = infer_strict(model, frame, args, current_device)
                infer_time = time.time() - start
                infer_fps = 1.0 / infer_time if infer_time > 0 else 0.0

                frame_count += 1
                total_infer_time += infer_time

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
                            annotated, label, (x1, y1 - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        )

                now = time.time()
                dt = now - last_time
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)
                last_time = now

                cv2.putText(annotated, f"PIPE FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated, f"INF FPS: {infer_fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(annotated, f"DEVICE: {current_device}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

                cv2.imshow("YOLO OAK-D Pro", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            print("Interrupted by user.")

    cv2.destroyAllWindows()

    total_time = time.time() - session_start
    avg_pipe_fps = frame_count / total_time if total_time > 0 else 0.0
    avg_inf_fps = frame_count / total_infer_time if total_infer_time > 0 else 0.0
    summary = (
        f"Frames processed: {frame_count}\n"
        f"Elapsed time:     {total_time:.2f} s\n"
        f"Avg pipeline FPS: {avg_pipe_fps:.2f}\n"
        f"Avg inference FPS:{avg_inf_fps:.2f}\n"
        f"Device:           {current_device}\n"
        f"Source:           OAK-D Pro (USB)\n"
        f"Model:            {model_path}\n"
    )
    print("\n=== Session summary ===")
    print(summary, end="")

    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fps_log.txt")
    with open(log_path, "a") as f:
        f.write(f"\n[{time.strftime('%Y-%m-%d %H:%M:%S')}]\n{summary}")
    print(f"Saved to: {log_path}")


if __name__ == "__main__":
    main()
