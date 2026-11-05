#!/usr/bin/env python3
import argparse
import ctypes
import glob
import os
import site
import sys
import time

import cv2


def parse_args() -> argparse.Namespace:
    default_model = os.path.join(os.path.dirname(__file__), "yolo26s.pt")

    parser = argparse.ArgumentParser(
        description="Run YOLO inference on a video file and optionally save annotated output."
    )
    parser.add_argument("--video", required=True, help="Path to input video file")
    parser.add_argument(
        "--model",
        default=default_model,
        help="Path to YOLO model (.pt, .engine, etc.)",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional path to save output video with boxes",
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size")
    parser.add_argument(
        "--device",
        default="auto",
        help="Inference device: auto, cpu, cuda:0",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show live preview window (press q to quit)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames (0 = full video)",
    )
    parser.add_argument(
        "--half",
        dest="half",
        action="store_true",
        help="Use FP16 inference (recommended on Jetson CUDA)",
    )
    parser.add_argument(
        "--no-half",
        dest="half",
        action="store_false",
        help="Disable FP16 inference",
    )
    parser.set_defaults(half=None)
    return parser.parse_args()


def resolve_device(torch_module, device_arg: str) -> str:
    if device_arg != "auto":
        return device_arg
    return "cuda:0" if torch_module.cuda.is_available() else "cpu"


def configure_cuda_for_jetson(torch_module, requested_device: str) -> bool:
    if "cuda" not in requested_device:
        return True
    if not torch_module.cuda.is_available():
        return False

    # Jetson stability settings: reduce cuDNN engine selection issues.
    torch_module.backends.cudnn.benchmark = False
    torch_module.backends.cudnn.deterministic = True
    torch_module.backends.cuda.matmul.allow_tf32 = True

    try:
        dummy = torch_module.randn(1, 3, 64, 64, device="cuda:0")
        conv = torch_module.nn.Conv2d(3, 8, 3, padding=1).to("cuda:0")
        _ = conv(dummy)
        torch_module.cuda.synchronize()
        return True
    except Exception:
        return False


def candidate_nvidia_lib_dirs() -> list[str]:
    dirs: list[str] = []

    try:
        usersite = site.getusersitepackages()
        if isinstance(usersite, str):
            dirs.extend(glob.glob(os.path.join(usersite, "nvidia", "*", "lib")))
    except Exception:
        pass

    try:
        for sp in site.getsitepackages():
            dirs.extend(glob.glob(os.path.join(sp, "nvidia", "*", "lib")))
    except Exception:
        pass

    # Fallback patterns for non-standard or venv layouts on Jetson.
    dirs.extend(glob.glob(os.path.expanduser("~/.local/lib/python*/site-packages/nvidia/*/lib")))
    dirs.extend(glob.glob(os.path.expanduser("~/*/.venv/lib/python*/site-packages/nvidia/*/lib")))

    unique_dirs: list[str] = []
    for d in dirs:
        if os.path.isdir(d) and d not in unique_dirs:
            unique_dirs.append(d)
    return unique_dirs


def preload_jetson_cuda_libs() -> str | None:
    lib_name = "libcudss.so.0"
    candidates = candidate_nvidia_lib_dirs()
    found_path = None

    for d in candidates:
        path = os.path.join(d, lib_name)
        if os.path.isfile(path):
            found_path = path
            break

    if not found_path:
        return None

    lib_dir = os.path.dirname(found_path)
    ld_parts = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
    if lib_dir not in ld_parts:
        os.environ["LD_LIBRARY_PATH"] = ":".join([lib_dir] + ld_parts)

    # Preload into this process so torch can resolve transitive dependencies.
    ctypes.CDLL(found_path, mode=ctypes.RTLD_GLOBAL)
    return found_path


def import_runtime_modules():
    try:
        import torch
        from ultralytics import YOLO
        return torch, YOLO
    except Exception as first_exc:
        loaded_lib = None
        preload_error = None

        try:
            loaded_lib = preload_jetson_cuda_libs()
        except Exception as exc:
            preload_error = exc

        try:
            import torch
            from ultralytics import YOLO
            return torch, YOLO
        except Exception as second_exc:
            details = [
                "Could not import torch/ultralytics.",
                f"Python: {sys.executable}",
                f"Initial import error: {first_exc}",
                f"Import error after preload: {second_exc}",
            ]
            if loaded_lib:
                details.append(f"Tried preloading: {loaded_lib}")
            if preload_error:
                details.append(f"Preload warning: {preload_error}")
            details.append(
                "Jetson tip: if you see 'libcudss.so.0' errors, run:\n"
                "export LD_LIBRARY_PATH=$HOME/.local/lib/python3.10/site-packages/nvidia/cu12/lib:$LD_LIBRARY_PATH"
            )
            raise RuntimeError("\n".join(details)) from second_exc


def ensure_parent_dir(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def main() -> None:
    args = parse_args()
    torch, YOLO = import_runtime_modules()

    video_path = os.path.expanduser(args.video)
    model_path = os.path.expanduser(args.model)

    if not os.path.isfile(video_path):
        raise FileNotFoundError(f"Input video not found: {video_path}")
    if not os.path.isfile(model_path):
        raise FileNotFoundError(f"Model file not found: {model_path}")

    device = resolve_device(torch, args.device)
    cuda_ready = configure_cuda_for_jetson(torch, device)
    if "cuda" in device and not cuda_ready:
        if args.device == "auto":
            print("CUDA warmup failed on this Jetson stack, falling back to CPU.")
            device = "cpu"
        else:
            raise RuntimeError(
                "CUDA preflight failed. Try --device cpu or install Jetson-compatible Torch/CUDA packages."
            )

    print(f"Loading model: {model_path}")
    print(f"Using device: {device}")
    use_half = args.half if args.half is not None else ("cuda" in device)
    print(f"Using FP16: {use_half}")
    model = YOLO(model_path)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    in_fps = cap.get(cv2.CAP_PROP_FPS)
    in_fps = in_fps if in_fps and in_fps > 0 else 30.0
    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = None
    if args.output:
        output_path = os.path.expanduser(args.output)
        ensure_parent_dir(output_path)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(output_path, fourcc, in_fps, (frame_w, frame_h))
        if not writer.isOpened():
            raise RuntimeError(f"Could not create output video: {output_path}")
        print(f"Saving annotated video to: {output_path}")

    frame_idx = 0
    t0 = time.time()
    switched_to_cpu = False
    reduced_imgsz = False

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            frame_idx += 1

            infer_start = time.time()
            try:
                result = model.predict(
                    source=frame,
                    conf=args.conf,
                    imgsz=args.imgsz,
                    device=device,
                    half=use_half,
                    verbose=False,
                )[0]
            except RuntimeError as exc:
                msg = str(exc).lower()
                if (
                    "cuda" in device
                    and args.device == "auto"
                    and "unable to find an engine to execute this computation" in msg
                    and not switched_to_cpu
                ):
                    print("GPU engine error on Jetson, switching to CPU and continuing.")
                    device = "cpu"
                    switched_to_cpu = True
                    result = model.predict(
                        source=frame,
                        conf=args.conf,
                        imgsz=args.imgsz,
                        device=device,
                        half=False,
                        verbose=False,
                    )[0]
                elif (
                    "cuda" in device
                    and "cublas_status_alloc_failed" in msg
                    and not reduced_imgsz
                ):
                    # On Orin Nano this model may need FP16 and smaller input.
                    new_imgsz = min(args.imgsz, 256)
                    if new_imgsz == args.imgsz and not use_half:
                        use_half = True
                        print("cublas alloc failed, retrying with FP16.")
                    else:
                        args.imgsz = new_imgsz
                        use_half = True
                        reduced_imgsz = True
                        print(f"cublas alloc failed, retrying with imgsz={args.imgsz}, FP16 enabled.")

                    result = model.predict(
                        source=frame,
                        conf=args.conf,
                        imgsz=args.imgsz,
                        device=device,
                        half=use_half,
                        verbose=False,
                    )[0]
                else:
                    raise

            infer_ms = (time.time() - infer_start) * 1000.0

            annotated = result.plot()
            det_count = len(result.boxes) if result.boxes is not None else 0
            cv2.putText(
                annotated,
                f"frame: {frame_idx}  detections: {det_count}  infer: {infer_ms:.1f} ms",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if writer is not None:
                writer.write(annotated)

            if args.show:
                cv2.imshow("YOLO video test", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break

            if args.max_frames > 0 and frame_idx >= args.max_frames:
                break

    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()

    total_time = time.time() - t0
    avg_fps = frame_idx / total_time if total_time > 0 else 0.0
    print(f"Processed frames: {frame_idx}")
    print(f"Elapsed time: {total_time:.2f} s")
    print(f"Average throughput: {avg_fps:.2f} FPS")


if __name__ == "__main__":
    main()
