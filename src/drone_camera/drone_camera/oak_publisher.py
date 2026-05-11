"""OAK camera publisher using depthai 2.x directly.

Bypasses depthai_ros_driver which silently fails to connect to PoE OAK on Jetson.
Connects via dai.Device(state=ANY) - the only path that works for this hardware.

Topics published (matching depthai_ros_driver naming):
  /oak/rgb/image_raw        (sensor_msgs/Image, bgr8)
  /oak/rgb/camera_info      (sensor_msgs/CameraInfo)

Parameters:
  ip            (str)   IP address of PoE OAK. Empty = USB autodetect. Default: ""
  fps           (int)   RGB fps. Default: 60
  width         (int)   RGB width. Default: 1280
  height        (int)   RGB height. Default: 720
  frame_id      (str)   TF frame_id. Default: "oak_rgb_camera_optical_frame"
"""

import sys
import threading

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class OakPublisher(Node):
    def __init__(self):
        super().__init__("oak_publisher")

        self.declare_parameter("ip", "")
        self.declare_parameter("fps", 60)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("frame_id", "oak_rgb_camera_optical_frame")

        self.ip = self.get_parameter("ip").value
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/oak/rgb/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/oak/rgb/camera_info", 10)

        self.device = None
        self.q_rgb = None
        self._stop = threading.Event()
        self._thread = None

        self._connect_and_start()

    def _build_pipeline(self):
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(self.fps)
        cam.setVideoSize(self.width, self.height)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("rgb")
        cam.video.link(xout.input)
        return pipeline

    def _connect_and_start(self):
        if self.ip:
            self.get_logger().info(f"Connecting to OAK at {self.ip}")
            info = dai.DeviceInfo(self.ip)
            self.device = dai.Device(self._build_pipeline(), info)
        else:
            self.get_logger().info("Connecting to first available OAK (USB)")
            self.device = dai.Device(self._build_pipeline())

        self.get_logger().info(
            f"Connected. MxID={self.device.getMxId()} Type={self.device.getDeviceName()}"
        )
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()

    def _publish_loop(self):
        while not self._stop.is_set():
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is None:
                self._stop.wait(0.001)
                continue
            frame = in_rgb.getCvFrame()
            stamp = self.get_clock().now().to_msg()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)

            ci = CameraInfo()
            ci.header.stamp = stamp
            ci.header.frame_id = self.frame_id
            ci.height = frame.shape[0]
            ci.width = frame.shape[1]
            self.info_pub.publish(ci)

    def destroy_node(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
