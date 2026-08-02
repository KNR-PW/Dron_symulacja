#!/usr/bin/env python3
"""
Takes a rectified mono8 :
left/right pair, runs an OpenCV StereoSGBM disparity search and reprojects
the pixels to 3D. Publishes:
  - <output_ns>/image_raw   sensor_msgs/Image  (16UC1, depth in millimeters)
  - <output_ns>/camera_info sensor_msgs/CameraInfo
  - <points_topic>          sensor_msgs/PointCloud2 (metric, camera optical frame)

"""

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2


class StereoDepth(Node):
    def __init__(self):
        super().__init__('stereo_depth')

        self.declare_parameter('left_topic', 'oak/left/image_raw')
        self.declare_parameter('right_topic', 'oak/right/image_raw')
        self.declare_parameter('output_ns', 'oak/stereo')
        self.declare_parameter('points_topic', 'oak/points')
        self.declare_parameter('optical_frame', 'oak_left_optical')

        self.declare_parameter('baseline', 0.075)
        self.declare_parameter('fx', 233.87)
        self.declare_parameter('fy', 233.87)
        self.declare_parameter('cx', 160.0)
        self.declare_parameter('cy', 120.0)

        self.declare_parameter('num_disparities', 64)
        self.declare_parameter('block_size', 7)
        self.declare_parameter('min_disparity', 0)
        self.declare_parameter('max_depth', 20.0)
        self.declare_parameter('swap_lr', False)
        self.declare_parameter('publish_points', True)

        self.baseline = float(self.get_parameter('baseline').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.swap_lr = bool(self.get_parameter('swap_lr').value)
        self.publish_points = bool(self.get_parameter('publish_points').value)
        self.optical_frame = self.get_parameter('optical_frame').value

        num_disp = int(self.get_parameter('num_disparities').value)
        num_disp = max(16, (num_disp // 16) * 16)
        block = int(self.get_parameter('block_size').value)
        min_disp = int(self.get_parameter('min_disparity').value)

        self.matcher = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block,
            P1=8 * block * block,
            P2=32 * block * block,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

        self.Q = np.array([
            [1, 0, 0, -self.cx],
            [0, 1, 0, -self.cy],
            [0, 0, 0, self.fx],
            [0, 0, -1.0 / self.baseline, 0],
        ], dtype=np.float64)

        self.bridge = CvBridge()

        output_ns = self.get_parameter('output_ns').value
        self.depth_pub = self.create_publisher(Image, f'{output_ns}/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, f'{output_ns}/camera_info', 10)
        self.points_pub = self.create_publisher(
            PointCloud2, self.get_parameter('points_topic').value, 10)

        left_sub = message_filters.Subscriber(self, Image, self.get_parameter('left_topic').value)
        right_sub = message_filters.Subscriber(self, Image, self.get_parameter('right_topic').value)
        self.sync = message_filters.ApproximateTimeSynchronizer([left_sub, right_sub], 10, 0.02)
        self.sync.registerCallback(self.stereo_callback)

        self.get_logger().info(
            f"StereoDepth ready: {self.get_parameter('left_topic').value} + "
            f"{self.get_parameter('right_topic').value} -> {output_ns}/image_raw "
            f"(baseline={self.baseline} m, fx={self.fx:.1f} px, disp={num_disp})")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        left = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
        if self.swap_lr:
            left, right = right, left

        disparity = self.matcher.compute(left, right).astype(np.float32) / 16.0

        valid = disparity > 0.0
        depth_m = np.zeros_like(disparity, dtype=np.float32)
        depth_m[valid] = (self.fx * self.baseline) / disparity[valid]
        depth_m[depth_m > self.max_depth] = 0.0

        depth_mm = (depth_m * 1000.0).astype(np.uint16)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, '16UC1')
        depth_msg.header = left_msg.header
        self.depth_pub.publish(depth_msg)

        self.info_pub.publish(self.build_camera_info(left_msg.header, left.shape))

        if self.publish_points and self.points_pub.get_subscription_count() > 0:
            self.publish_pointcloud(disparity, left_msg.header)

    def build_camera_info(self, header, shape) -> CameraInfo:
        info = CameraInfo()
        info.header = header
        info.height, info.width = int(shape[0]), int(shape[1])
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_pointcloud(self, disparity: np.ndarray, header):
        points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
        mask = (disparity > 0.0) & np.isfinite(points_3d[:, :, 2])
        mask &= points_3d[:, :, 2] < self.max_depth
        pts = points_3d[mask].astype(np.float32)
        if pts.shape[0] == 0:
            return

        msg = PointCloud2()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.optical_frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * pts.shape[0]
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.points_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
