"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Author: Nathan Sprague modified by Stanislaw Kolodziejczyk
Further modified to use the new ArUco API and manual pose estimation
Version: 06/05/2025
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from drone_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # --- Declare and read parameters ---
        self.declare_parameter(
            name="marker_size",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters."
            ),
        )
        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_4X4_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers."
            ),
        )
        self.declare_parameter(
            name="image_topic",
            value="/camera",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to."
            ),
        )
        self.declare_parameter(
            name="intrinsic_matrix",
            value=[1.0, 0.0, 320.0,
                   0.0, 1.0, 240.0,
                   0.0, 0.0,   1.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera intrinsic matrix."
            ),
        )
        self.declare_parameter(
            name="distortion",
            value=[0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera distortion coefficients (k1, k2, p1, p2)."
            ),
        )

        # Read marker size
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size} m")

        # Read dictionary ID (string like "DICT_4X4_250")
        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco dictionary: {dictionary_id_name}")

        # Read image topic
        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")

        # Read intrinsic matrix (flattened list of 9 doubles)
        intrinsic_list = (
            self.get_parameter("intrinsic_matrix").get_parameter_value().double_array_value
        )
        self.intrinsic_mat = np.reshape(np.array(intrinsic_list, dtype=np.float64), (3, 3))
        self.get_logger().info(f"Intrinsic matrix:\n{self.intrinsic_mat}")

        # Read distortion coefficients (list of 4 doubles)
        distortion_list = (
            self.get_parameter("distortion").get_parameter_value().double_array_value
        )
        self.distortion = np.array(distortion_list, dtype=np.float64)
        self.get_logger().info(f"Distortion coeffs: {self.distortion}")

        # Validate sizes
        if self.intrinsic_mat.size != 9:
            self.get_logger().error(
                f"Intrinsic matrix must have 9 elements, got {self.intrinsic_mat.size}"
            )
            return
        if self.distortion.size != 4:
            self.get_logger().error(
                f"Distortion must have 4 elements (k1, k2, p1, p2), got {self.distortion.size}"
            )
            return

        # Validate dictionary ID
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            # Ensure it's an integer constant
            if not isinstance(dictionary_id, (int, np.integer)):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                f"Invalid aruco_dictionary_id: {dictionary_id_name}"
            )
            valid = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
            self.get_logger().error(f"Valid options: {valid}")
            return

        # Set up ArUco dictionary and detector (new API)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dictionary, self.aruco_parameters
        )

        # Set up CV Bridge
        self.bridge = CvBridge()

        # Create subscription to image topic
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Create publishers for poses and marker info
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)


    def image_callback(self, img_msg):
        # Convert ROS Image to OpenCV grayscale
        cv_image_color = self.bridge.imgmsg_to_cv2(img_msg)
        cv_image = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)

        # Prepare output messages
        markers_msg = ArucoMarkers()
        poses_msg = PoseArray()
        markers_msg.header.stamp = img_msg.header.stamp
        markers_msg.header.frame_id = ""
        poses_msg.header.stamp = img_msg.header.stamp
        poses_msg.header.frame_id = ""

        # Detect markers using the new ArucoDetector API
        corners, marker_ids, _ = self.aruco_detector.detectMarkers(cv_image)

        if marker_ids is None or len(marker_ids) == 0:
            # No markers detected: publish empty messages
            self.poses_pub.publish(poses_msg)
            self.markers_pub.publish(markers_msg)
            return

        # For each detected marker, perform pose estimation via solvePnP
        for idx, marker_id in enumerate(marker_ids.flatten()):
            # corners[idx] has shape (1, 4, 2); reshape to (4, 2)
            img_pts = corners[idx].reshape((4, 2)).astype(np.float64)

            # Define the 3D coordinates of the marker's corners in the marker frame
            # The order must match the OpenCV convention: top-left, top-right, bottom-right, bottom-left
            half_size = self.marker_size / 2.0
            obj_pts = np.array([
                [-half_size,  half_size, 0.0],
                [ half_size,  half_size, 0.0],
                [ half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0]
            ], dtype=np.float64)

            # Solve PnP to get rotation and translation vectors
            # Using SOLVEPNP_IPPE_SQUARE for square planar markers is recommended if available
            success, rvec, tvec = cv2.solvePnP(
                obj_pts,
                img_pts,
                self.intrinsic_mat,
                self.distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                # Fallback to iterative method if IPPE fails
                success, rvec, tvec = cv2.solvePnP(
                    obj_pts,
                    img_pts,
                    self.intrinsic_mat,
                    self.distortion,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

            if not success:
                # Could not estimate pose for this marker; skip it
                continue

            # Build Pose message
            pose = Pose()
            pose.position.x = float(tvec[0][0])
            pose.position.y = float(tvec[1][0])
            pose.position.z = float(tvec[2][0])

            # Convert rotation vector to quaternion
            rot_mat_4x4 = np.eye(4)
            rot_mat_3x3, _ = cv2.Rodrigues(rvec)
            rot_mat_4x4[0:3, 0:3] = rot_mat_3x3
            quat = tf_transformations.quaternion_from_matrix(rot_mat_4x4)

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            # Append to PoseArray and ArucoMarkers messages
            poses_msg.poses.append(pose)
            markers_msg.poses.append(pose)
            markers_msg.marker_ids.append(int(marker_id))

        # Flatten all corner points into a single list of floats
        corners_flat = []
        for c in corners:
            # c has shape (1, 4, 2)
            pts = c.reshape((4, 2))
            for (x, y) in pts:
                corners_flat.extend([float(x), float(y)])
        markers_msg.corners = corners_flat
        # Log detected marker IDs
        self.get_logger().info(f"Detected marker IDs: {marker_ids.flatten().tolist()}")
        # Publish results
        self.poses_pub.publish(poses_msg)
        self.markers_pub.publish(markers_msg)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
