"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Author: Nathan Sprague modified by Stanislaw Kolodziejczyk
Further modified to use the new ArUco API and manual pose estimation
Version: 06/05/2025
"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
import cv2
import tf_transformations

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from drone_interfaces.msg import ArucoMarkers, MiddleOfAruco


def make_aruco_detector(dictionary_name: str):
    """
    Zwraca funkcję detect(gray) -> (corners, ids, rejected), kompatybilną
    z nowym (ArucoDetector) i starym (detectMarkers) API OpenCV.
    """
    # stała słownika z cv2.aruco, np. DICT_4X4_50
    dict_const = getattr(cv2.aruco, dictionary_name)

    # Spróbuj nowego API (OpenCV >= 4.7)
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_const)
        aruco_params = cv2.aruco.DetectorParameters()  # nowe API
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        def detect(gray_img):
            return detector.detectMarkers(gray_img)

        return detect

    except AttributeError:
        # Stare API (OpenCV < 4.7)
        aruco_dict = cv2.aruco.Dictionary_get(dict_const)
        aruco_params = cv2.aruco.DetectorParameters_create()

        def detect(gray_img):
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_img, aruco_dict, parameters=aruco_params
            )
            return corners, ids, rejected

        return detect


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # --- Parametry ---
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
            value="DICT_4X4_50",  # bezpieczny default do Twoich tekstur Webots
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
        self.declare_parameter(
            name="dev",
            value="false",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="mode to print debug stuff."
            ),
        )

        # Odczyt parametrów
        self.marker_size = self.get_parameter("marker_size").value
        self.get_logger().info(f"Marker size: {self.marker_size} m")
        
        self.dev_mode = self.get_parameter("dev").value
        if self.dev_mode == "true":
            self.dev_mode = True
            self.get_logger().info("Dev mode enabled")
        else:
            self.dev_mode = "false"

        dictionary_id_name = self.get_parameter("aruco_dictionary_id").value
        self.get_logger().info(f"Aruco dictionary: {dictionary_id_name}")

        image_topic = self.get_parameter("image_topic").value
        self.get_logger().info(f"Subscribing to image topic: {image_topic}")

        intrinsic_list = self.get_parameter("intrinsic_matrix").value
        self.intrinsic_mat = np.reshape(np.array(intrinsic_list, dtype=np.float64), (3, 3))
        self.get_logger().info(f"Intrinsic matrix:\n{self.intrinsic_mat}")

        distortion_list = self.get_parameter("distortion").value
        self.distortion = np.array(distortion_list, dtype=np.float64)
        self.get_logger().info(f"Distortion coeffs: {self.distortion}")

        # Walidacje
        if self.intrinsic_mat.size != 9:
            self.get_logger().error("Intrinsic matrix must have 9 elements")
            raise RuntimeError("Bad intrinsic length")
        if self.distortion.size != 4:
            self.get_logger().error("Distortion must have 4 elements (k1,k2,p1,p2)")
            raise RuntimeError("Bad distortion length")

        # Walidacja i przygotowanie detektora
        try:
            _ = getattr(cv2.aruco, dictionary_id_name)
        except AttributeError:
            valid = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
            self.get_logger().error(
                f"Invalid aruco_dictionary_id: {dictionary_id_name}. Valid: {valid}"
            )
            raise RuntimeError("Bad aruco_dictionary_id")

        self.detect_aruco = make_aruco_detector(dictionary_id_name)

        # CV Bridge
        self.bridge = CvBridge()

        # Subskrypcja kamery
        self.create_subscription(Image, image_topic, self.image_callback, qos_profile_sensor_data)

        # Publikatory:
        # 1) pełne info: ArucoMarkers (tablice) – na /aruco_markers_full
        # self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers_full", 10)
        # self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        # 2) Środek największego markera: MiddleOfAruco – na /aruco_markers (to czyta Twoja misja)
        self.middle_pub = self.create_publisher(MiddleOfAruco, "aruco_markers", 10)

    def image_callback(self, img_msg: Image):
        # Upewnij się, że zdekodujemy poprawnie nawet przy encodingu 8UC3
        if self.dev_mode:
            self.get_logger().info("dziala?")
        try:
            cv_image_color = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception:
            # fallback: bez konwersji (jeśli bgr8 się nie uda)
            cv_image_color = self.bridge.imgmsg_to_cv2(img_msg)
        gray = cv2.cvtColor(cv_image_color, cv2.COLOR_BGR2GRAY)

        # Wykryj markery
        corners, marker_ids, _ = self.detect_aruco(gray)

        # Przygotuj wiadomości „pełne”
        # markers_msg = ArucoMarkers()
        # poses_msg = PoseArray()
        # markers_msg.header.stamp = img_msg.header.stamp
        # markers_msg.header.frame_id = ""
        # poses_msg.header.stamp = img_msg.header.stamp
        # poses_msg.header.frame_id = ""

        # Jeśli brak detekcji – opublikuj puste i wróć
        # if marker_ids is None or len(marker_ids) == 0:
        #     self.poses_pub.publish(poses_msg)
        #     self.markers_pub.publish(markers_msg)
        #     return

        # Znajdź największy marker (po polu w pikselach) – posłuży do MiddleOfAruco
        areas = []
        centers = []
        for c in corners:
            pts = c.reshape((4, 2)).astype(np.float32)
            areas.append(cv2.contourArea(pts))
            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())
            centers.append((cx, cy))
        if len(areas) > 0:
            k = int(np.argmax(areas))
            cx_big, cy_big = centers[k]

            # Opublikuj MiddleOfAruco na /aruco_markers (dla Twojej misji)
            mid = MiddleOfAruco()
            mid.x = int(cx_big)
            mid.y = int(cy_big)
            self.middle_pub.publish(mid)

        # Dla pełnego komunikatu policz pozycje przez solvePnP
        # half = self.marker_size / 2.0
        # obj_pts = np.array(
        #     [
        #         [-half,  half, 0.0],
        #         [ half,  half, 0.0],
        #         [ half, -half, 0.0],
        #         [-half, -half, 0.0],
        #     ],
        #     dtype=np.float64,
        # )

        # for idx, marker_id in enumerate(marker_ids.flatten()):
        #     img_pts = corners[idx].reshape((4, 2)).astype(np.float64)

        #     # PnP – spróbuj IPPE (kwadrat), fallback na ITERATIVE
        #     success, rvec, tvec = cv2.solvePnP(
        #         obj_pts, img_pts, self.intrinsic_mat, self.distortion,
        #         flags=cv2.SOLVEPNP_IPPE_SQUARE
        #     )
        #     if not success:
        #         success, rvec, tvec = cv2.solvePnP(
        #             obj_pts, img_pts, self.intrinsic_mat, self.distortion,
        #             flags=cv2.SOLVEPNP_ITERATIVE
        #         )
        #     if not success:
        #         continue

            # pose = Pose()
            # pose.position.x = float(tvec[0][0])
            # pose.position.y = float(tvec[1][0])
            # pose.position.z = float(tvec[2][0])

            # rot_mat_3x3, _ = cv2.Rodrigues(rvec)
            # rot_mat_4x4 = np.eye(4)
            # rot_mat_4x4[:3, :3] = rot_mat_3x3
            # qx, qy, qz, qw = tf_transformations.quaternion_from_matrix(rot_mat_4x4)
            # pose.orientation.x = float(qx)
            # pose.orientation.y = float(qy)
            # pose.orientation.z = float(qz)
            # pose.orientation.w = float(qw)

            # poses_msg.poses.append(pose)
            # markers_msg.poses.append(pose)
            # markers_msg.marker_ids.append(int(marker_id))

        # Spłaszcz rogi (do ArucoMarkers)
        # corners_flat = []
        # for c in corners:
        #     pts = c.reshape((4, 2))
        #     for (x, y) in pts:
        #         corners_flat.extend([float(x), float(y)])
        # markers_msg.corners = corners_flat

        # Publikacje „pełne”
        # self.poses_pub.publish(poses_msg)
        # self.markers_pub.publish(markers_msg)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

