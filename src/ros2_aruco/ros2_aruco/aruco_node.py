"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Author: Nathan Sprague modified by Stanislaw Kolodziejczyk
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from drone_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            # value="DICT_5X5_250",
            # value="DICT_4X4_100",
            value="DICT_4X4_250",
            # value="DICT_ARUCO_ORIGINAL",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="intrinsic_matrix",
            value=[1.0, 0.0, 320.0, 0.0, 1.0, 240.0, 0.0, 0.0, 1.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera intrinsic matrix.",
            ),
        )

        self.declare_parameter(
            name="distortion",
            value=[0.0, 0.0, 0.0, 0.0],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description="Camera distortion coefficients.",
            ),
        )


        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        self.intrinsic_mat = (
            self.get_parameter("intrinsic_matrix").get_parameter_value().double_array_value
        )
        self.intrinsic_mat = np.reshape(np.array(self.intrinsic_mat), (3, 3))
        self.get_logger().info(f"Intrinsic matrix: {self.intrinsic_mat}")

        self.distortion = (
            self.get_parameter("distortion").get_parameter_value().double_array_value
        )
        self.distortion = np.array(self.distortion)
        self.get_logger().info(f"Distortion: {self.distortion}")

        # Check that the intrinsic matrix is 3x3
        if self.intrinsic_mat.size != 9:
            self.get_logger().error(
                "Intrinsic matrix must be 3x3, got {} values".format(
                    len(self.intrinsic_mat)
                )
            )
            return
        # Check that the distortion is 1x4
        if self.distortion.size != 4:
            self.get_logger().error(
                "Distortion must be 1x4, got {} values".format(len(self.distortion))
            )
            return

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)


        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()

        self.bridge = CvBridge()

    def image_callback(self, img_msg):
        # self.get_logger().info(f"Received image")
        cv_image = self.bridge.imgmsg_to_cv2(img_msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markers = ArucoMarkers()
        pose_array = PoseArray()

        markers.header.frame_id = ""
        pose_array.header.frame_id = ""

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )
        if marker_ids is not None:
            self.get_logger().info(f"Found {len(marker_ids)} markers")
            self.get_logger().info(f"Marker ids: {marker_ids.flatten()}")
            if cv2.__version__ > "4.0.0":
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.intrinsic_mat, self.distortion
                )
            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
