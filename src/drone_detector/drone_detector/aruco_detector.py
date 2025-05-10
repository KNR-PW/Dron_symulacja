import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
#from drone_interfaces.msg import ArucoMarker, ArucoMarkersList
from drone_interfaces.msg import MiddleofAruco
from drone_interfaces.srv import GetLocationRelative
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import datetime

# DEPRECATED. WE NOW USE ROS2_ARUCO PACKAGE

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10)
        self.gps_cli = self.create_client(GetLocationRelative, "get_location_relative")
        # Create the publisher for Aruco marker data
        self.publisher = self.create_publisher(MiddleofAruco, 'aruco_markers', 10)

        self.br = CvBridge()
        self.get_logger().info('aruco_detector node created')

    def get_gps(self):
        self.get_logger().info("Sending GPS request")
        request_gps = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request_gps)
        rclpy.spin_until_future_complete(self, gps_future, timeout_sec=10)
        if gps_future.result() is not None:
            self.north = gps_future.result().north
            self.east = gps_future.result().east
            self.down = gps_future.result().down
            self.drone_amplitude = -self.down
            self.get_logger().info("GPS Recieved")
        else:
            self.get_logger().info("GPS request failed")
            self.drone_amplitude = 0
            return None
        return self.down

    def camera_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)

        # Convert the image to grayscale
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)

        # Load the predefined dictionary
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

        # Initialize the detector parameters using default values
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # Detect the markers in the image
        corners, ids, rejectedImgPoints = detector.detectMarkers(current_frame)

        #aruco_markers_list = ArucoMarkersList()
        middle_of_aruco = MiddleofAruco()
        mid = [0,0]
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i].flatten().tolist()
                #aruco_marker = ArucoMarker()
                #aruco_marker.marker_id = int(marker_id)
                #aruco_marker.corners = marker_corners
                #aruco_markers_list.aruco_markers.append(aruco_marker)
                
                middle_of_aruco.x = int((marker_corners[0]+marker_corners[4])/2)
                middle_of_aruco.y = int((marker_corners[1]+marker_corners[5])/2)
                mid = [int((marker_corners[0]+marker_corners[4])/2),int((marker_corners[1]+marker_corners[5])/2)]
                self.get_logger().info(f'Marker ID: {marker_id}, corners: {marker_corners}')
                frame_markers = aruco.drawDetectedMarkers(current_frame.copy(), corners, ids)
                now = datetime.datetime.now()
                height = 0
                cv2.imwrite("/home/knr/Dron_symulacja/saved_images/detect/"+str(height)+"m"+str(now)+".jpg",frame_markers)
        #self.publisher.publish(aruco_markers_list)

        current_frame = cv2.circle(current_frame,(mid[0],mid[1]),radius = 5,color=(255,0,0), thickness=-1)
        
        #cv2.imshow("camera", current_frame)

        #cv2.waitKey(1)
        self.publisher.publish(middle_of_aruco)


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
