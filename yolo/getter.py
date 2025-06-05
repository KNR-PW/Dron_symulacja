# Basic ROS 2 program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import os
import json
from droniada_inspekcja.inspekcja_db import DroneDB
from drone_interfaces.srv import (
    GetLocationRelative,
    GetAttitude,
    SetMode,
    SetSpeed,
)


def json_counter(sciezka):
    max_liczba = -1
    plik_z_max_liczba = None

    try:
        for nazwa_pliku in os.listdir(sciezka):
            pelna_sciezka = os.path.join(sciezka, nazwa_pliku)
            if os.path.isfile(pelna_sciezka) and nazwa_pliku.endswith('.json'):
                bez_rozszerzenia = nazwa_pliku[:-5]  # usuń ".json"
                if bez_rozszerzenia.isdigit():
                    liczba = int(bez_rozszerzenia)
                    if liczba > max_liczba:
                        max_liczba = liczba
                        plik_z_max_liczba = nazwa_pliku

        if plik_z_max_liczba:
            print(f"Plik z największą liczbą w nazwie: {plik_z_max_liczba} (liczba: {max_liczba})")
            return max_liczba

    except FileNotFoundError:
        print("Podany katalog nie istnieje.")
    except PermissionError:
        print("Brak uprawnień do odczytu katalogu.")

def photo_counter(sciezka):
    try:
        pliki = [f for f in os.listdir(sciezka) if os.path.isfile(os.path.join(sciezka, f))]
        # print(f"Liczba plików w katalogu '{sciezka}': {len(pliki)}")
        return len(pliki)
    except FileNotFoundError:
        print("Podany katalog nie istnieje.")
    except PermissionError:
        print("Brak uprawnień do odczytu katalogu.")



class ImagePublisher(Node):
    """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """

    def __init__(self):
        """
    Class constructor to set up the node
    """
        # Initiate the Node class's constructor and give it a name
        super().__init__('smierc')

        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'camera', 10)

        # We will publish a message every 0.1 seconds
        timer_period = 0.1  # seconds
        self._gps_client  = self.create_client(GetLocationRelative, 'get_location_relative')

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_frame = 0
        self.last_json = 0
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
        self.get_logger().info('Image Publisher node created')
        self.db = DroneDB("drone_data.db")
        self.mission_id = self.db.add_mission(
            team="Team A",
            email="pilot@example.com",
            pilot="Test Pilot",
            phone="000-000-0000",
            mission_time="47",
            mission_no=f"Mrafal",
            duration="0m",
            battery_before="100%",
            battery_after="100%",
            kp_index=0,
            infra_map="/static/img/mapa.jpg",
        )


    def timer_callback(self):
        """
    Callback function.
    This function gets called every 0.1 seconds.
    """
        # Capture frame-by-frame
        # This method returns True/False as well
        path = "/root/ros_ws/hailo/"
        p_path = path+"photo/"
        j_path = path+"json/"
        current_photo = photo_counter(p_path)
        current_json = json_counter(j_path)
        if self.last_frame != current_photo:
            f_path = p_path + str(current_photo) + ".jpg"

            self.frame = cv2.imread(f_path)

            gps = self.get_gps()
            cur_north, cur_east, cur_down = gps
            gps_str = str(cur_down)+" "+str(cur_east)+" "+str(cur_north)
            
            

            self.publisher_.publish(self.br.cv2_to_imgmsg(self.frame))
            self.last_frame = current_photo
        if self.last_json != current_json:
            s_path = j_path + str(current_json) + ".json"
            with open(s_path, "r", encoding="utf-8") as plik:
                gps = self.get_gps()
                cur_north, cur_east, cur_down = gps
                gps_str = str(cur_down)+" "+str(cur_east)+" "+str(cur_north)
                lista = json.load(plik)
                print(lista)
                self.last_json = current_json
                to_store = []
                bhp_str = ""
                if lista["hat"] == "none":
                    bhp_str+="0"
                else:
                    bhp_str+="1"

                if lista["vest"] == "none":
                    bhp_str+="0"
                else:
                    bhp_str+="1"

                record = {
                    "present": "tak",
                    "bhp": bhp_str,
                    "location": gps_str,
                    "location_changed": "-",
                    "content_changed": "-",
                    "image": p_path+str(current_json)+".jpg",
                    "jury": "-"

                }
                to_store.append(record)

                try:
                    self.db.add_arucos(self.mission_id, to_store)
                except Exception as e:
                    self.get_logger().error(f"Failed to save ArUco(s) to DB: {e}")

    def get_gps(self):
        req = GetLocationRelative.Request()
        fut = self._gps_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if fut.result() is None:
            self.get_logger().error('GPS service failed')
            return None
        return (fut.result().north, fut.result().east, fut.result().down)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()