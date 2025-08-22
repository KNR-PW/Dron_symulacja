'''
This file implements a class that acts as a bridge between ArduPilot SITL and Webots

AP_FLAKE8_CLEAN
'''

# Imports
import os
import sys
import time
import socket
import select
import struct
import numpy as np
from threading import Thread
from typing import List, Union
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# Here we set up environment variables so we can run this script
# as an external controller outside of Webots (useful for debugging)
# https://cyberbotics.com/doc/guide/running-extern-robot-controllers
if sys.platform.startswith("win"):
    WEBOTS_HOME = "C:\\Program Files\\Webots"
elif sys.platform.startswith("darwin"):
    WEBOTS_HOME = "/Applications/Webots.app"
elif sys.platform.startswith("linux"):
    WEBOTS_HOME = "/usr/local/webots"
else:
    raise Exception("Unsupported OS")

if os.environ.get("WEBOTS_HOME") is None:
    os.environ["WEBOTS_HOME"] = WEBOTS_HOME
else:
    WEBOTS_HOME = os.environ.get("WEBOTS_HOME")

os.environ["PYTHONIOENCODING"] = "UTF-8"
sys.path.append(f"{WEBOTS_HOME}/lib/controller/python")

from controller import Robot, Camera, RangeFinder # noqa: E401, E402


class WebotsArduVehicleRos():
    """Class representing an ArduPilot controlled Webots Vehicle"""

    controls_struct_format = 'f'*16
    controls_struct_size = struct.calcsize(controls_struct_format)
    fdm_struct_format = 'd'*(1+3+3+3+3+3)
    fdm_struct_size = struct.calcsize(fdm_struct_format)

    def __init__(self,
                 motor_names: List[str],
                 accel_name: str = "accelerometer",
                 imu_name: str = "inertial unit",
                 gyro_name: str = "gyro",
                 gps_name: str = "gps",
                 camera_name: str = None,
                 camera_fps: int = 10,
                 camera_stream_port: int = None,
                 gimbal_camera_name: str = None,
                 gimbal_camera_stream_port: int = None,
                 gimbal_servo_name: str = None,
                 rangefinder_name: str = None,
                 rangefinder_fps: int = 10,
                 rangefinder_stream_port: int = None,
                 instance: int = 0,
                 motor_velocity_cap: float = float('inf'),
                 reversed_motors: List[int] = None,
                 bidirectional_motors: bool = False,
                 uses_propellers: bool = True,
                 sitl_address: str = "127.0.0.1"):
        """WebotsArduVehicle constructor

        Args:
            motor_names (List[str]): Motor names in ArduPilot numerical order (first motor is SERVO1 etc).
            accel_name (str, optional): Webots accelerometer name. Defaults to "accelerometer".
            imu_name (str, optional): Webots imu name. Defaults to "inertial unit".
            gyro_name (str, optional): Webots gyro name. Defaults to "gyro".
            gps_name (str, optional): Webots GPS name. Defaults to "gps".
            camera_name (str, optional): Webots camera name. Defaults to None.
            camera_fps (int, optional): Camera FPS. Lower FPS runs better in sim. Defaults to 10.
            camera_stream_port (int, optional): Port to stream grayscale camera images to.
                                                If no port is supplied the camera will not be streamed. Defaults to None.
            rangefinder_name (str, optional): Webots RangeFinder name. Defaults to None.
            rangefinder_fps (int, optional): RangeFinder FPS. Lower FPS runs better in sim. Defaults to 10.
            rangefinder_stream_port (int, optional): Port to stream rangefinder images to.
                                                     If no port is supplied the camera will not be streamed. Defaults to None.
            instance (int, optional): Vehicle instance number to match the SITL. This allows multiple vehicles. Defaults to 0.
            motor_velocity_cap (float, optional): Motor velocity cap. This is useful for the crazyflie
                                                  which default has way too much power. Defaults to float('inf').
            reversed_motors (list[int], optional): Reverse the motors (indexed from 1). Defaults to None.
            bidirectional_motors (bool, optional): Enable bidirectional motors. Defaults to False.
            uses_propellers (bool, optional): Whether the vehicle uses propellers.
                                              This is important as we need to linearize thrust if so. Defaults to True.
            sitl_address (str, optional): IP address of the SITL (useful with WSL2 eg \"172.24.220.98\").
                                          Defaults to "127.0.0.1".
        """
        # init class variables
        self.motor_velocity_cap = motor_velocity_cap
        self._instance = instance
        self._reversed_motors = reversed_motors
        self._bidirectional_motors = bidirectional_motors
        self._uses_propellers = uses_propellers
        self._webots_connected = True

        # Setup ros2
        rclpy.init()
        self.node = Node('webots_vehicle_node')
        self.camera_publisher = self.node.create_publisher(Image, 'camera', 10)
        self.gimbal_camera_publisher = self.node.create_publisher(Image, 'gimbal_camera', 10)

        self.gps_publisher = self.node.create_publisher(Image, 'gps', 10)
        self.br = CvBridge()
        # setup Webots robot instance
        self.robot = Robot()

        # set robot time step relative to sim time step
        self._timestep = int(self.robot.getBasicTimeStep())

        # init sensors
        self.accel = self.robot.getDevice(accel_name)
        self.imu = self.robot.getDevice(imu_name)
        self.gyro = self.robot.getDevice(gyro_name)
        self.gps = self.robot.getDevice(gps_name)

        self.accel.enable(self._timestep)
        self.imu.enable(self._timestep)
        self.gyro.enable(self._timestep)
        self.gps.enable(self._timestep)

        # init camera
        if camera_name is not None:
            self.camera = self.robot.getDevice(camera_name)
            if self.camera:
                self.camera.enable(1000 // camera_fps)
                self._camera_thread = Thread(daemon=True,
                                            target=self._handle_image_stream,
                                            args=[self.camera, self.camera_publisher]) # Przekazujemy publishera
                self._camera_thread.start()
            else:
                self.node.get_logger().warn(f"Device '{camera_name}' not found.")

        if gimbal_camera_name is not None:
            self.gimbal_camera = self.robot.getDevice(gimbal_camera_name)
            if self.gimbal_camera:
                self.gimbal_camera.enable(1000 // camera_fps)
                self._gimbal_camera_thread = Thread(daemon=True,
                                                    target=self._handle_image_stream,
                                                    args=[self.gimbal_camera, self.gimbal_camera_publisher]) # Przekazujemy publishera
                self._gimbal_camera_thread.start()
            else:
                self.node.get_logger().warn(f"Device '{gimbal_camera_name}' not found.")

        if gimbal_servo_name is not None:
            self.gimbal_servo = self.robot.getDevice(gimbal_servo_name)
            self.gimbal_servo.setPosition(45.0 * 3.14 / 180.0) # point gimbal downwards by default
  
        # init rangefinder
        if rangefinder_name is not None:
            self.rangefinder = self.robot.getDevice(rangefinder_name)
            self.rangefinder.enable(1000//rangefinder_fps) # takes frame period in ms

            # start rangefinder streaming thread if requested
            if rangefinder_stream_port is not None:
                self._rangefinder_thread = Thread(daemon=True,
                                                  target=self._handle_image_stream,
                                                  args=[self.rangefinder, rangefinder_stream_port])
                self._rangefinder_thread.start()

        # init motors (and setup velocity control)
        self._motors = [self.robot.getDevice(n) for n in motor_names]
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

        # start ArduPilot SITL communication thread
        self._sitl_thread = Thread(daemon=True, target=self._handle_sitl, args=[sitl_address, 9002+10*instance])
        self._sitl_thread.start()

    def _handle_sitl(self, sitl_address: str = "127.0.0.1", port: int = 9002):
        """Handles all communications with the ArduPilot SITL

        Args:
            port (int, optional): Port to listen for SITL on. Defaults to 9002.
        """

        # create a local UDP socket server to listen for SITL
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # SOCK_STREAM
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', port))

        # wait for SITL to connect
        print(f"Listening for ardupilot SITL (I{self._instance}) at 127.0.0.1:{port}")
        self.robot.step(self._timestep) # flush print in webots console

        while not select.select([s], [], [], 0)[0]: # wait for socket to be readable
            # if webots is closed, close the socket and exit
            if self.robot.step(self._timestep) == -1:
                s.close()
                self._webots_connected = False
                return

        print(f"Connected to ardupilot SITL (I{self._instance})")

        # main loop handling communications
        while True:
            # check if the socket is ready to send/receive
            readable, writable, _ = select.select([s], [s], [], 0)

            # send data to SITL port (one lower than its output port as seen in SITL_cmdline.cpp)
            if writable:
                fdm_struct = self._get_fdm_struct()
                s.sendto(fdm_struct, (sitl_address, port+1))

            # receive data from SITL port
            if readable:
                data = s.recv(512)
                if not data or len(data) < self.controls_struct_size:
                    continue

                # parse a single struct
                command = struct.unpack(self.controls_struct_format, data[:self.controls_struct_size])
                self._handle_controls(command)

                # wait until the next Webots time step as no new sensor data will be available until then
                step_success = self.robot.step(self._timestep)
                if step_success == -1: # webots closed
                    break

        # if we leave the main loop then Webots must have closed
        s.close()
        self._webots_connected = False
        print(f"Lost connection to Webots (I{self._instance})")

    def _get_fdm_struct(self) -> bytes:
        """Form the Flight Dynamics Model struct (aka sensor data) to send to the SITL

        Returns:
            bytes: bytes representing the struct to send to SITL
        """
        # get data from Webots
        i = self.imu.getRollPitchYaw()
        g = self.gyro.getValues()
        a = self.accel.getValues()
        gps_pos = self.gps.getValues()
        gps_vel = self.gps.getSpeedVector()

        # pack the struct, converting ENU to NED (ish)
        # https://discuss.ardupilot.org/t/copter-x-y-z-which-is-which/6823/3
        # struct fdm_packet {
        #     double timestamp;
        #     double imu_angular_velocity_rpy[3];
        #     double imu_linear_acceleration_xyz[3];
        #     double imu_orientation_rpy[3];
        #     double velocity_xyz[3];
        #     double position_xyz[3];
        # };
        return struct.pack(self.fdm_struct_format,
                           self.robot.getTime(),
                           g[0], -g[1], -g[2],
                           a[0], -a[1], -a[2],
                           i[0], -i[1], -i[2],
                           gps_vel[0], -gps_vel[1], -gps_vel[2],
                           gps_pos[0], -gps_pos[1], -gps_pos[2])

    def _handle_controls(self, command: tuple):
        """Set the motor speeds based on the SITL command

        Args:
            command (tuple): tuple of motor speeds 0.0-1.0 where -1.0 is unused
        """

        # get only the number of motors we have
        command_motors = command[:len(self._motors)]
        if -1 in command_motors:
            print(f"Warning: SITL provided {command.index(-1)} motors "
                  f"but model specifies {len(self._motors)} (I{self._instance})")

        # scale commands to -1.0-1.0 if the motors are bidirectional (ex rover wheels)
        if self._bidirectional_motors:
            command_motors = [v*2-1 for v in command_motors]

        # linearize propeller thrust for `MOT_THST_EXPO=0`
        if self._uses_propellers:
            # `Thrust = thrust_constant * |omega| * omega` (ref https://cyberbotics.com/doc/reference/propeller)
            # if we set `omega = sqrt(input_thottle)` then `Thrust = thrust_constant * input_thottle`
            linearized_motor_commands = [np.sqrt(np.abs(v))*np.sign(v) for v in command_motors]

        # reverse motors if desired
        if self._reversed_motors:
            for m in self._reversed_motors:
                linearized_motor_commands[m-1] *= -1

        # set velocities of the motors in Webots
        for i, m in enumerate(self._motors):
            m.setVelocity(linearized_motor_commands[i] * min(m.getMaxVelocity(), self.motor_velocity_cap))

    def _handle_image_stream(self, camera: Union[Camera, RangeFinder], publisher: rclpy.publisher.Publisher):
        time.sleep(2)

        """Stream images over a ROS2 topic.

        Args:
            camera (Camera or RangeFinder): the camera to get images from
            port (int): port to send images over
        """
        # get camera info
        # https://cyberbotics.com/doc/reference/camera
        cam_sample_period = camera.getSamplingPeriod()
        cam_width = camera.getWidth()
        cam_height = camera.getHeight()
        self.node.get_logger().info(
            f"ROS2 stream started for camera '{camera.getName()}' "
            f"({cam_width}x{cam_height} @ {1000/cam_sample_period if cam_sample_period > 0 else 0:.2f}fps)"
        )

        while self._webots_connected:
            start_time = self.robot.getTime()
            
            # get image
            img_data = camera.getImage()
            if img_data:
                # convert and publish
                img_np = np.frombuffer(img_data, np.uint8).reshape((cam_height, cam_width, 4))
                publisher.publish(self.br.cv2_to_imgmsg(img_np[:, :, :3], "rgb8"))
            
            # wait for next frame
            elapsed_time = self.robot.getTime() - start_time
            sleep_duration = (cam_sample_period / 1000.0) - elapsed_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
    
    def set_gimbal_angle(self, angle_radians: float):
        """Sets the position of the gimbal servo.

        Args:
            angle_radians (float): The desired angle in radians.
        """
        if hasattr(self, 'gimbal_servo'):
            self.gimbal_servo.setPosition(angle_radians)
        else:
            self.node.get_logger().warn("Gimbal servo not initialized.")

    def get_camera_gray_image(self) -> np.ndarray:
        """Get the grayscale image from the camera as a numpy array of bytes"""
        img = self.get_camera_image()
        img_gray = np.average(img, axis=2).astype(np.uint8)
        return img_gray

    def get_camera_image(self) -> np.ndarray:
        """Get the RGB image from the camera as a numpy array of bytes"""
        img = self.camera.getImage()
        img = np.frombuffer(img, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
        return img[:, :, :3] # RGB only, no Alpha

    def get_gimbal_camera_image(self) -> np.ndarray:
        """Get the RGB image from the camera as a numpy array of bytes"""
        img = self.gimbal_camera.getImage()
        img = np.frombuffer(img, np.uint8).reshape((self.gimbal_camera.getHeight(), self.gimbal_camera.getWidth(), 4))
        return img[:, :, :3] # RGB only, no Alpha

    def get_rangefinder_image(self, use_int16: bool = False) -> np.ndarray:
        """Get the rangefinder depth image as a numpy array of int8 or int16"""\

        # get range image size
        height = self.rangefinder.getHeight()
        width = self.rangefinder.getWidth()

        # get image, and convert raw ctypes array to numpy array
        # https://cyberbotics.com/doc/reference/rangefinder
        image_c_ptr = self.rangefinder.getRangeImage(data_type="buffer")
        img_arr = np.ctypeslib.as_array(image_c_ptr, (width*height,))
        img_floats = img_arr.reshape((height, width))

        # normalize and set unknown values to max range
        range_range = self.rangefinder.getMaxRange() - self.rangefinder.getMinRange()
        img_normalized = (img_floats - self.rangefinder.getMinRange()) / range_range
        img_normalized[img_normalized == float('inf')] = 1

        # convert to int8 or int16, allowing for the option of higher precision if desired
        if use_int16:
            img = (img_normalized * 65535).astype(np.uint16)
        else:
            img = (img_normalized * 255).astype(np.uint8)

        return img

    def stop_motors(self):
        """Set all motors to zero velocity"""
        for m in self._motors:
            m.setPosition(float('inf'))
            m.setVelocity(0)

    def webots_connected(self) -> bool:
        """Check if Webots client is connected"""
        return self._webots_connected