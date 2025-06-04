import threading
import time
import asyncio

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from drone_comunication.drone_controller import DroneController
from drone_interfaces.msg import ArucoMarkers
from drone_interfaces.srv import PostLog

class WebLogger(Node):
    def __init__(self):
        super().__init__('web_logger')
        self.client = self.create_client(PostLog, 'post_log_to_web')

    def log(self, message, level="info"):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service post_log_to_server...')

        request = PostLog.Request()
        request.message = message
        request.level = level

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().result:
                self.get_logger().info('Successfully posted web log')
            else:
                self.get_logger().warn(f"Failed to post log")
        else:
            self.get_logger().error('Service call failed')
            

class ArucoMissionNode(DroneController):
    def __init__(self):
        super().__init__()

        # Marker reception
        self._marker_pose = None
        self._marker_received = False
        self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self._marker_callback,
            10
        )
        # Delay mission start by 1s to allow connections
        # self._mission_timer = self.create_timer(1.0, self._mission_entry)
        # self.web_logger = WebLogger()
        self.web_logger.log("Aruco mission node started", "info")
        self._mission_started = False
        self.i = 0

    def _marker_callback(self, msg: ArucoMarkers):
        if msg.marker_ids and not self._marker_received:
            self._marker_pose = msg.poses[0]
            marker_id = msg.marker_ids[0]
            self.get_logger().info(f'Detected ArUco marker {marker_id}')
            self.get_logger().info(f'Marker pose: x={self._marker_pose.position.x:.2f}, y={self._marker_pose.position.y:.2f}, z={self._marker_pose.position.z:.2f}')
            print(f"elo {self.i}")
            self.i += 1
            self._marker_received = True

    def start_mission(self):
        if self._mission_started:
            return
        self._mission_started = True
        self.get_logger().info('Mission starting...')

        # 1. Record home position
        home = self.get_gps()
        if not home:
            self.get_logger().error('Unable to fetch home GPS. Aborting.')
            return
        self.get_logger().info(f'Home GPS N={home[0]:.2f}, E={home[1]:.2f}, D={home[2]:.2f}')

        # 2. Arm and take off to 5m
        if not self.arm():
            self.get_logger().error('Arm failed. Aborting.')
            return
        if not self.takeoff(5.0):
            self.get_logger().error('Takeoff failed. Aborting.')
            return

        # 3. Wait for marker detection
        self.get_logger().info('Waiting for ArUco marker detection...')
        while rclpy.ok() and not self._marker_received:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self._marker_pose:
            self.get_logger().error('Marker never detected. Aborting.')
            return

        # 4. Navigate to marker (horizontal only)
        # idk why but speed change works only after the first goto
        self.send_goto_relative(0.0, 0.0, 0.0)
        if not self.set_speed(0.3):
            self.get_logger().error('Failed to set speed. Aborting.')
            return
        pos = self._marker_pose.position
        self.get_logger().info(f'Navigating to marker: x={pos.x:.2f}, y={pos.y:.2f}')
        if not self.send_goto_relative(-pos.y, pos.x, 0.0):
            self.get_logger().error('Goto marker failed. Aborting.')
            rclpy.shutdown(); return

        # 5. Land on marker
        self.get_logger().info('Landing on marker...')
        self.land()
        return

        # 6. Take off again
        if not self.arm():
            self.get_logger().error('Second Arm failed. Aborting.')
            return
        self.get_logger().info('Re-taking off to 5m')
        if not self.takeoff(5.0):
            self.get_logger().error('Second takeoff failed. Aborting.')
            return

        # 7. Return home
        current = self.get_gps()
        if current:
            dx = home[0] - current[0]
            dy = home[1] - current[1]
            self.get_logger().info(f'Returning home: dx={dx:.2f}, dy={dy:.2f}')
            self.send_goto_relative(dx, dy, 0.0)
        else:
            self.get_logger().warn('Unable to fetch current GPS. Skipping return move.')

        # 8. Final landing
        self.get_logger().info('Landing at home...')
        self.land()
        self.get_logger().info('Mission complete.')

    async def mission_async(self):
        if self._mission_task is not None:
            return  # Mission already running
        self.get_logger().info('Mission starting...')
        
        home = self.get_gps()
        if not home:
            self.get_logger().error('Unable to fetch home GPS. Aborting.')
            return
        self.get_logger().info(f'Home GPS: {home}')

        if not self.arm():
            self.get_logger().error('Arm failed.')
            return
        if not self.takeoff(5.0):
            self.get_logger().error('Takeoff failed.')
            return

        # ✨ Here we "pause" for 5 seconds without blocking
        await self.pause(5.0)

        # Wait for marker
        self.get_logger().info('Waiting for marker...')
        while not self._marker_received:
            await asyncio.sleep(0.1)  # Yield control to executor

        self.get_logger().info('Marker detected, proceeding!')

        # continue mission...

    async def pause(self, seconds: float):
        """Non-blocking sleep helper."""
        await asyncio.sleep(seconds)

    def start_mission_async(self):
        """Launch the mission as a background task."""
        self._mission_task = asyncio.create_task(self.mission_async())

    def _marker_callback_2(self, msg):
        if msg.marker_ids:
            self._marker_received = True
            # handle marker


def main(args=None):
    rclpy.init(args=args)

    node = ArucoMissionNode()

    # # Spin node in a separate thread to get subscription callbacks
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # node.start_mission()
    node.start_video()

    node.arm()
    node.takeoff(5.0)
    node.send_goto_relative(0.0, 0.0, 0.0)
    node.set_speed(0.5)
    
    node.send_goto_global(50.27228213642658, 18.672786340141236, 10)
    
    node.send_goto_global(50.27217242065779, 18.672783657932438, 10)
    
    node.send_goto_global(50.27217927790075, 18.67259590331664, 10)
    
    node.send_goto_global(50.27229585087989, 18.67277292909725, 10)

    time.sleep(3)

    node.rtl()
    node.stop_video()

    # Shut down the executor and node
    # This is important to ensure all threads are cleaned up properly
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()
