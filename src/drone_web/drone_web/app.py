import threading
import time
from flask import Flask, request, jsonify

import rclpy
from rclpy.executors import MultiThreadedExecutor

from drone_comunication.drone_controller import DroneController  # adjust import if needed

app = Flask(__name__)
drone = None

@app.route('/takeoff', methods=['POST'])
def takeoff():
    data = request.get_json()
    altitude = data.get('altitude', 2.0)  # default to 2m if not provided

    success = drone.arm() and drone.takeoff(altitude)
    return jsonify({'success': success}), (200 if success else 500)

@app.route('/goto_relative', methods=['POST'])
def goto_relative():
    data = request.get_json()
    north = data.get('north', 0.0)
    east = data.get('east', 0.0)
    down = data.get('down', 0.0)

    success = drone.send_goto_relative(north, east, down)
    return jsonify({'success': success}), (200 if success else 500)

@app.route('/telemetry', methods=['GET'])
def telemetry():
    gps = drone.get_gps()
    yaw = drone.get_yaw()
    if gps is None:
        return jsonify({'error': 'Failed to get telemetry'}), 500

    # Get additional telemetry values from the drone object
    battery_percentage = getattr(drone, 'battery_percentage', None)
    battery_voltage = getattr(drone, 'battery_voltage', None)
    battery_current = getattr(drone, 'battery_current', None)
    lat = getattr(drone, 'lat', None)
    lon = getattr(drone, 'lon', None)
    alt = getattr(drone, 'alt', None)
    flight_mode = getattr(drone, 'flight_mode', None)

    return jsonify({
        'north': gps[0],
        'east': gps[1],
        'down': gps[2],
        'yaw': yaw,
        'battery_percentage': battery_percentage,
        'battery_voltage': battery_voltage,
        'battery_current': battery_current,
        'lat': lat,
        'lon': lon,
        'alt': alt,
        'flight_mode': flight_mode
    })

def ros_spin_thread():
    rclpy.init()
    global drone
    drone = DroneController()
    executor = MultiThreadedExecutor()
    executor.add_node(drone)
    executor.spin()
    drone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    thread = threading.Thread(target=ros_spin_thread, daemon=True)
    thread.start()
    time.sleep(2)  # Give time to initialize ROS 2 node
    app.run(host='0.0.0.0', port=5000)
