from dronekit import connect, VehicleMode, LocationLocal, LocationGlobalRelative


connection_string = 'tcp:127.0.0.1:5762'

vehicle = connect(connection_string, wait_ready=False)

vehicle.mode=VehicleMode("GUIDED")
vehicle.armed = True
vehicle.simple_takeoff(2)