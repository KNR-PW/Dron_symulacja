import numpy as np

class KalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=1.0):
        # State: [x, y, vx, vy] (Relative position in Yaw-Stabilized Frame)
        self.x = np.zeros(4)
        self.P = np.eye(4)
        
        # Process Noise (Tune this: higher = trust model less)
        self.Q = np.eye(4) * process_noise
        
        # Measurement Noise (Tune this: higher = trust measurement less)
        self.R = np.eye(2) * measurement_noise
        
        # Measurement Matrix (We measure x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        self.initialized = False

    def predict(self, dt):
        if not self.initialized: 
            return np.zeros(2)
        
        # Constant Velocity Model
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        return self.x[:2]

    def update(self, z):
        # z: measurement [x, y]
        if not self.initialized:
            self.x = np.array([z[0], z[1], 0, 0])
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

class KalmanFilterAware:
    """
    Kalman Filter that accounts for Drone's Ego-Motion (Velocity).
    State: [x_rel, y_rel, vx_abs, vy_abs]
    Estimates absolute velocity of the target to avoid lag during drone acceleration.
    """
    def __init__(self, process_noise=10.0, measurement_noise=0.1):
        # State: [x_rel, y_rel, vx_ABS, vy_ABS]
        self.x = np.zeros(4)
        self.P = np.eye(4)
        
        # Process Noise for Absolute Velocity Model
        # We trust the drone's velocity reading, so uncertainty comes mainly from target's acceleration
        self.Q = np.eye(4) * process_noise
        
        # Measurement Noise
        self.R = np.eye(2) * measurement_noise
        
        # Measurement Matrix (We measure relative position)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        self.initialized = False

    def predict(self, dt, drone_velocity_xy=(0,0)):
        """
        dt: time step
        drone_velocity_xy: tuple (vx_drone, vy_drone) in the same frame as tracking
        """
        if not self.initialized: 
            return np.zeros(2)
        
        vx_drone, vy_drone = drone_velocity_xy

        # Prediction Step with Control Input (Drone Motion)
        # x_rel_new = x_rel + (v_abs - v_drone) * dt
        # v_abs_new = v_abs (Constant Velocity assumption for target)
        
        # Transition Matrix (State evolution)
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Control Input Matrix (Effect of drone velocity on state)
        # We subtract drone velocity * dt from position
        B = np.array([
            [-dt, 0],
            [0, -dt],
            [0, 0],
            [0, 0]
        ])
        
        u = np.array([vx_drone, vy_drone])
        
        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x[:2]

    def update(self, z):
        # Standard update
        if not self.initialized:
            # Initialize assuming target is stationary (v_abs = 0) OR better:
            # if we knew drone velocity here, we could init better. 
            # For now standard init:
            self.x = np.array([z[0], z[1], 0, 0])
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
