import numpy as np

class KalmanFilter:
    def __init__(self, process_noise=1.0, measurement_noise=0.1):
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
