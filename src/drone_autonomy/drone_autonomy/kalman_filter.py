import numpy as np

class KalmanFilter:
    def __init__(self, process_noise=100.0, measurement_noise=0.01):
        # State: [x, y, vx, vy] (Relative position in Yaw-Stabilized Frame)
        self.x = np.zeros(4)
        self.P = np.eye(4)
        
        # Process Noise factor (replaces fixed matrix)
        self.process_noise = process_noise
        self.Q = np.eye(4) * process_noise # Fallback initialization
        
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
        
        # Constant Velocity Model with Discrete White Noise Acceleration Q Matrix
        # This links position uncertainty to velocity uncertainty
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4
        
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Q matrix based on Piecewise White Noise Acceleration Model
        # [ x_noise,  0,        vx_noise, 0       ]
        # [ 0,        y_noise,  0,        vy_noise] ...
        q_pos = dt4 / 4.0
        q_pos_vel = dt3 / 2.0
        q_vel = dt2
        
        self.Q = np.array([
            [q_pos, 0,     q_pos_vel, 0],
            [0,     q_pos, 0,         q_pos_vel],
            [q_pos_vel, 0, q_vel,     0],
            [0,     q_pos_vel, 0,     q_vel]
        ]) * self.process_noise

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
