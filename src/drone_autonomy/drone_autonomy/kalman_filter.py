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
    State: [x_rel, y_rel, vx_ABS, vy_ABS]
    Estimates absolute velocity of the target to avoid lag during drone acceleration.
    """
    def __init__(self, process_noise=1.0, measurement_noise=0.01):
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

class EKF_CTRV:
    """
    Extended Kalman Filter with Constant Turn Rate and Velocity (CTRV) model.
    State: [x_rel, y_rel, v_abs, yaw_abs, yaw_rate_abs]
    
    This model assumes the target moves with constant absolute velocity and turn rate (circular motion),
    while accounting for the drone's own velocity in the relative position update.
    """
    def __init__(self, process_noise=1.0, measurement_noise=0.01):
        # State: [x_rel, y_rel, v, yaw, ω]
        self.x = np.zeros(5)
        self.P = np.eye(5)
        
        # Process Noise Jacobian (Simplified diagonal approximation)
        # We expect noise mainly in velocity change (acceleration) and yaw rate change (angular acceleration)
        self.Q = np.diag([
            process_noise,     # x_rel noise
            process_noise,     # y_rel noise
            process_noise * 5.0,   # v noise (acceleration)
            process_noise * 0.1,   # yaw noise
            process_noise * 5.0    # ω noise (jerk/ang_accel)
        ])
        
        # Measurement Noise
        self.R = np.eye(2) * measurement_noise
        
        # Measurement Matrix (Linear: we observe x_rel, y_rel)
        self.H = np.zeros((2, 5))
        self.H[0, 0] = 1
        self.H[1, 1] = 1
        
        self.initialized = False

    def predict(self, dt, drone_velocity_xy=(0,0)):
        if not self.initialized: 
            return self.x[:2]

        # Unpack state
        # x_rel = self.x[0]
        # y_rel = self.x[1]
        v = self.x[2]
        yaw = self.x[3]
        omega = self.x[4]
        
        vx_drone, vy_drone = drone_velocity_xy

        # --- 1. State Prediction (Non-Linear) ---
        # Avoid division by zero
        if abs(omega) > 0.001:
            dx = (v / omega) * (np.sin(yaw + omega * dt) - np.sin(yaw))
            dy = (v / omega) * (-np.cos(yaw + omega * dt) + np.cos(yaw))
        else:
            # Linear approximation (Constant Velocity)
            dx = v * np.cos(yaw) * dt
            dy = v * np.sin(yaw) * dt

        # Apply Drone Motion Compensation (Relative State Update)
        dx -= vx_drone * dt
        dy -= vy_drone * dt
        
        # Predict State
        self.x[0] += dx
        self.x[1] += dy
        self.x[2] = v                   # Constant Speed assumption
        self.x[3] += omega * dt         # Constant Turn Rate
        self.x[4] = omega               # Constant Turn Rate assumption
        
        # Normalize yaw
        self.x[3] = (self.x[3] + np.pi) % (2 * np.pi) - np.pi

        # --- 2. Jacobian Calculation (F) ---
        # F = df/dx
        F = np.eye(5)
        
        # Derivatives of position w.r.t v, yaw, omega
        if abs(omega) > 0.001:
            sin_term = np.sin(yaw + omega * dt)
            cos_term = np.cos(yaw + omega * dt)
            sin_yaw = np.sin(yaw)
            cos_yaw = np.cos(yaw)
            
            # d(x)/dv
            F[0, 2] = (1/omega) * (sin_term - sin_yaw)
            # d(x)/d(yaw)
            F[0, 3] = (v/omega) * (cos_term - cos_yaw)
            # d(x)/d(omega)
            F[0, 4] = (dt * v / omega) * cos_term - (v / omega**2) * (sin_term - sin_yaw)

            # d(y)/dv
            F[1, 2] = (1/omega) * (-cos_term + cos_yaw)
            # d(y)/d(yaw)
            F[1, 3] = (v/omega) * (sin_term - sin_yaw)
            # d(y)/d(omega)
            F[1, 4] = (dt * v / omega) * sin_term - (v / omega**2) * (-cos_term + cos_yaw)
        else:
            # Linear Limit derivatives
            # d(x)/dv = cos(yaw) * dt
            F[0, 2] = np.cos(yaw) * dt
            # d(x)/d(yaw) = -v * sin(yaw) * dt
            F[0, 3] = -v * np.sin(yaw) * dt
            # d(x)/d(omega) = 0 (approx)
            F[0, 4] = 0

            F[1, 2] = np.sin(yaw) * dt
            F[1, 3] = v * np.cos(yaw) * dt
            F[1, 4] = 0

        # d(yaw)/d(omega)
        F[3, 4] = dt
        
        # --- 3. Covariance Update ---
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x[:2]

    def update(self, z):
        # z: measurement [x_rel, y_rel]
        if not self.initialized:
            # We can't fully initialize 5D state from one 2D point.
            # We need at least velocity.
            # State: [x_rel, y_rel, v, yaw, ω]
            self.x[0] = z[0]
            self.x[1] = z[1]
            self.x[2] = 0  # Assume 0 velocity initially
            self.x[3] = 0  # Unknown heading
            self.x[4] = 0  # Unknown turn rate
            
            # High uncertainty for unobserved states
            self.P[2,2] = 100
            self.P[3,3] = 100
            self.P[4,4] = 100
            
            self.initialized = True
            return

        # Handle simplified initialization for velocity/heading if we have previous pos
        if self.P[2,2] > 50:
            # Crude velocity estimation to converge faster
            # v ~ dist / dt (requires stored time, but filter handles convergence naturally too)
            pass

        # Standard EKF Update (Linear Measurement Model)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ self.H) @ self.P
