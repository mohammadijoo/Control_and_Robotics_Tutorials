import numpy as np

class KalmanFilterCV2D:
    """
    Constant-velocity Kalman filter in 2D.
    State x = [px, py, vx, vy]^T
    Measurement z = [px, py]^T
    """
    def __init__(self, dt=0.1, process_var=1.0, meas_var=4.0):
        self.dt = dt

        # State transition and measurement matrices
        self.F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=float)

        # Process and measurement noise
        q = process_var
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2

        Q1d = np.array([
            [0.25 * dt4, 0.5 * dt3],
            [0.5 * dt3,  dt2]
        ])
        self.Q = q * np.block([
            [Q1d,          np.zeros((2, 2))],
            [np.zeros((2, 2)), Q1d]
        ])

        self.R = meas_var * np.eye(2)

        # Initialize state and covariance
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 1e3

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """z is a length-2 iterable [zx, zy]."""
        z = np.asarray(z, dtype=float).reshape(2, 1)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def position(self):
        return self.x[0, 0], self.x[1, 0]


# Example usage with hypothetical detections (e.g., from segmentation)
kf = KalmanFilterCV2D(dt=0.1)
detections = [(0.0, 0.0), (0.9, 0.1), (2.1, 0.2), (3.0, 0.1)]

for z in detections:
    kf.predict()
    kf.update(z)
    print("Estimated position:", kf.position())
      
