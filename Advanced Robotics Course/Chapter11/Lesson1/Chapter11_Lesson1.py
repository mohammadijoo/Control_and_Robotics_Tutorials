import numpy as np

class DemonstrationLogger:
    """
    Collects a single demonstration as arrays of times, states x_t and controls u_t.
    Suitable for teleop (u_t from device) or kinesthetic (u_t can be None).
    """
    def __init__(self):
        self.times = []
        self.x_list = []
        self.u_list = []

    def append_sample(self, t, x, u=None):
        """Append one sample."""
        self.times.append(float(t))
        self.x_list.append(np.asarray(x, dtype=float))
        if u is None:
            self.u_list.append(None)
        else:
            self.u_list.append(np.asarray(u, dtype=float))

    def to_arrays(self):
        """Return stacked arrays (NaNs for missing controls)."""
        t = np.asarray(self.times, dtype=float)
        X = np.vstack(self.x_list)
        if any(u is None for u in self.u_list):
            # Use NaNs for missing controls (e.g., kinesthetic demos)
            U = np.full((len(self.u_list), X.shape[1]), np.nan, dtype=float)
            for i, u in enumerate(self.u_list):
                if u is not None:
                    U[i, :] = u
        else:
            U = np.vstack(self.u_list)
        return t, X, U


def resample_demonstration(t, X, U, dt):
    """
    Resample a demonstration (t, X, U) onto a uniform grid with step dt.
    Linear interp for states; zero-order hold for controls (if available).
    """
    t = np.asarray(t, dtype=float)
    X = np.asarray(X, dtype=float)
    U = np.asarray(U, dtype=float)

    t_min, t_max = float(t[0]), float(t[-1])
    t_grid = np.arange(t_min, t_max + 0.5 * dt, dt)

    # Linear interpolation for states
    X_resampled = np.empty((len(t_grid), X.shape[1]), dtype=float)
    for j in range(X.shape[1]):
        X_resampled[:, j] = np.interp(t_grid, t, X[:, j])

    # Zero-order hold for controls (ignore all-NaN case)
    if np.all(np.isnan(U)):
        U_resampled = np.full_like(X_resampled, np.nan, dtype=float)
    else:
        U_resampled = np.empty_like(X_resampled)
        for i, tg in enumerate(t_grid):
            # index of last time <= tg
            idx = int(np.max(np.where(t <= tg)))
            U_resampled[i, :] = U[idx, :]

    return t_grid, X_resampled, U_resampled


# Example usage:
if __name__ == "__main__":
    logger = DemonstrationLogger()

    # This part would typically be inside a ROS callback that reads robot state and teleop device
    t = 0.0
    dt = 0.01
    for k in range(1000):
        # Example: circular motion in task space (x, y)
        x = np.array([np.cos(0.5 * t), np.sin(0.5 * t)])
        u = np.array([0.5 * -np.sin(0.5 * t), 0.5 * np.cos(0.5 * t)])  # nominal velocity
        logger.append_sample(t, x, u)
        t += dt

    t_raw, X_raw, U_raw = logger.to_arrays()
    t_grid, X_res, U_res = resample_demonstration(t_raw, X_raw, U_raw, dt=0.02)

    print("Original samples:", len(t_raw))
    print("Resampled samples:", len(t_grid))
      
