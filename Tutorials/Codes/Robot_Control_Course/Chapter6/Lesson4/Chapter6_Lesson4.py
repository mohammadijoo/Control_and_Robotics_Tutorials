
import numpy as np

class Admittance1D:
    def __init__(self, M_a, D_a, K_a, x0=0.0, dt=0.001):
        self.M_a = float(M_a)
        self.D_a = float(D_a)
        self.K_a = float(K_a)
        self.x0 = float(x0)
        self.dt = float(dt)

        # discrete-time state: z1 = x_r - x0, z2 = xdot_r
        self.z1 = 0.0
        self.z2 = 0.0

        alpha = self.dt / self.M_a
        self.A11 = 1.0 - self.dt * alpha * self.K_a
        self.A12 = self.dt * (1.0 - alpha * self.D_a)
        self.A21 = -alpha * self.K_a
        self.A22 = 1.0 - alpha * self.D_a
        self.B1 = self.dt * alpha
        self.B2 = alpha

    def step(self, F_ext):
        """
        One control step of the admittance model.
        F_ext: measured external force (scalar) at current sample.
        Returns x_r, xdot_r.
        """
        F = float(F_ext)

        z1_next = self.A11 * self.z1 + self.A12 * self.z2 + self.B1 * F
        z2_next = self.A21 * self.z1 + self.A22 * self.z2 + self.B2 * F

        self.z1 = z1_next
        self.z2 = z2_next

        x_r = self.z1 + self.x0
        xdot_r = self.z2
        return x_r, xdot_r


if __name__ == "__main__":
    ctrl = Admittance1D(M_a=3.0, D_a=20.0, K_a=50.0, x0=0.0, dt=0.001)
    T = 2.0
    n_steps = int(T / ctrl.dt)
    F_profile = np.zeros(n_steps)
    # constant push of 10 N after 0.2 s
    start_idx = int(0.2 / ctrl.dt)
    F_profile[start_idx:] = 10.0

    x_log = np.zeros(n_steps)
    for k in range(n_steps):
        x_r, _ = ctrl.step(F_profile[k])
        x_log[k] = x_r

    # x_log can be plotted to inspect the compliant motion
