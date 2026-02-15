import numpy as np

class FirstOrderLowPass:
    def __init__(self, tau, x0=0.0):
        """
        tau: time constant (seconds)
        x0: initial state
        """
        self.tau = float(tau)
        self.x = float(x0)

    def update(self, u, dt):
        """
        Forward-Euler integration of tau * x_dot + x = u.
        u : current input sample
        dt: sampling period
        """
        a = -1.0 / self.tau
        b = 1.0 / self.tau
        self.x = self.x + dt * (a * self.x + b * u)
        return self.x

# Example: filtering a noisy joint position signal
if __name__ == "__main__":
    tau = 0.05  # 50 ms time constant
    dt = 0.001  # 1 kHz loop
    lp = FirstOrderLowPass(tau)

    # Simulate 1 s of data for a robot joint moving with a ramp + noise
    t = np.arange(0.0, 1.0, dt)
    true_theta = 0.5 * t  # rad
    noise = 0.02 * np.random.randn(len(t))
    y_meas = true_theta + noise

    y_filt = np.zeros_like(y_meas)
    for k in range(len(t)):
        y_filt[k] = lp.update(y_meas[k], dt)

    # Inspect Bode magnitude of the underlying analog prototype using python-control
    try:
        import control  # pip install control
        s = control.TransferFunction.s
        wc = 1.0 / tau
        F_lp = wc / (s + wc)
        mag, phase, omega = control.bode(F_lp, dB=True, omega_limits=(1.0, 1e3), Plot=False)
        # At this point you could integrate with robotics toolboxes such as
        # roboticstoolbox-python to validate the filter on full robot models.
    except ImportError:
        print("python-control not installed; skipping Bode computation.")
