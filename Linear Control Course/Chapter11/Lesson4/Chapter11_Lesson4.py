import math
from typing import Optional

class PIDController:
    """
    Parallel PID controller for robotic actuators.

    Parameters
    ----------
    Kp, Ki, Kd : float
        Proportional, integral, derivative gains.
    dt : float
        Sampling period (seconds).
    u_min, u_max : Optional[float]
        Optional actuator limits for saturation.
    """
    def __init__(self, Kp: float, Ki: float, Kd: float,
                 dt: float,
                 u_min: Optional[float] = None,
                 u_max: Optional[float] = None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.u_min = u_min
        self.u_max = u_max

        self.integral = 0.0
        self.e_prev = 0.0
        self.first_call = True

    def reset(self) -> None:
        self.integral = 0.0
        self.e_prev = 0.0
        self.first_call = True

    def update(self, r: float, y: float) -> float:
        """
        Compute control u[k] given reference r and measurement y.
        """
        e = r - y

        # Integral update (forward Euler)
        self.integral += e * self.dt

        # Derivative term (backward difference)
        if self.first_call:
            d = 0.0
            self.first_call = False
        else:
            d = (e - self.e_prev) / self.dt

        u = self.Kp * e + self.Ki * self.integral + self.Kd * d

        # Simple saturation and anti-windup
        if self.u_min is not None and u < self.u_min:
            u = self.u_min
            # Optional: prevent further integral growth
            # self.integral -= e * self.dt
        if self.u_max is not None and u > self.u_max:
            u = self.u_max
            # Optional: prevent further integral growth
            # self.integral -= e * self.dt

        self.e_prev = e
        return u

# Example usage inside a robotic joint control loop:
# pid = PIDController(Kp=10.0, Ki=5.0, Kd=0.2, dt=0.001, u_min=-12.0, u_max=12.0)
# while running:
#     q_ref = desired_joint_angle()
#     q_meas = read_encoder()
#     u_cmd = pid.update(q_ref, q_meas)
#     send_torque_command(u_cmd)
