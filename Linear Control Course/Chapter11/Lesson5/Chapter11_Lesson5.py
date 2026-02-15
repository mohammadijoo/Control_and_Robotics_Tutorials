from dataclasses import dataclass
from enum import Enum
import math

class ZNMode(str, Enum):
    P = "P"
    PI = "PI"
    PID = "PID"

def zn_closed_loop(Ku: float, Tu: float, mode: ZNMode = ZNMode.PID):
    """
    Ziegler-Nichols closed-loop tuning.
    Returns (Kp, Ki, Kd) for the parallel PID form.
    """
    if mode == ZNMode.P:
        Kp = 0.5 * Ku
        Ti = math.inf
        Td = 0.0
    elif mode == ZNMode.PI:
        Kp = 0.45 * Ku
        Ti = Tu / 1.2
        Td = 0.0
    else:  # PID
        Kp = 0.60 * Ku
        Ti = Tu / 2.0
        Td = Tu / 8.0

    if math.isinf(Ti):
        Ki = 0.0
    else:
        Ki = Kp / Ti
    Kd = Kp * Td
    return Kp, Ki, Kd

@dataclass
class PIDController:
    Kp: float
    Ki: float
    Kd: float
    dt: float  # sampling period
    u_min: float = -float("inf")
    u_max: float = float("inf")

    # internal state
    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error: float) -> float:
        """
        One control step. For a robot joint, 'error' can be the difference
        between desired and measured joint position.
        """
        # Integrate error
        self.integral += error * self.dt

        # Derivative of error
        derivative = (error - self.prev_error) / self.dt

        # PID law
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Saturation with simple integral anti-windup
        if u > self.u_max:
            u = self.u_max
            # anti-windup: do not integrate further when saturated high
            self.integral -= error * self.dt
        elif u < self.u_min:
            u = self.u_min
            # anti-windup: do not integrate further when saturated low
            self.integral -= error * self.dt

        self.prev_error = error
        return u

# Example usage:
if __name__ == "__main__":
    Ku = 6.0   # measured ultimate gain
    Tu = 1.2   # measured ultimate period (seconds)
    Kp, Ki, Kd = zn_closed_loop(Ku, Tu, ZNMode.PID)
    print("ZN PID gains:", Kp, Ki, Kd)

    pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd, dt=0.001,
                        u_min=-10.0, u_max=10.0)

    # In a robotics control loop (e.g., with ROS),
    # pid.step(error) would be called every dt seconds.
