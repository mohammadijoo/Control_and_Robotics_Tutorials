
import numpy as np

class OneDOFImpactSystem:
    def __init__(self, m=1.0, g=9.81, e=0.0):
        self.m = m      # mass
        self.g = g      # gravity
        self.e = e      # restitution (0 inelastic, 1 elastic)

    def dynamics(self, t, x, tau):
        """
        Continuous dynamics between impacts.
        x = [q, v]. tau is control torque (force).
        """
        q, v = x
        # mass * acceleration = tau - m*g
        a = (tau - self.m * self.g) / self.m
        return np.array([v, a])

    def impact_event(self, t, x):
        """
        Guard function: impact when q = 0 and v < 0.
        We only encode the surface condition here; direction handled separately.
        """
        q, _ = x
        return q  # zero when contact with the stop

    def handle_impact(self, x):
        """
        Apply Newton impact law with coefficient of restitution e.
        """
        q, v = x
        if q == 0.0 and v < 0.0:
            v_plus = -self.e * v  # normal velocity reversed and scaled
            return np.array([q, v_plus])
        else:
            return x

def pd_controller(q, v, q_des, v_des, kp=50.0, kd=10.0, g=9.81, m=1.0):
    """
    Simple PD with gravity compensation for a vertical 1-DOF system.
    """
    # gravity compensation
    tau_g = m * g
    # PD term
    tau_pd = -kp * (q - q_des) - kd * (v - v_des)
    return tau_g + tau_pd

def simulate(system, x0, t0, tf, dt, q_des):
    t = t0
    x = np.array(x0, dtype=float)
    ts = [t]
    xs = [x.copy()]

    while t < tf:
        q, v = x
        # Mode: free or resting on the stop
        if q > 0.0 or v > 0.0:
            # free motion (above or separating from the stop)
            tau = pd_controller(q, v, q_des, 0.0, g=system.g, m=system.m)
        else:
            # we are on the stop and not separating (q == 0 and v == 0)
            # hold position with PD to avoid drift
            tau = 0.0

        # integrate one Euler step (for simplicity)
        dx = system.dynamics(t, x, tau)
        x_new = x + dt * dx

        # detect surface crossing q = 0
        if x[0] > 0.0 and x_new[0] <= 0.0:
            # approximate impact at intermediate time
            # move to surface
            alpha = x[0] / (x[0] - x_new[0] + 1e-12)
            x_impact = x + alpha * (x_new - x)

            # apply impact law
            x_after = system.handle_impact(x_impact)

            # continue with remaining time in the step if needed
            remaining = (1.0 - alpha) * dt
            dx_after = system.dynamics(t, x_after, tau)
            x_new = x_after + remaining * dx_after

        t += dt
        x = x_new
        ts.append(t)
        xs.append(x.copy())

    return np.array(ts), np.stack(xs, axis=0)

if __name__ == "__main__":
    sys = OneDOFImpactSystem(m=1.0, g=9.81, e=0.2)
    x0 = [0.5, -1.0]  # start above the stop, moving downward
    t, x = simulate(sys, x0, t0=0.0, tf=3.0, dt=1e-3, q_des=0.2)
    # Here you can plot q(t) and v(t) to inspect impact behaviour.
