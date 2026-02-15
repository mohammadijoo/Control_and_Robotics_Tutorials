import math
from typing import List, Tuple

State = Tuple[float, float, float]   # (x, y, theta)
Control = Tuple[float, float]        # (v, omega)

def unicycle_dynamics(x: State, u: Control) -> State:
    """Continuous-time dynamics xdot = f(x,u)."""
    px, py, th = x
    v, w = u
    return (v * math.cos(th),  # xdot
            v * math.sin(th),  # ydot
            w)                 # thetadot

def step_euler(x: State, u: Control, dt: float) -> State:
    """Explicit Euler integration for one step."""
    dx, dy, dth = unicycle_dynamics(x, u)
    return (x[0] + dt * dx,
            x[1] + dt * dy,
            x[2] + dt * dth)

def within_bounds(u: Control,
                  v_max: float,
                  w_max: float) -> bool:
    v, w = u
    if abs(v) > v_max:
        return False
    if abs(w) > w_max:
        return False
    return True

def is_state_valid(x: State) -> bool:
    """
    Placeholder for collision checking and other state constraints.
    Replace with calls to a collision checker using the robot geometry
    and environment representation.
    """
    # Example: keep robot inside a square workspace
    px, py, _ = x
    if abs(px) > 5.0 or abs(py) > 5.0:
        return False
    return True

def simulate_trajectory(x0: State,
                        controls: List[Control],
                        dt: float,
                        v_max: float,
                        w_max: float) -> Tuple[bool, List[State]]:
    """
    Simulate a sequence of piecewise-constant controls and check feasibility.
    Returns (is_feasible, trajectory).
    """
    traj = [x0]
    x = x0
    for u in controls:
        if not within_bounds(u, v_max, w_max):
            return False, traj
        x_next = step_euler(x, u, dt)
        if not is_state_valid(x_next):
            return False, traj
        traj.append(x_next)
        x = x_next
    return True, traj

if __name__ == "__main__":
    x0 = (0.0, 0.0, 0.0)
    dt = 0.1
    v_max = 1.0
    w_max = 2.0

    # Simple "arc" trajectory: forward with constant v and omega
    controls = [(0.8, 1.0) for _ in range(50)]
    feasible, traj = simulate_trajectory(x0, controls, dt, v_max, w_max)

    print("Feasible:", feasible)
    print("Final state:", traj[-1])
      
