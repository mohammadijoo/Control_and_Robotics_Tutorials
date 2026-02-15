import numpy as np
from dataclasses import dataclass
from typing import Protocol, Any, Dict

# --- Core protocols for modules ---

class Perception(Protocol):
    def reset(self) -> None: ...
    def update(self, sensor: np.ndarray) -> np.ndarray: ...

class Planner(Protocol):
    def plan(self, belief: np.ndarray, goal: np.ndarray) -> Dict[str, Any]: ...

class Controller(Protocol):
    def reset(self) -> None: ...
    def compute_control(self, x_ref: np.ndarray, belief: np.ndarray) -> np.ndarray: ...

class Simulator(Protocol):
    def reset(self, omega: Dict[str, Any]) -> Dict[str, Any]: ...
    def step(self, u: np.ndarray) -> Dict[str, Any]:
        """
        Returns dict with keys: 'state', 'sensor', 'done', 'collision', 'cost'
        """
        ...

@dataclass
class Metrics:
    J: float
    success: bool
    violation: bool

# --- Single rollout ---

def run_episode(sim: Simulator,
                perc: Perception,
                planner: Planner,
                ctrl: Controller,
                omega: Dict[str, Any],
                horizon: int) -> Metrics:
    sim_state = sim.reset(omega)
    perc.reset()
    ctrl.reset()

    J = 0.0
    violated = False
    success = False

    for t in range(horizon):
        sensor = sim_state["sensor"]
        belief = perc.update(sensor)
        goal = sim_state["goal"]

        plan = planner.plan(belief, goal)
        x_ref = plan["x_ref"]  # e.g., reference state for this step

        u = ctrl.compute_control(x_ref, belief)
        sim_state = sim.step(u)

        J += float(sim_state["cost"])
        if sim_state.get("collision", False):
            violated = True

        if sim_state.get("done", False):
            success = not violated and sim_state.get("task_success", True)
            break

    # If we never reached 'done', treat as failure
    return Metrics(J=J, success=success, violation=violated)

# --- Monte Carlo evaluation ---

def evaluate_stack(sim: Simulator,
                   perc: Perception,
                   planner: Planner,
                   ctrl: Controller,
                   N: int,
                   horizon: int,
                   rng: np.random.Generator) -> Dict[str, float]:
    Js = []
    successes = []
    violations = []

    for i in range(N):
        omega = {
            "init_state": rng.normal(size=sim.state_dim),
            "noise_seed": int(rng.integers(0, 2**31 - 1))
        }
        metrics = run_episode(sim, perc, planner, ctrl, omega, horizon)
        Js.append(metrics.J)
        successes.append(1 if metrics.success else 0)
        violations.append(1 if metrics.violation else 0)

    Js = np.array(Js, dtype=float)
    successes = np.array(successes, dtype=float)
    violations = np.array(violations, dtype=float)

    results = {
        "J_hat": float(Js.mean()),
        "J_std_over_sqrtN": float(Js.std(ddof=1) / np.sqrt(N)),
        "p_succ_hat": float(successes.mean()),
        "p_succ_std_over_sqrtN": float(successes.std(ddof=1) / np.sqrt(N)),
        "p_viol_hat": float(violations.mean()),
        "p_viol_std_over_sqrtN": float(violations.std(ddof=1) / np.sqrt(N)),
    }
    return results


if __name__ == "__main__":
    rng = np.random.default_rng(0)
    # Instantiate your concrete modules here.
    # sim = ...
    # perc = ...
    # planner = ...
    # ctrl = ...
    # results = evaluate_stack(sim, perc, planner, ctrl, N=100, horizon=200, rng=rng)
    # print(results)
      
