
from abc import ABC, abstractmethod
import numpy as np

# -------------------------------
# Abstract controller interface
# -------------------------------

class Controller(ABC):
    @abstractmethod
    def compute_tau(self, t, q, dq, ref):
        """
        Parameters
        ----------
        t   : float
        q   : np.ndarray, shape (n,)
        dq  : np.ndarray, shape (n,)
        ref : dict, e.g. {"qd": qd, "dqd": dqd, "xdd": xdd, ...}

        Returns
        -------
        tau : np.ndarray, shape (n,)
        """
        pass

# -------------------------------
# Joint-space PD controller
# -------------------------------

class JointPD(Controller):
    def __init__(self, Kp, Kd):
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)

    def compute_tau(self, t, q, dq, ref):
        qd = ref["qd"]
        dqd = ref.get("dqd", np.zeros_like(q))
        e = qd - q
        de = dqd - dq
        return self.Kp @ e + self.Kd @ de

# -------------------------------
# Computed-torque controller
# -------------------------------

class ComputedTorque(Controller):
    def __init__(self, model, Kp, Kd):
        """
        model must provide:
          M(q), C(q, dq), g(q)
        """
        self.model = model
        self.Kp = np.diag(Kp)
        self.Kd = np.diag(Kd)

    def compute_tau(self, t, q, dq, ref):
        qd = ref["qd"]
        dqd = ref.get("dqd", np.zeros_like(q))
        ddqd = ref.get("ddqd", np.zeros_like(q))

        e = qd - q
        de = dqd - dq
        v = ddqd + self.Kd @ de + self.Kp @ e

        M = self.model.M(q)
        C = self.model.C(q, dq)
        g = self.model.g(q)

        # tau = M(q) v + C(q, dq) dq + g(q)
        tau = M @ v + C @ dq + g
        return tau

# -------------------------------
# Safety filter (CBF-inspired QP)
# Here we use a simple projection placeholder
# -------------------------------

class SafetyFilter:
    def __init__(self, umin, umax):
        self.umin = np.asarray(umin)
        self.umax = np.asarray(umax)

    def filter(self, x, u_nom):
        # Simple box projection; replace by QP with CBF constraints in real project
        return np.clip(u_nom, self.umin, self.umax)

# -------------------------------
# Architecture wrapper
# -------------------------------

class Architecture:
    def __init__(self, controller: Controller,
                 safety: SafetyFilter | None = None):
        self.controller = controller
        self.safety = safety

    def compute_tau(self, t, x, ref):
        q, dq = x["q"], x["dq"]
        tau_nom = self.controller.compute_tau(t, q, dq, ref)
        if self.safety is not None:
            return self.safety.filter(x, tau_nom)
        return tau_nom

# -------------------------------
# Architecture selection
# -------------------------------

def select_architecture(spec, model):
    """
    spec: dict with keys like
        {"contact": False,
         "constraints": True,
         "uncertainty": "medium",
         "compute_budget": "moderate"}
    """
    n = model.nq

    if not spec["contact"]:
        if spec["uncertainty"] == "low":
            # inner joint servo assumed; outer PD is enough
            ctrl = JointPD(Kp=5.0 * np.ones(n), Kd=1.0 * np.ones(n))
        else:
            # use computed-torque for better robustness
            ctrl = ComputedTorque(model, Kp=10.0 * np.ones(n), Kd=3.0 * np.ones(n))
    else:
        # For contact-sensitive tasks, you might choose an impedance control
        # implementation here (not shown for brevity).
        ctrl = JointPD(Kp=3.0 * np.ones(n), Kd=0.8 * np.ones(n))

    if spec.get("constraints", False):
        safety = SafetyFilter(umin=-20.0 * np.ones(n), umax=20.0 * np.ones(n))
    else:
        safety = None

    return Architecture(ctrl, safety)
