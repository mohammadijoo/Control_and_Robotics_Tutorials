import numpy as np

def sign_smooth(v, eps=1e-3):
    """Smooth approximation of sign(v) using tanh."""
    return np.tanh(v / eps)

def coulomb_friction(qdot, Fc, eps=1e-3):
    """
    Coulomb friction: tau_C = Fc * sgn(qdot).
    qdot: float or ndarray
    Fc:   scalar or ndarray (same shape)
    """
    return Fc * sign_smooth(qdot, eps=eps)

def viscous_friction(qdot, b):
    """
    Viscous friction: tau_V = b * qdot.
    """
    return b * qdot

def stribeck_friction(qdot, Fc, Fs, vs, alpha=1.0, b=0.0, eps=1e-3):
    """
    Stribeck + viscous friction:
      tau = (Fc + (Fs - Fc)*exp(-(abs(qdot)/vs)**alpha))*sgn(qdot) + b*qdot
    """
    qdot = np.asarray(qdot)
    sgn = sign_smooth(qdot, eps=eps)
    phi = np.exp(- (np.abs(qdot) / vs)**alpha)
    return (Fc + (Fs - Fc) * phi) * sgn + b * qdot

def joint_dynamics_step(q, qdot, tau_act, dt,
                        I=0.1, model="stribeck",
                        params=None):
    """
    One explicit Euler integration step for a 1-DOF joint with friction.
    """
    if params is None:
        params = {}

    if model == "coulomb":
        Fc = params.get("Fc", 0.2)
        tau_f = coulomb_friction(qdot, Fc)
    elif model == "viscous":
        b = params.get("b", 0.01)
        tau_f = viscous_friction(qdot, b)
    elif model == "stribeck":
        Fc = params.get("Fc", 0.2)
        Fs = params.get("Fs", 0.3)
        vs = params.get("vs", 0.05)
        alpha = params.get("alpha", 1.0)
        b = params.get("b", 0.01)
        tau_f = stribeck_friction(qdot, Fc, Fs, vs, alpha, b)
    else:
        raise ValueError("Unknown model")

    qddot = (tau_act - tau_f) / I
    qdot_next = qdot + dt * qddot
    q_next = q + dt * qdot_next
    return q_next, qdot_next, qddot

if __name__ == "__main__":
    # Simple simulation: constant torque, compare models
    dt = 1e-3
    T = 2.0
    N = int(T / dt)

    q = 0.0
    qdot = 0.0
    tau_act = 0.4
    traj = []

    params = {"Fc": 0.2, "Fs": 0.3, "vs": 0.05, "alpha": 1.0, "b": 0.01}
    for k in range(N):
        q, qdot, qddot = joint_dynamics_step(
            q, qdot, tau_act, dt,
            I=0.1,
            model="stribeck",
            params=params
        )
        traj.append((k*dt, q, qdot, qddot))

    traj = np.array(traj)
    # traj[:,1] = q(t), traj[:,2] = qdot(t)
    # Plotting can be done with matplotlib in a separate script or notebook.
      
