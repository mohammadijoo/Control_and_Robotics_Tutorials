import numpy as np
import control as ctl  # python-control library

def design_lead_bode(G, PM_des_deg, wc_des, safety_deg=5.0):
    """
    Bode-based design of a phase-lead compensator for SISO plant G(s).

    Parameters
    ----------
    G : control.TransferFunction
        Plant transfer function.
    PM_des_deg : float
        Desired phase margin in degrees.
    wc_des : float
        Desired crossover frequency (rad/s).
    safety_deg : float
        Extra phase margin to compensate for modeling errors.

    Returns
    -------
    C_lead : control.TransferFunction
        Lead compensator C(s) = Kc (tau s + 1)/(alpha tau s + 1).
    info : dict
        Dictionary with alpha, tau, Kc, and achieved PM.
    """
    # Evaluate plant at desired crossover
    mag, phase, w = ctl.bode(G, [wc_des], Plot=False)
    mag = mag[0]
    phase_deg = phase[0] * 180.0 / np.pi

    PM0 = 180.0 + phase_deg
    phi_req = PM_des_deg - PM0 + safety_deg

    # Clamp phi_req to a reasonable range
    phi_req = np.clip(phi_req, 5.0, 60.0)
    phi_max = np.deg2rad(phi_req)

    # Compute alpha from phi_max:
    # sin(phi_max) = (1 - alpha)/(1 + alpha)
    s = np.sin(phi_max)
    alpha = (1.0 - s) / (1.0 + s)

    # tau from omega_m = 1/(tau * sqrt(alpha)) approx wc_des
    tau = 1.0 / (wc_des * np.sqrt(alpha))

    # Lead compensator without final gain
    num_lead = [tau, 1.0]
    den_lead = [alpha * tau, 1.0]
    C0 = ctl.TransferFunction(num_lead, den_lead)

    # Determine Kc so that |Kc C0(j wc_des) G(j wc_des)| = 1
    mag_C0G, _, _ = ctl.bode(C0 * G, [wc_des], Plot=False)
    mag_C0G = mag_C0G[0]
    Kc = 1.0 / mag_C0G

    C_lead = Kc * C0

    # Evaluate margins
    GM, PM, wcg, wcp = ctl.margin(C_lead * G)

    info = dict(alpha=alpha, tau=tau, Kc=Kc,
                PM_ach_deg=PM,
                wc_ach=wcp)
    return C_lead, info

# Example: G(s) = 1/(s(s+1))
s = ctl.TransferFunction.s
G = 1 / (s * (s + 1))

C_lead, info = design_lead_bode(G, PM_des_deg=50.0, wc_des=4.0)
L = C_lead * G

print("Lead parameters:")
for k, v in info.items():
    print(f"{k}: {v}")

# For a robotic joint, G could be identified from experiments and then used similarly.
# Example: run bode plot and step response
ctl.bode(L)
ctl.step_response(ctl.feedback(L, 1))
