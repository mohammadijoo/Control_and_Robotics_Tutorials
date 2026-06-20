import numpy as np

# --- Task requirement profiles (example) ---
# peak load torque and speed
tau_L_peak = 35.0      # Nm
omega_L_peak = 4.0     # rad/s
alpha_L_peak = 12.0    # rad/s^2
J_L = 0.9              # kg m^2

# desired duty cycle represented by an example torque waveform
tau_m_samples = np.array([0, 10, 30, 10, 0])  # Nm after gearing
tau_rms_req = np.sqrt(np.mean(tau_m_samples**2))

# bandwidth and stiffness needs (qualitative scalar here)
bw_req = 8.0           # rad/s (rough target)
k_out_req = 500.0      # Nm/rad

# --- Candidate actuator catalog (motor + gear) ---
catalog = [
    dict(name="BLDC-A", tau_cont=12, tau_peak=40, omega_max=80, J_m=0.0008, mass=1.2, cost=300, eta=0.90),
    dict(name="Servo-B", tau_cont=20, tau_peak=60, omega_max=40, J_m=0.0016, mass=2.2, cost=550, eta=0.85),
    dict(name="Hydro-C", tau_cont=50, tau_peak=120, omega_max=25, J_m=0.0030, mass=6.0, cost=900, eta=0.70),
]

eta_g = 0.92
g_candidates = [5, 8, 12, 16]

def feasible(motor, g):
    omega_m_peak = g * omega_L_peak
    tau_m_peak = (tau_L_peak/(g*eta_g)) + g*motor["J_m"]*alpha_L_peak
    return (tau_m_peak <= motor["tau_peak"] and
            omega_m_peak <= motor["omega_max"] and
            tau_rms_req <= motor["tau_cont"])

feasible_designs = []
for m in catalog:
    for g in g_candidates:
        if feasible(m, g):
            feasible_designs.append((m["name"], g, m))

print("Feasible designs:")
for name, g, m in feasible_designs:
    print(name, "gear", g)

# --- weighted ranking among feasible set ---
w_mass, w_cost, w_eta = 0.4, 0.4, 0.2
def score(m):
    return w_mass*m["mass"] + w_cost*(m["cost"]/1000.0) - w_eta*m["eta"]

ranked = sorted(feasible_designs, key=lambda x: score(x[2]))
print("\nRanked feasible designs:")
for name, g, m in ranked:
    print(f"{name} gear {g} score={score(m):.3f}")
      
