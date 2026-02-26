
import numpy as np

# Motor parameters (example)
T_s = 2.0       # stall torque [Nm]
w0 = 400.0      # no-load speed [rad/s]
eta = 0.85

# Geometry and material (example)
r = 0.6         # lever arm to payload [m]
L = 0.8         # link length [m]
E = 70e9        # Young modulus [Pa] (aluminum)
I = 2.5e-7      # second moment [m^4]
g = 9.81

def joint_curve(N, wj):
    # motor speed = N*wj
    wm = N*wj
    Tm = T_s*(1 - wm/w0)
    Tm = np.maximum(Tm, 0.0)
    return eta*N*Tm

def payload_limit(N, wj):
    Tj = joint_curve(N, wj)
    return Tj/(r*g)   # max (m_p+m_l) at that speed

def stiffness(E, I, L):
    return 3*E*I / (L**3)

def deflection(m_payload):
    F = m_payload*g
    return F*(L**3)/(3*E*I)

Ns = np.array([20, 40, 80, 120])
wj = np.linspace(0, 8, 200)

for N in Ns:
    mp_max = payload_limit(N, wj)
    print(f"N={N}, max payload at zero speed ~ {mp_max[0]:.2f} kg, max speed ~ {w0/N:.2f} rad/s")

k = stiffness(E, I, L)
print("Link stiffness k =", k, "N/m")
print("Deflection for 5 kg payload =", deflection(5.0), "m")
      