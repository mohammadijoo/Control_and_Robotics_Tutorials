import sympy as sp
from control import tf, ss, series, minreal, forced_response

# --- Exact algebraic cancellation (symbolic) ---
s = sp.Symbol("s")
G1 = (s + 1) / (s + 2)
G2 = (s + 2) / (s + 3)
G = sp.simplify(G2 * G1)          # symbolic simplification
G_cancel = sp.cancel(G2 * G1)     # exact cancellation in rational function

print("G(s) simplified:", G)      # should be (s+1)/(s+3)
print("G(s) canceled:", G_cancel) # same

# --- Control objects: build cascade and compare with minreal ---
G1c = tf([1, 1], [1, 2])          # (s+1)/(s+2)
G2c = tf([1, 2], [1, 3])          # (s+2)/(s+3)
G_series = series(G2c, G1c)       # cascade interconnection
G_min = minreal(G_series, verbose=False)  # cancels near pole-zero pairs numerically

print("G_series:", G_series)
print("G_min:", G_min)

# --- One possible (nonminimal) cascade state-space realization ---
# Realizations consistent with: (s+1)/(s+2) = 1 - 1/(s+2) and (s+2)/(s+3) = 1 - 1/(s+3)

A1, B1, C1, D1 = [[-2.0]], [[1.0]], [[-1.0]], [[1.0]]  # v = -x1 + u
A2, B2, C2, D2 = [[-3.0]], [[1.0]], [[-1.0]], [[1.0]]  # y = -x2 + v

S1 = ss(A1, B1, C1, D1)  # u -> v
S2 = ss(A2, B2, C2, D2)  # v -> y
S_cascade = series(S2, S1)

# Minimal realization (numerical) for comparison
S_min = minreal(S_cascade, verbose=False)

# --- Compare output responses (they match), but internal state dimension differs ---
import numpy as np
T = np.linspace(0.0, 10.0, 1000)
U = np.ones_like(T)  # step input

t1, y1 = forced_response(S_cascade, T=T, U=U)
t2, y2 = forced_response(S_min, T=T, U=U)

print("Cascade order:", S_cascade.nstates, "Minimal order:", S_min.nstates)
print("Max |y_cascade - y_min|:", np.max(np.abs(y1 - y2)))
      
