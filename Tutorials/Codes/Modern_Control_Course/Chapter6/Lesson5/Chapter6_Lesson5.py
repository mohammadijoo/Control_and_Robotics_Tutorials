import numpy as np

# Libraries for modern control computations
import control as ct

# Example: G(s) = (s+1)/((s+1)(s+2)) = 1/(s+2)
s = ct.TransferFunction.s
G_nonreduced = (s + 1) / ((s + 1) * (s + 2))
G_reduced = 1 / (s + 2)

print("G_nonreduced =", G_nonreduced)
print("G_reduced    =", G_reduced)

# Minimal realization (state-space) from reduced TF
sys_min = ct.ss(G_reduced)  # realization produced by library (will be minimal here)
print("Minimal order n =", sys_min.nstates)

# Construct a nonminimal realization by adding a hidden state x_h:
# x_hdot = -1 * x_h, with no input coupling and no output coupling.
A = np.array(sys_min.A, dtype=float)
B = np.array(sys_min.B, dtype=float)
C = np.array(sys_min.C, dtype=float)
D = np.array(sys_min.D, dtype=float)

Ah = np.array([[-1.0]])
A_aug = np.block([[A,               np.zeros((A.shape[0], 1))],
                  [np.zeros((1, A.shape[1])), Ah]])
B_aug = np.vstack([B, np.zeros((1, B.shape[1]))])
C_aug = np.hstack([C, np.zeros((C.shape[0], 1))])
D_aug = D.copy()

sys_nonmin = ct.ss(A_aug, B_aug, C_aug, D_aug)
print("Nonminimal order n =", sys_nonmin.nstates)

# Compare transfer functions
G_from_min = ct.tf(sys_min)
G_from_nonmin = ct.tf(sys_nonmin)

print("TF from minimal realization   =", G_from_min)
print("TF from nonminimal realization =", G_from_nonmin)

# Optional: "minreal" to remove cancellations numerically (tolerance-based)
G_minreal = ct.minreal(G_from_nonmin, verbose=False)
print("minreal(TF from nonminimal) =", G_minreal)

# Check frequency responses match
w = np.logspace(-2, 2, 50)
mag1, phase1, _ = ct.freqresp(sys_min, w)
mag2, phase2, _ = ct.freqresp(sys_nonmin, w)

max_mag_err = np.max(np.abs(mag1 - mag2))
max_phase_err = np.max(np.abs(phase1 - phase2))
print("Max magnitude error:", max_mag_err)
print("Max phase error:", max_phase_err)
      
