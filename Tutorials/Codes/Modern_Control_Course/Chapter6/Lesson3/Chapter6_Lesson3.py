import numpy as np

# Optional: pip install control
import control as ct

# Original realization for G(s) = (s + 2) / (s^2 + 3 s + 2)
A = np.array([[0.0, 1.0],
              [-2.0, -3.0]])
B = np.array([[0.0],
              [1.0]])
C = np.array([[2.0, 1.0]])
D = np.array([[0.0]])

sys1 = ct.ss(A, B, C, D)

# Similarity transform z = T x (T invertible)
T = np.array([[1.0, 1.0],
              [0.0, 1.0]])
Ti = np.linalg.inv(T)

A2 = T @ A @ Ti
B2 = T @ B
C2 = C @ Ti
D2 = D.copy()

sys2 = ct.ss(A2, B2, C2, D2)

# Compare transfer functions
tf1 = ct.tf(sys1)
tf2 = ct.tf(sys2)

print("tf1 =", tf1)
print("tf2 =", tf2)

# Compare numerator/denominator arrays (SISO)
num1, den1 = ct.tfdata(tf1)
num2, den2 = ct.tfdata(tf2)

num1 = np.squeeze(num1)
den1 = np.squeeze(den1)
num2 = np.squeeze(num2)
den2 = np.squeeze(den2)

print("num1:", num1, " den1:", den1)
print("num2:", num2, " den2:", den2)

# Frequency response check (sample a grid)
w = np.logspace(-2, 2, 50)
mag1, phase1, _ = ct.freqresp(sys1, w)
mag2, phase2, _ = ct.freqresp(sys2, w)

err_mag = np.max(np.abs(mag1 - mag2))
err_phase = np.max(np.abs(phase1 - phase2))
print("Max magnitude error:", err_mag)
print("Max phase error:", err_phase)

# Augmentation test: append hidden stable dynamics
Ah = np.array([[-5.0]])   # hidden pole at -5 (does not appear in G(s))
A_aug = np.block([
    [A, np.zeros((2,1))],
    [np.zeros((1,2)), Ah]
])
B_aug = np.vstack([B, np.zeros((1,1))])
C_aug = np.hstack([C, np.zeros((1,1))])
D_aug = D.copy()

sys_aug = ct.ss(A_aug, B_aug, C_aug, D_aug)
tf_aug = ct.tf(sys_aug)

print("tf_aug =", tf_aug)  # should match tf1 exactly (up to numerical formatting)
      
