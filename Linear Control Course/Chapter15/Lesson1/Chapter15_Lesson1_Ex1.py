import control as ctrl

J = 0.01; B = 0.1; K = 1.0; Kp = 5.0

s = ctrl.TransferFunction.s
G = K / (J * s**2 + B * s)
C = Kp
L = C * G

# Nyquist plot (uses similar sampling internally)
ctrl.nyquist_plot(L)
