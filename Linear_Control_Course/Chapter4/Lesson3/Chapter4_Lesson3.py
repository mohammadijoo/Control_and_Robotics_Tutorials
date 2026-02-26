import control as ctl  # python-control library

# Example plant and controller:
# Gp(s) = Km / (J s^2 + B s)
J = 0.02   # kg m^2
B = 0.1    # N m s/rad
Km = 0.5   # N m/A
Kp = 20.0  # proportional gain
Kenc = 1.0 # encoder gain (unity for simplicity)

# Define s as Laplace variable and construct transfer functions
s = ctl.TransferFunction.s
Gp = Km / (J*s**2 + B*s)
Gc = Kp
H  = Kenc

# Forward path and closed-loop transfer function
G = Gc * Gp
T = ctl.feedback(G, H)  # T(s) = G(s)/(1 + G(s)H(s))

print("Plant Gp(s):", Gp)
print("Controller Gc(s):", Gc)
print("Closed-loop T(s):", T)

# Step response (position tracking)
t, y = ctl.step_response(T)
# t and y can be plotted with matplotlib
