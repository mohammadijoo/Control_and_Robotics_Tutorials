import numpy as np
from control import tf, step_response  # python-control, common in robotics

def roots_with_multiplicity(den, tol=1e-6):
    """
    Compute roots of a real polynomial with approximate multiplicity.
    den: list of coefficients [a_n, ..., a_0]
    """
    r = np.roots(den)
    used = np.zeros(len(r), dtype=bool)
    clusters = []
    for i in range(len(r)):
        if used[i]:
            continue
        cluster = [r[i]]
        used[i] = True
        for j in range(i + 1, len(r)):
            if not used[j] and abs(r[j] - r[i]) < tol:
                used[j] = True
                cluster.append(r[j])
        clusters.append((np.mean(cluster), len(cluster)))
    return clusters  # list of (representative_root, multiplicity)

def classify_system(den, tol=1e-6):
    roots_mult = roots_with_multiplicity(den, tol)
    unstable = False
    marginal = False
    for s, m in roots_mult:
        if np.real(s) > tol:
            unstable = True
        elif abs(np.real(s)) <= tol:
            if m >= 2:
                unstable = True  # repeated imaginary-axis root
            else:
                marginal = True  # simple imag-axis root
    if unstable:
        return "unstable", roots_mult
    elif marginal:
        return "marginally stable", roots_mult
    else:
        return "asymptotically stable", roots_mult

# Example 1: (s^2 + 1)^2 => repeated imaginary-axis roots
den1 = [1, 0, 2, 0, 1]
cls1, info1 = classify_system(den1)
print("Example 1:", cls1)
for r, m in info1:
    print(" root =", r, " multiplicity =", m)

# Example 2: (s^2 + 1)(s + 1)(s + 2) => simple imag-axis pair
den2 = [1, 3, 3, 3, 2]
cls2, info2 = classify_system(den2)
print("\nExample 2:", cls2)
for r, m in info2:
    print(" root =", r, " multiplicity =", m)

# Robot joint with inertia J and viscous friction b
J = 0.01
b = 0.0   # set to zero => double integrator
num_joint = [1]
den_joint = [J, 0, 0]  # J s^2
cls3, info3 = classify_system(den_joint)
print("\nJoint example:", cls3)

# Build a transfer function and visualize step response
joint_sys = tf(num_joint, den_joint)
t, y = step_response(joint_sys)
print("Step response final value ~", y[-1])  # grows without bound for double integrator
