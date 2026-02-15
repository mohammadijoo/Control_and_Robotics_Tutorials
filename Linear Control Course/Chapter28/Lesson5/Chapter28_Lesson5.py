import json
import platform
from datetime import datetime

import numpy as np
import control as ct   # python-control library
import matplotlib.pyplot as plt

# ------------------------------
# 1. Configuration (single source of truth)
# ------------------------------
config = {
    "experiment_name": "pi_design_second_order",
    "plant": {
        "num": [1.0],
        "den": [1.0, 2.0, 1.0]  # (s + 1)^2
    },
    "controller": {
        "structure": "PI",
        "Kp": 4.0,
        "Ki": 3.0
    },
    "simulation": {
        "t_final": 10.0,
        "n_points": 2000
    },
    "tolerances": {
        "settling_band": 0.02
    },
    "environment": {
        "python_version": platform.python_version(),
        "control_version": ct.__version__,
        "numpy_version": np.__version__
    }
}

# ------------------------------
# 2. Construct plant and controller from config
# ------------------------------
G = ct.TransferFunction(config["plant"]["num"], config["plant"]["den"])
Kp = config["controller"]["Kp"]
Ki = config["controller"]["Ki"]
C = ct.TransferFunction([Kp, Ki], [1.0, 0.0])  # Kp + Ki / s

# Closed-loop with unity feedback
L = C * G
T = ct.feedback(L, 1.0)

# ------------------------------
# 3. Simulate step response and compute metrics
# ------------------------------
t = np.linspace(0.0, config["simulation"]["t_final"], config["simulation"]["n_points"])
t_out, y_out = ct.step_response(T, T=t)

# Overshoot and steady-state error
y_final = y_out[-1]
M_p = float(np.max(y_out) - 1.0)
e_ss = float(1.0 - y_final)

# Settling time: last time when |y - 1| is greater than band
band = config["tolerances"]["settling_band"]
mask_outside = np.abs(y_out - 1.0) > band
if np.any(mask_outside):
    last_outside_index = np.where(mask_outside)[0][-1]
    t_s = float(t_out[last_outside_index])
else:
    t_s = 0.0

metrics = {
    "overshoot": M_p,
    "steady_state_error": e_ss,
    "settling_time": t_s
}

# ------------------------------
# 4. Persist results (JSON) for later reuse
# ------------------------------
result_record = {
    "timestamp": datetime.utcnow().isoformat() + "Z",
    "config": config,
    "metrics": metrics
}

with open("pi_design_second_order_result.json", "w", encoding="utf-8") as f:
    json.dump(result_record, f, indent=2)

# ------------------------------
# 5. Generate reproducible plots
# ------------------------------
plt.figure()
plt.plot(t_out, y_out)
plt.axhline(1.0, linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Output y(t)")
plt.title("Closed-loop step response (PI-controlled second-order plant)")
plt.grid(True)
plt.tight_layout()
plt.savefig("pi_design_second_order_step_response.png", dpi=150)
plt.close()

# Note: In a robotics context, G could represent the linearized transfer from
# joint torque to joint angle, obtained for instance from roboticstoolbox or
# another robotics library. The same script structure still guarantees that
# given the same config, metrics and plots are reproducible.
