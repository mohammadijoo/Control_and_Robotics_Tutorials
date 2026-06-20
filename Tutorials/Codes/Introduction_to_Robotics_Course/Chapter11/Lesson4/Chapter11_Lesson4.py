import logging
import numpy as np
from collections import deque

# --- Logging setup ---
logging.basicConfig(
    filename="robot_run.log",
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s"
)
logger = logging.getLogger("robot")

# --- Monitoring window ---
W = 50
health_window = deque(maxlen=W)

# --- Example LTI observer residual test ---
A = np.array([[1.0, 0.01],
              [0.0, 1.0]])
B = np.array([[0.0],
              [0.01]])
C = np.array([[1.0, 0.0]])
L = np.array([[0.2],
              [2.0]])

S = np.array([[0.04]])          # nominal residual covariance
S_inv = np.linalg.inv(S)
gamma = 6.635                   # chi-square(1, 0.99) threshold (alpha=0.01)

xhat = np.zeros((2,1))

def step(u_k, y_k, cpu_load):
    global xhat

    # Log raw signals
    logger.info(f"u={float(u_k):.3f}, y={float(y_k):.3f}, cpu={cpu_load:.2f}")

    # Monitoring metric with moving average
    health_window.append(cpu_load)
    cpu_avg = sum(health_window) / len(health_window)
    if cpu_avg > 0.9:
        logger.warning(f"High CPU average: {cpu_avg:.2f}")

    # Observer update
    r_k = y_k - C @ xhat
    xhat = A @ xhat + B @ u_k + L @ r_k

    # Diagnostic statistic
    J_k = float(r_k.T @ S_inv @ r_k)
    if J_k > gamma:
        logger.error(f"Fault detected: J={J_k:.3f} > gamma={gamma:.3f}")

# Example usage
for k in range(100):
    u = np.array([[0.1]])
    y = np.array([[0.0]])  # pretend measurement
    cpu = 0.5
    step(u, y, cpu)
      
