import time
import threading
import numpy as np

class HeartbeatMonitor:
    def __init__(self, timeout_s=0.5):
        self.timeout_s = timeout_s
        self.last_hb = time.time()
        self.lock = threading.Lock()
        self.alive = True

    def heartbeat(self):
        with self.lock:
            self.last_hb = time.time()

    def watch(self):
        while self.alive:
            time.sleep(self.timeout_s / 4)
            with self.lock:
                dt = time.time() - self.last_hb
            if dt > self.timeout_s:
                print("[WATCHDOG] heartbeat timeout; switching to safe mode.")
                # Here you would restart or trigger safe-limited controller
                self.alive = False

# Residual-based fault detector for y_k = C x_hat_k + noise
class ResidualDetector:
    def __init__(self, S, gamma):
        self.Sinv = np.linalg.inv(S)
        self.gamma = gamma

    def check(self, y, y_hat):
        r = y - y_hat
        g = float(r.T @ self.Sinv @ r)
        if g > self.gamma:
            return True, g
        return False, g

# Example usage
monitor = HeartbeatMonitor(timeout_s=0.3)
threading.Thread(target=monitor.watch, daemon=True).start()

S = np.eye(2)
gamma = 9.21  # ~95% quantile of chi^2_2
detector = ResidualDetector(S=S, gamma=gamma)

for k in range(10):
    # Simulated heartbeat from component
    if k < 6:
        monitor.heartbeat()
    # Simulated residual check
    y = np.random.randn(2)
    y_hat = np.zeros(2)
    fault, score = detector.check(y, y_hat)
    if fault:
        print(f"[FAULT] residual score={score:.2f}")
    time.sleep(0.1)
      
