import time
import numpy as np
import serial  # pyserial driver

# ---------------- Driver Layer ----------------
class IMUDriver:
    def __init__(self, port="/dev/ttyUSB0", baud=115200, alpha=1.0, beta=0.0):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
        self.alpha = alpha
        self.beta = beta
        self.y_prev = 0.0
        self.lam = 0.2  # filter coefficient (0 < lam <= 1)

    def read_raw(self):
        line = self.ser.readline().decode().strip()
        # raw sample s_k as float; robust parsers are recommended in real drivers
        return float(line) if line else None

    def read(self):
        s_k = self.read_raw()
        if s_k is None:
            return None
        # calibration: y = alpha s + beta
        y_k = self.alpha * s_k + self.beta
        # simple low-pass filtering
        y_k = (1 - self.lam) * self.y_prev + self.lam * y_k
        self.y_prev = y_k
        return y_k

# ---------------- Middleware Layer (Pub/Sub) ----------------
# ultra-lightweight local pub/sub using a shared buffer
class Topic:
    def __init__(self):
        self.msg = None
        self.t = None

    def publish(self, msg):
        self.msg = msg
        self.t = time.time()

    def latest(self):
        return self.msg, self.t

imu_topic = Topic()

# ---------------- Application Layer ----------------
def control_application():
    # pretend we are doing a proportional controller u = -K y
    K = 0.8
    while True:
        y, t_msg = imu_topic.latest()
        if y is not None:
            u = -K * y
            print(f"[APP] y={y:.3f}, u={u:.3f}, age={time.time()-t_msg:.3f}s")
        time.sleep(0.01)

# wiring layers together
imu = IMUDriver()
def driver_task():
    while True:
        y = imu.read()
        if y is not None:
            imu_topic.publish(y)
        time.sleep(0.005)  # T_d = 5 ms

# In practice, run driver_task and control_application in separate threads/processes.
      
