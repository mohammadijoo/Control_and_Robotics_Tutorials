import numpy as np
from scipy.optimize import curve_fit
from scipy import signal
import matplotlib.pyplot as plt

# --- Logistic model for adoption of service robots (historical trend idea) ---
def logistic(t, K, r, t0):
    return K / (1 + np.exp(-r*(t - t0)))

# synthetic historical data (replace with real data if desired)
t = np.array([0, 5, 10, 15, 20, 25, 30])   # years since baseline
y = np.array([0.5, 1.2, 2.8, 6.0, 11.0, 17.5, 22.0])  # arbitrary units

popt, _ = curve_fit(logistic, t, y, p0=[25, 0.2, 15])
K_hat, r_hat, t0_hat = popt
print("Estimated K, r, t0:", popt)

tt = np.linspace(0, 30, 300)
plt.plot(t, y, 'o', label="data")
plt.plot(tt, logistic(tt, *popt), label="logistic fit")
plt.xlabel("years"); plt.ylabel("adoption (scaled)")
plt.legend(); plt.show()

# --- Tremor filtering example ---
fs = 200.0
t2 = np.arange(0, 2, 1/fs)
# intended slow motion + 10 Hz tremor
x_m = 0.5*np.sin(2*np.pi*0.5*t2) + 0.05*np.sin(2*np.pi*10*t2)

tau = 0.08  # time constant
b, a = signal.butter(1, 1/(2*np.pi*tau) / (fs/2))  # 1st-order low-pass
x_filt = signal.filtfilt(b, a, x_m)

plt.plot(t2, x_m, label="raw master")
plt.plot(t2, x_filt, label="filtered")
plt.legend(); plt.show()
      