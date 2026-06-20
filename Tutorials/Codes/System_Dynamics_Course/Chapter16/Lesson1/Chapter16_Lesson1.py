\
# Chapter16_Lesson1.py
"""
Sampling, aliasing, and zero-order hold demo for Chapter 16 Lesson 1.
Generates:
  1) time-domain plot (continuous, samples, and ZOH output)
  2) magnitude spectrum plot (continuous and sampled)
  3) CSV file with sampled and held values
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def alias_frequency(f0, fs):
    """Return folded alias frequency in [0, fs/2]."""
    m = int(np.round(f0 / fs))
    fa = abs(f0 - m * fs)
    if fa > fs / 2:
        fa = fs - fa
    return fa


# Parameters
fs = 80.0                  # sampling frequency [Hz]
Ts = 1.0 / fs
f1 = 12.0                  # in-band component
f2 = 55.0                  # above Nyquist -> aliases to 25 Hz for fs=80
A1, A2 = 1.0, 0.7
duration = 0.25            # seconds

# "Continuous-time" dense grid for visualization
fdense = 5000.0
td = np.arange(0.0, duration, 1.0 / fdense)
x_cont = A1 * np.sin(2 * np.pi * f1 * td) + A2 * np.sin(2 * np.pi * f2 * td)

# Samples
n = np.arange(0, int(duration * fs))
ts = n * Ts
x_samp = A1 * np.sin(2 * np.pi * f1 * ts) + A2 * np.sin(2 * np.pi * f2 * ts)

# Zero-order hold reconstruction on dense grid
# For each dense-time point t, hold the most recent sample x[k] for kTs <= t < (k+1)Ts
k_idx = np.floor(td / Ts).astype(int)
k_idx = np.clip(k_idx, 0, len(x_samp) - 1)
x_zoh = x_samp[k_idx]

# FFT for spectral picture (zero-pad for smooth display)
Nfft = 4096
Xc = np.fft.rfft(x_cont, n=Nfft)
Xs = np.fft.rfft(x_samp, n=Nfft)
fc = np.fft.rfftfreq(Nfft, d=1.0 / fdense)
fspec = np.fft.rfftfreq(Nfft, d=1.0 / fs)

# Save sampled and held values to CSV
out_dir = Path(".")
csv_path = out_dir / "Chapter16_Lesson1_samples.csv"
with csv_path.open("w", encoding="utf-8") as f:
    f.write("t_dense,x_cont,x_zoh\n")
    for t_i, xc_i, xh_i in zip(td, x_cont, x_zoh):
        f.write(f"{t_i:.8f},{xc_i:.8f},{xh_i:.8f}\n")
    f.write("\n")
    f.write("k,ts,x_sample\n")
    for k, t_k, x_k in zip(n, ts, x_samp):
        f.write(f"{k},{t_k:.8f},{x_k:.8f}\n")

print(f"Alias of {f2:.1f} Hz at fs={fs:.1f} Hz is {alias_frequency(f2, fs):.1f} Hz")
print(f"CSV saved: {csv_path.resolve()}")

# Plot 1: Time-domain waveform and ZOH
plt.figure(figsize=(10, 4.8))
plt.plot(td, x_cont, label="Continuous (dense reference)")
plt.step(td, x_zoh, where="post", label="ZOH output")
plt.plot(ts, x_samp, "o", label="Samples")
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("Sampling and Zero-Order Hold")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("Chapter16_Lesson1_time.png", dpi=150)

# Plot 2: Spectra
plt.figure(figsize=(10, 4.8))
plt.plot(fc, np.abs(Xc), label="Dense-grid spectrum")
plt.plot(fspec, np.abs(Xs), label="Sampled-sequence spectrum")
plt.xlim(0, 120)
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude (unnormalized)")
plt.title("Aliasing in the Frequency Domain")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("Chapter16_Lesson1_spectrum.png", dpi=150)

plt.show()
