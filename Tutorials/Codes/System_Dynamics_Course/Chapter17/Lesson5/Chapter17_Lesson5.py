# Chapter17_Lesson5.py
import numpy as np
import matplotlib.pyplot as plt

np.random.seed(17)

# ------------------------------------------------------------
# Monte Carlo simulation of a stochastic mass-spring-damper system
# m x'' + c x' + k x = F(t)
# Randomness:
#   - k, c (uncertain parameters)
#   - x(0), v(0) (random initial conditions)
#   - F(t_n) piecewise-constant Gaussian forcing
# ------------------------------------------------------------

T = 10.0
dt = 0.005
N = 2000
m = 1.0

# Nominal parameters and uncertainties
k_mean, k_std = 25.0, 2.0
c_mean, c_std = 1.5, 0.2

# Random initial conditions
x0_mean, x0_std = 0.0, 0.05
v0_mean, v0_std = 0.0, 0.05

# Random forcing (piecewise-constant samples)
sigma_F = 2.0

# Quantity of interest: exceedance probability of peak displacement
x_threshold = 0.75

t = np.arange(0.0, T + dt, dt)
nt = len(t)

sum_x = np.zeros(nt)
sum_x2 = np.zeros(nt)
peak_exceed_count = 0

def rk4_step(x, v, force, k, c, h):
    def f(state):
        xs, vs = state
        dx = vs
        dv = -(k / m) * xs - (c / m) * vs + force / m
        return np.array([dx, dv], dtype=float)

    y = np.array([x, v], dtype=float)
    k1 = f(y)
    k2 = f(y + 0.5 * h * k1)
    k3 = f(y + 0.5 * h * k2)
    k4 = f(y + h * k3)
    yn = y + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return yn[0], yn[1]

for trial in range(N):
    # Truncate to keep physical parameters positive
    k = max(1e-6, np.random.normal(k_mean, k_std))
    c = max(1e-6, np.random.normal(c_mean, c_std))

    x = np.random.normal(x0_mean, x0_std)
    v = np.random.normal(v0_mean, v0_std)

    x_hist = np.zeros(nt)
    x_hist[0] = x

    for n in range(nt - 1):
        F_n = np.random.normal(0.0, sigma_F)
        x, v = rk4_step(x, v, F_n, k, c, dt)
        x_hist[n + 1] = x

    sum_x += x_hist
    sum_x2 += x_hist * x_hist

    if np.max(np.abs(x_hist)) > x_threshold:
        peak_exceed_count += 1

mean_x = sum_x / N
var_x = (sum_x2 - N * mean_x**2) / (N - 1)
var_x = np.maximum(var_x, 0.0)
std_x = np.sqrt(var_x)

# 95% CI for the final-time mean displacement using CLT
z975 = 1.959963984540054
se_final = std_x[-1] / np.sqrt(N)
ci_final = (mean_x[-1] - z975 * se_final, mean_x[-1] + z975 * se_final)

p_hat = peak_exceed_count / N
se_p = np.sqrt(max(p_hat * (1.0 - p_hat), 1e-12) / N)
ci_p = (max(0.0, p_hat - z975 * se_p), min(1.0, p_hat + z975 * se_p))

print("Monte Carlo trajectories:", N)
print("Final mean x(T):", mean_x[-1])
print("Final variance x(T):", var_x[-1])
print("95% CI for E[x(T)]:", ci_final)
print("P(max |x| > %.3f): %.6f" % (x_threshold, p_hat))
print("95% CI for probability:", ci_p)

# Save time history statistics for post-processing
data = np.column_stack([t, mean_x, var_x, std_x])
np.savetxt(
    "Chapter17_Lesson5_python_results.csv",
    data,
    delimiter=",",
    header="t,mean_x,var_x,std_x",
    comments=""
)

# Plot mean and ±2 std envelope
plt.figure(figsize=(8, 4.5))
plt.plot(t, mean_x, label="Mean displacement")
plt.plot(t, mean_x + 2*std_x, "--", label="Mean + 2 std")
plt.plot(t, mean_x - 2*std_x, "--", label="Mean - 2 std")
plt.xlabel("Time (s)")
plt.ylabel("Displacement x(t)")
plt.title("Monte Carlo response statistics")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig("Chapter17_Lesson5_python_plot.png", dpi=160)
plt.show()
