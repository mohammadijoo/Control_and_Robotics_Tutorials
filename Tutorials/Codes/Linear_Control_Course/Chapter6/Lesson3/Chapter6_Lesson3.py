import numpy as np

def second_order_step_response(t, zeta, wn):
    """
    Underdamped (0 < zeta < 1) standard second-order unit step response.
    """
    wd = wn * np.sqrt(1.0 - zeta**2)
    phi = np.arctan2(np.sqrt(1.0 - zeta**2), zeta)
    return 1.0 - (np.exp(-zeta * wn * t) / np.sqrt(1.0 - zeta**2)) * np.sin(wd * t + phi)

def transient_metrics_from_samples(t, y, band=0.02):
    """
    Compute tr (10-90%), tp, Mp, ts from sampled step response.
    t, y: 1D numpy arrays.
    band: settling band (e.g. 0.02 for 2%).
    """
    t = np.asarray(t).flatten()
    y = np.asarray(y).flatten()
    assert t.shape == y.shape

    # Estimate final value as mean of last 10% of samples
    n = len(t)
    n_tail = max(1, n // 10)
    c_inf = np.mean(y[-n_tail:])

    # Rise time: first crossing of 0.1*c_inf and 0.9*c_inf
    low = 0.1 * c_inf
    high = 0.9 * c_inf
    t10 = None
    t90 = None
    for tk, ck in zip(t, y):
        if t10 is None and ck >= low:
            t10 = tk
        if t90 is None and ck >= high:
            t90 = tk
            break
    tr = None
    if t10 is not None and t90 is not None:
        tr = t90 - t10

    # Peak time and maximum overshoot
    idx_max = np.argmax(y)
    tp = t[idx_max]
    Mp = (y[idx_max] - c_inf) / c_inf

    # Settling time using band around c_inf
    ts = t[-1]
    tol = band * abs(c_inf)
    # Scan from the beginning to find the smallest time after which all samples stay in band
    for k in range(n):
        if np.all(np.abs(y[k:] - c_inf) <= tol):
            ts = t[k]
            break

    return dict(tr=tr, tp=tp, Mp=Mp, ts=ts, c_inf=c_inf)

if __name__ == "__main__":
    # Example: zeta = 0.4, wn = 10 rad/s
    zeta = 0.4
    wn = 10.0
    t = np.linspace(0.0, 3.0, 3000)
    y = second_order_step_response(t, zeta, wn)
    metrics = transient_metrics_from_samples(t, y, band=0.02)
    print("Transient metrics:")
    for k, v in metrics.items():
        print(f"{k} = {v:.4f}")

    # Optional: verify using python-control library if installed
    try:
        import control
        sys = control.TransferFunction([wn**2], [1.0, 2.0*zeta*wn, wn**2])
        t2, y2 = control.step_response(sys, T=t)
        metrics2 = transient_metrics_from_samples(t2, y2, band=0.02)
        print("Metrics (python-control step):", metrics2)
    except ImportError:
        print("python-control not installed; skipping library-based check.")
